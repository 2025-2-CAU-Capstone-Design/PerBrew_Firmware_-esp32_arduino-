#pragma once
#include "../pin_map.h"
#include <Ultrasonic.h>
#include <AccelStepper.h>

// 설정 상수들
namespace PouringConfig {
    const float DEFAULT_STEP_SPEED = 800.0f;       // 스텝/초
    const float DEFAULT_ACCELERATION = 400.0f;     // 스텝/초^2
    const int ULTRASONIC_TIMEOUT_MS = 30;          // 초음파 센서 타임아웃
    const float SOUND_SPEED_CM_US = 0.034f;        // 음속 (cm/us)
    const int MIN_DISTANCE_CM = 2;                 // 최소 측정 거리
    const int MAX_DISTANCE_CM = 200;               // 최대 측정 거리
    const int PUMP_MIN_PWM = 0;                    // 펌프 최소 PWM
    const int PUMP_MAX_PWM = 255;                  // 펌프 최대 PWM
}

enum class PouringError {
    NONE = 0,
    ULTRASONIC_TIMEOUT,
    DISTANCE_OUT_OF_RANGE,
    PUMP_ERROR,
    STEPPER_ERROR
};

struct PouringStatus {
    bool isRotating = false;
    bool isTilting = false;
    bool isPumping = false;
    long currentDistance = -1;
    int currentPumpPWM = 0;
    PouringError lastError = PouringError::NONE;
};

class PouringSectionDriver {
    public:
        PouringSectionDriver();
        ~PouringSectionDriver();
        
        // 초기화 및 설정
        bool begin();
        void setStepperSpeed(float speed);
        void setStepperAcceleration(float acceleration);
        
        // 거리 측정
        long measureDistanceCM();
        long measureDistanceMM() { return measureDistanceCM() * 10; }
        bool isObjectInRange(int minCM = 5, int maxCM = 50);
        
        /*
        // 노즐 회전 제어 (AccelStepper 사용)
        void rotateNozzle(int steps, bool clockwise = true);
        void rotateNozzleToAngle(float degrees); // 각도 기반 회전
        void stopRotation();
        */
        // 노즐 회전 제어 (DC 모터 사용)
        void startRotation(bool clockwise, int pwmValue);
        void stopRotation();
        bool isRotating() const { return rotationRunning_; }
        
        // 노즐 틸트 제어
        void tiltNozzle(long distanceCM); // 거리 기반 틸트
        void tiltNozzleToAngle(float degrees); // 각도 기반 틸트
        void stopTilt();
        
        // 펌프 제어 (개선된 버전)
        void startPump(int pwmValue = 128);
        void stopPump();
        void setPumpSpeed(int pwmValue); // 0-255
        void setPumpSpeedPercent(float percent); // 0-100%
        bool isPumpRunning() const { return status_.isPumping; }
        
        // 상태 및 에러 관리
        PouringStatus getStatus() const { return status_; }
        PouringError getLastError() const { return status_.lastError; }
        const char* getErrorString() const;
        void clearError() { status_.lastError = PouringError::NONE; }
        
        // 업데이트 함수 (메인 루프에서 호출)
        void update();
        
        // 레거시 호환성
        int getDuration() const { return duration_; }
        void pumping() { startPump(); } // 기존 함수명 호환성

    private:
        // 핀 정의
        const int TrigPin = pin::UltrasonicTrig;
        const int EchoPin = pin::UltrasonicEcho;
        const int PumpPin = pin::PumpPWM;
        const int PouringRotationDir = pin::PouringRotationDir;
        const int PouringRotationPWM = pin::PouringRotationPWM;
        const int PouringTiltStepPin = pin::PouringAngleStep;
        const int PouringTiltDirPin = pin::PouringAngleDir;
        bool rotationRunning_ = false;

        // 스텝 모터 객체
        AccelStepper rotationStepper_;
        AccelStepper tiltStepper_;
        
        // 상태 관리
        PouringStatus status_;
        int duration_ = 0; // 레거시 호환성
        
        // 내부 함수
        void setError(PouringError error);
        bool validateDistance(long distance);
        void updateSteppers();
};