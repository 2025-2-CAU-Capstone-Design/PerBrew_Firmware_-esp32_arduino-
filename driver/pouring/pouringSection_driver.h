#pragma once
#include "../pin_map.h"
#include <AccelStepper.h>

// 설정 상수들
namespace PouringConfig {
    const float DEFAULT_STEP_SPEED = 800.0f;       // 스텝/초
    const float DEFAULT_ACCELERATION = 400.0f;     // 스텝/초^2
    const int PUMP_MIN_PWM = 0;                    // 펌프 최소 PWM
    const int PUMP_MAX_PWM = 255;                  // 펌프 최대 PWM
}

enum class PouringError {
    NONE = 0,
    PUMP_ERROR,
    STEPPER_ERROR
};

struct PouringStatus {
    bool isRotating = false;
    bool isTilting = false;
    bool isPumping = false;
    int currentPumpPWM = 0;
    PouringError lastError = PouringError::NONE;
};

enum class PouringTechnique {
    CENTER,          // 중심부 집중 주수
    //SPIRAL_OUT,      // 바깥쪽 나선형 주수
    //SPIRAL_IN,       // 안쪽 나선형 주수
    SPIRAL,
    PULSE,           // 펄스형 주수
    CIRCULAR,        // 원형 주수
};

class PouringSectionDriver {
    public:
        PouringSectionDriver();
        ~PouringSectionDriver();
        
        // 초기화 및 설정
        bool begin();
        void setStepperSpeed(float speed);
        void setStepperAcceleration(float acceleration);
        
        // 노즐 회전 제어 (DC 모터 사용)
        void startRotation(bool clockwise, int pwmValue);
        void stopRotation();
        bool isRotating() const { return rotationRunning_; }


        // 노즐 틸트 제어
        void stopTilt();            
        void tiltToZero();

        void applyTechnique(PouringTechnique technique);
        void executeTechnique(PouringTechnique technique, float intensity = 1.0f);
        
        void circularPour(float intensity);    // 원형 패턴
        void centerPour(float intensity);      // 고정된 중심 주수
        //void spiralOutPour(float intensity);   // 바깥쪽 나선형 주수
        //void spiralInPour(float intensity);    // 안쪽 나선형 주수
        void spiralPour(float intensity);

        
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
        //const int TrigPin = pin::UltrasonicTrig;
        //const int EchoPin = pin::UltrasonicEcho;
        const int PumpPin = pin::PumpPWM;
        const int PouringRotationDir = pin::PouringRotationDir;
        const int PouringRotationPWM = pin::PouringRotationPWM;
        const int PouringTiltStepPin = pin::PouringAngleStep;
        const int PouringTiltDirPin = pin::PouringAngleDir;
        const int PouringTiltEndStopPin = pin::PouringMotorEndStop; // 재배치 엔드스톱 공유
        bool rotationRunning_ = false;
        bool tiltingDirection_ = true; // true: CW, false: CCW

        // 스텝 모터 객체
        //AccelStepper rotationStepper_;
        AccelStepper tiltStepper_;
        
        // 상태 관리
        PouringStatus status_;
        int duration_ = 0; // 레거시 호환성
        
        // 내부 함수
        void setError(PouringError error);
        //bool validateDistance(long distance);
        //void updateSteppers();
};