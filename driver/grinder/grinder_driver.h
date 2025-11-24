#pragma once
#include "../pin_map.h"
#include "Arduino.h"
#include <AccelStepper.h>

enum class GrinderState {
    IDLE,               //대기 상태
    GRINDING,           // 분쇄 중
    COMPLETED,          // 완료됨
    ERROR               // 오류
};

class GrinderDriver {
    public:
        GrinderDriver();
        void begin();

        // === 그라인더 위치 설정 ===
        bool setClicks(int clicks);
        // Calibration: how many stepper steps correspond to one 'click' of the potentiometer
        void setStepsPerClick(unsigned int stepsPerClick) { stepsPerClick_ = stepsPerClick; }
            
        // === 현재 클릭 값 반환 ===
        int getCurrentClicks(); 
        int getCurrentADC() { return readADCValue(); }

        // === 상태 확인 ===
        GrinderState getState() const { return grindingState_; }
        bool isGrinding() const { return grindingState_ == GrinderState::GRINDING; }
        bool isCompleted() const { return grindingState_ == GrinderState::COMPLETED; }

        // === 전류센서 관련 함수 ===
        int readCurrentSensor();           // 현재 전류값 읽기
        int readCurrentAverage(int samples = 10); // 평균 전류값 읽기
        bool isGrindingComplete(int threshold = 50); // 그라인딩 완료 여부 확인
         
        // === 분쇄 제어 ===
        void startGrinding(uint32_t max_duration_ms = 10000);
        void stopGrinding();
        void update();          // task에서 주기적 호출

    private:
        // == 상태 변수 == 
        int currentClicks_ = 0.0f;
        AccelStepper stepper_;
        float kMaxSpeed_      = 1200.0f;   // steps/s
        float kAcceleration_  = 800.0f;    // steps/s^2   
        
        GrinderState grindingState_ = GrinderState::IDLE;
        unsigned long grindingStartTime_ = 0;
        unsigned long maxGrindingDuration_ = 0;
        unsigned long lastCurrentCheckTime_ = 0;
        int stableIdleCount_ = 0;
        int lastCurrentReading_ = 0;

        // === 스탭모터 + 포텐셜미터 핀 === 
        const int ClickStepPin = pin::ClickMotorStep;           
        const int ClickDirPin = pin::ClickMotorDir;
        const int ClickPotADC = pin::ClickMotorPotentiometerADC;        
        //  === 그라인딩 모터 + 전류센서 핀 === 
        const int GrindPWMPin = pin::GrindingMoterPWM;
        const int GrindMoterADC = pin::GrindingMoterADC;

        // calibration for open-loop movement (steps per potentiometer click)
        unsigned int stepsPerClick_ = 64; // default, tune per hardware
        unsigned int filterWindow_ = 5; // moving average window for ADC filtering

        //  === 내부 함수 구현 === 
        int adcToClicks(int adc);       // ADC -> 클릭 수 변환
        int clicksToADC(int clicks);    // 클릭 수 -> ADC 변환
        long readADCValue() { 
            //Serial.println(analogRead(ClickPotADC));
            return analogRead(ClickPotADC); 
        }    // 포텐셜미터 ADC 읽기
        void updateGrindingState();  // 상태 머신 업데이트
        void handleCurrentMonitoring();  // 전류 모니터링 처리
};
