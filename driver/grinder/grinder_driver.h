#pragma once
#include "../pin_map.h"
#include "Arduino.h"
#include <AccelStepper.h>

class GrinderDriver {
    public:
        GrinderDriver();
        void begin();
        bool setBaseCalibration(uint32_t timeout_ms = 10000); // 홈+캘리브레이션
        void setClicks(int clicks);
        // 현재 ADC를 클릭 수로 변환
        int convertADCtoClicks();
        
        // 클릭 목표로 이동
        int getCurrentClicks() const { return currentClicks_; }
        void grind(uint32_t duration_ms = 10000);
    private:
        int readADCAvg(int pin, int samples);
        int adcToClicks(int adc);
        int clicksToADC(int clicks);
        int getPulsePerClick();


        int currentClicks_ = 0;
        float stepsPerClick_ = 1.0f;
        // 스탭모터 + 포텐셜미터 핀
        const int ClickStepPin = pin::ClickMotorStep;           
        const int ClickDirPin = pin::ClickMotorDir;
        const int ClickEndStopPin = pin::ClickMotorEndStop;
        const int ClickPotADC = pin::ClickMotorPotentiometerADC;
        
        // 그라인딩 모터 + 전류센서 핀
        const int GrindPWMPin = pin::GrindingMoterPWM;
        const int GrindMoterADC = pin::GrindingMoterADC;

        AccelStepper stepper_;
        float kMaxSpeed_      = 1200.0f;   // steps/s
        float kAcceleration_  = 800.0f;    // steps/s^2
        float kHomeSpeedFast_ = -600.0f;   // 홈 방향(음수) 빠른 접근
        float kHomeSpeedSlow_ = -150.0f;   // 홈 방향(음수) 저속 재접근
        int   kBackoffSteps_  = 60;        // 백오프 스텝       
};
