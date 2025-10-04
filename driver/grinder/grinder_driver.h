#pragma once
#include "../pin_map.h"
#include "Arduino.h"
#include <AccelStepper.h>

class GrinderDriver {
    public:
        GrinderDriver();
        void begin();
        // 그라인더 위치 설정
        void setClicks(int clicks);
        
        // 현재 클릭 값 반환
        int getCurrentClicks(); 

        // 분쇄
        void grind(uint32_t duration_ms = 10000);
    
    private:
        // ADC -> 클릭 수 변환
        int adcToClicks(int adc);
        // 클릭 수 -> ADC 변환
        int clicksToADC(int clicks);
        
        // 포텐셜미터 ADC 읽기
        long readADCValue() { return analogRead(ClickPotADC); }

        int currentClicks_ = 0;
        // 스탭모터 + 포텐셜미터 핀
        const int ClickStepPin = pin::ClickMotorStep;           
        const int ClickDirPin = pin::ClickMotorDir;
        const int ClickPotADC = pin::ClickMotorPotentiometerADC;        
        // 그라인딩 모터 + 전류센서 핀
        const int GrindPWMPin = pin::GrindingMoterPWM;
        const int GrindMoterADC = pin::GrindingMoterADC;

        AccelStepper stepper_;
        float kMaxSpeed_      = 1200.0f;   // steps/s
        float kAcceleration_  = 800.0f;    // steps/s^2   
};
