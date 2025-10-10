#pragma once
#include "../pin_map.h"
#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>

enum class HeaterState {
    IDLE,
    HEATING,
    MAINTAINING,
    COMPLETED,
    ERROR
};

class heaterDriver {
    public:
        heaterDriver();
        ~heaterDriver() {delete pidController;};
        void begin();

        // == 온도 설정 및 제어 == 
        void startHeating(double targetTemp); 
        void stopHeating(); 
        void update(); 

        // == 상태 확인 ==
        HeaterState getState() const { return heaterState_; }
        bool isHeating() const { return heaterState_ == HeaterState::HEATING || heaterState_ == HeaterState::MAINTAINING; }
        bool isCompleted() const { return heaterState_ == HeaterState::COMPLETED; }
        bool isTemperatureStable(double tolerance = 1.0) const;

        double readThermistor();                                // 즉시 온도 읽기
        double readTemperatureAverage(int samples = 10);        // 평균 온도 (노이즈 제거)
        
        // not used
        double getCurrentTemperature() const { return currentTemperature_; }
        double getTargetTemperature() const { return targetTemperature_; }
        int getCurrentPWM() const { return lastPWMOutput_; }

    private:
        // == 상태 변수 ==
        HeaterState heaterState_ = HeaterState::IDLE;
        unsigned long heatingStartTime_ = 0;
        unsigned long lastUpdateTime_ = 0;
        unsigned long lastPIDUpdateTime_ = 0;
        unsigned long lastSafetyCheckTime_ = 0;
        int consecutiveErrors_ = 0;
        int lastPWMOutput_ = 0;
        int stableCount_ = 0;

        double currentTemperature_ = 0.0;
        double targetTemperature_ = 0.0;
        PID* pidController;
        
        // 핀 설정
        const int HeaterPWMPin = pin::HeaterPWM; // Example pin number
        const int TempSensorPin = pin::HeaterThermistorADC; // Example analog pin for temperature sensor

        void updateHeatingState();                              // 상태 머신 업데이트
        void handleTemperatureControl();                        // 온도 제어 처리
        void handleSafetyCheck();                               // 안전 체크 처리
};
