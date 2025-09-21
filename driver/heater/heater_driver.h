#pragma once
#include "../pin_map.h"
#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>

class heaterDriver {
    public:
        heaterDriver();
        void begin();
        void setTargetTemperature(double target);
        double readThermistor();
        double getCurrentTemperature() {return currentTemperature; };
        void updateValue();
    private:
        double currentTemperature = 0.0;
        double targetTemperature = 0.0;
        PID* pidController;
        const int HeaterPWMPin = pin::HeaterPWM; // Example pin number
        const int TempSensorPin = pin::HeaterThermistorADC; // Example analog pin for temperature sensor
};
