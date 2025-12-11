#pragma once
#include <Arduino.h>

namespace pin {
    // Pin definitions (ported from ESP-IDF mapping; values kept as-is)
    // LCD (I2C)
    constexpr uint8_t LCD_SDA = 21;
    constexpr uint8_t LCD_SCL = 22;

    // Weighing Sensor (HX711)
    constexpr uint8_t WeighingDT = 32;
    constexpr uint8_t WeightingSCK = 33;

    // Rearranging Motor (Carriage)
    constexpr uint8_t RearraingMotorDir = 17; // NOTE: shares with Grinding DIR in original map
    constexpr uint8_t RearraingMotorStep = 18;
    constexpr uint8_t RearraingEndStopDI = 13;

    // Grinding Section Motor
    constexpr uint8_t GrindingMoterPWM = 27;
    constexpr uint8_t GrindingMoterADC = 34;
    constexpr uint8_t ClickMotorDir = 14; 
    constexpr uint8_t ClickMotorStep = 12; 
    constexpr uint8_t ClickMotorPotentiometerADC = 36; 

    // Pouring (Nozzle) Motors
    constexpr uint8_t PouringAngleDir = 4; // strap pin
    constexpr uint8_t PouringAngleStep = 16;
    constexpr uint8_t PouringRotationPWM = 15;  // DC motor PWM pin
    constexpr uint8_t PouringRotationDir = 18;
    constexpr uint8_t PouringMotorEndStop = 35; 



    // Pump (PWM via LEDC)
    constexpr uint8_t PumpPWM = 19;
    // Heater
    constexpr uint8_t HeaterPWM = 23;
    constexpr uint8_t HeaterThermistorADC = 39; // ADC1 input-only


    // Ultrasonic Sensor -> 푸어링 dc 모터
    //constexpr uint8_t UltrasonicTrig = 5; // strap pin
    //constexpr uint8_t UltrasonicEcho = 15; // strap pin
    constexpr uint8_t PouringDC = 15;

    // Extra EndStop
    //constexpr uint8_t ExtraEndStopDI = 13;
}