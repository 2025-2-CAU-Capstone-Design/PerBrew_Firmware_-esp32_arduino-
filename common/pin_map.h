#pragma once
#include <Arduino.h>
/**
 * @brief Pin definitions for the project
 * 
 * All pins are defined as constexpr for compile-time optimization.
 * Ported from original ESP-IDF mappings. Some pins are shared or reassigned
 * based on hardware changes (e.g., repurposed for pouring DC motor).
 * 
 * Note: There are intentional pin shares/conflicts in the current mapping:
 *   - Pin 18: Used by both RearrangingMotorStep and PouringRotationDir
 *   - Pin 15: Used by PouringRotationPWM and PouringDC (likely the same PWM signal)
 *   - Pin 13: Used by PouringMotorEndStop (ExtraEndStopDI commented out)
 */
namespace pin {

    // -----------------------------
    // LCD (I2C)
    // -----------------------------
    constexpr uint8_t LCD_SDA = 21;
    constexpr uint8_t LCD_SCL = 22;

    // -----------------------------
    // Weighing Sensor (HX711)
    // -----------------------------
    constexpr uint8_t Weighing_DT  = 33;
    constexpr uint8_t Weighing_SCK = 32;

    // -----------------------------
    // Rearranging Motor (Carriage)
    // -----------------------------
    constexpr uint8_t Rearranging_Motor_Dir   = 17;  // Shares with Grinding DIR in original map
    constexpr uint8_t Rearranging_Motor_Step  = 18;
    constexpr uint8_t Rearranging_EndStop_DI  = 26;

    // -----------------------------
    // Grinding Section Motors
    // -----------------------------
    constexpr uint8_t Grinding_Motor_PWM          = 27;
    constexpr uint8_t Grinding_Motor_ADC          = 34;
    constexpr uint8_t Click_Motor_Dir             = 14;
    constexpr uint8_t Click_Motor_Step            = 12;
    constexpr uint8_t Click_Motor_Potentiometer_ADC = 36;

    // -----------------------------
    // Pouring (Nozzle) Motors
    // -----------------------------
    constexpr uint8_t Pouring_Angle_Dir    = 4;   // Strap pin
    constexpr uint8_t Pouring_Angle_Step   = 16;
    constexpr uint8_t Pouring_Rotation_PWM = 15;  // DC motor PWM
    constexpr uint8_t Pouring_Rotation_Dir = 18;  // Note: shares pin 18 with Rearranging_Motor_Step
    constexpr uint8_t Pouring_Motor_EndStop = 13;

    // Pouring DC motor (repurposed from ultrasonic)
    constexpr uint8_t Pouring_DC = 15;  // Same as Pouring_Rotation_PWM

    // -----------------------------
    // Pump & Heater
    // -----------------------------
    constexpr uint8_t Pump_PWM           = 19;  // PWM via LEDC
    constexpr uint8_t Heater_PWM         = 23;
    constexpr uint8_t Heater_Thermistor_ADC = 39;  // ADC1 only

    // -----------------------------
    // Commented / Unused pins
    // -----------------------------
    // constexpr uint8_t Ultrasonic_Trig = 5;   // Strap pin (repurposed)
    // constexpr uint8_t Ultrasonic_Echo = 15;  // Repurposed for Pouring_DC
    // constexpr uint8_t Extra_EndStop_DI = 13; // Conflicts with Pouring_Motor_EndStop

}  // namespace pin

/*
namespace pin {
    // Pin definitions (ported from ESP-IDF mapping; values kept as-is)
    // LCD (I2C)
    constexpr uint8_t LCD_SDA = 21;
    constexpr uint8_t LCD_SCL = 22;

    // Weighing Sensor (HX711)
    constexpr uint8_t WeighingDT = 33;
    constexpr uint8_t WeightingSCK = 32;

    // Rearranging Motor (Carriage)
    constexpr uint8_t RearraingMotorDir = 17; // NOTE: shares with Grinding DIR in original map
    constexpr uint8_t RearraingMotorStep = 18;
    constexpr uint8_t RearraingEndStopDI = 35;

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
    constexpr uint8_t PouringMotorEndStop = 13; 



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
}*/