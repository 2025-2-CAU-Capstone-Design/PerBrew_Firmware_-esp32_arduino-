#pragma once
#include <Arduino.h>
#include <HX711.h>
#include "../common/pin_map.h"  // Your pin header with namespace 'pin'
#include <ArduinoJson.h>
#include "../shared/shared_state.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>


extern HX711 scale;

// Constants for NTC 100k thermistor (B=3950 typical)
constexpr float THERMISTOR_NOMINAL = 100000.0;   // Resistance at 25°C
constexpr float TEMPERATURE_NOMINAL = 25.0;     // °C
constexpr float B_COEFFICIENT = 3950.0;          // Beta value (common for 100k NTC)
constexpr float SERIES_RESISTOR = 10000.0;       // 10k pull-down resistor
//constexpr float ADC_MAX = 4095.0;                // ESP32 12-bit ADC
//constexpr int ADC_MIN = 0;                 
constexpr float ADC_MAX = 2200.0;   // 0 click
constexpr int ADC_MIN  = 64;        // 240 cllicks
constexpr int CLICK_MIN = 0;
constexpr int CLICK_MAX = 240;
constexpr int BIAS_CLICKS = 0;

// Minimum ADC value

void brewTask(void* pv);
void userInput(void* pv);