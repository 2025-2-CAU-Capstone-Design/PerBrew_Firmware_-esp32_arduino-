#pragma once

#include <Arduino.h>
#include "../shared/shared_state.h"

#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#ifdef ARDUINO_ARCH_ESP32
#endif

// === Step Once === 
void runRinse(DriverContext* driver, RecipeInfo& recipe);
void runGrind(DriverContext* driver, RecipeInfo& recipe);
void runBrew(DriverContext* driver, RecipeInfo& recipe);

// === Stop All ===
void stopAll(DriverContext* driver);

// === Main Brew FSM ===
void BrewTask(void* pv);


