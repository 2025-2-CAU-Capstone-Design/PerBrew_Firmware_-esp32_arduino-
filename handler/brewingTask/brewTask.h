#pragma once

#include <Arduino.h>
#include "./driverTask.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#ifdef ARDUINO_ARCH_ESP32
#endif

void BrewTask(void* pv);