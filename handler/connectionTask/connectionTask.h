#pragma once
#include<Arduino.h>
#include "../driver/data_format.h"
#include <ArduinoJson.h>


#ifdef ARDUINO_ARCH_ESP32
#endif

// ===== 부팅 테스크 =====
void BleConnectionTask(void* pv);
void WIFIConnectionTask(void* pv);
void ConnectionSupervisorTask(void* pv);