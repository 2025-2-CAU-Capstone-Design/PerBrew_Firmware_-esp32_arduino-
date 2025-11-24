#pragma once
#include<Arduino.h>
#include "../driver/data_format.h"


#ifdef ARDUINO_ARCH_ESP32
#endif

// ===== 부팅 테스크 =====
void BleConnectionTask(void* pv);
void WifiConnectionTask(void* pv);