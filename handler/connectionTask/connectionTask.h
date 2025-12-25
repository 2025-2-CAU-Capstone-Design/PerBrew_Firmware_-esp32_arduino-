#pragma once
#include<Arduino.h>
#include "../common/data_format.h"
#include <ArduinoJson.h>

// ===== 부팅 테스크 =====
void BleConnectionTask(void* pv);
void WIFIConnectionTask(void* pv);
void connectionSupervisorTask(void* pv);