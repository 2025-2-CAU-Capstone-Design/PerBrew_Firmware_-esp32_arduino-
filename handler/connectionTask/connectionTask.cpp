#include "./connectionTask.h"
#include "../driver/common/BLEconnection.h"
#include "../driver/common/WIFIconnection.h"
#include "../driver/common/boot.h"


// [C++ 수정 전]
// wsClient.begin(serverIP, 8080, "/ws");

// [C++ 수정 후]
// 1. 포트 확인: FastAPI 기본은 8000입니다. (uvicorn 실행 시 포트 확인)
// 2. 경로 확인: /ws/machine/{머신ID} 형식이어야 합니다.
/*
    String machineId = "ESP32_001"; // 실제 머신 ID 변수 사용
    String url = "/ws/machine/" + machineId;
    wsClient.begin(serverIP, 8000, url); 
*/

extern QueueHandle_t gSendQueue;

static void startBleTask(ConnectionContext* ctx) {
    if (ctx->bleTask == nullptr) {
        xTaskCreate(
            BleConnectionTask,
            "BleConnectionTask",
            8192,
            ctx,
            1,
            &ctx->bleTask
        );
        Serial.println("[SUP] BLE task started");
    }
}

static void startWifiTask(ConnectionContext* ctx) {
    if (ctx->wifiTask == nullptr) {
        xTaskCreate( WIFIConnectionTask,"WIFIConnectionTask", 8192,ctx,1,&ctx->wifiTask);
        Serial.println("[SUP] WIFI task started");
    }
}

void BleConnectionTask(void* pv) {
    const byte maxRetry = 5;
    byte currentTry = 0;

    ConnectionContext* ctx = (ConnectionContext*)pv;
    BLEConnectionManager* ble = ctx->ble;
    BootManager* bootManager = ctx->boot;
    HttpConnectionManager* wifi = ctx->wifi;

    const uint32_t WIFI_EVT_PROVISIONED = (1 << 0);
    ble->begin();

    while(true) {

        if (ble->hasReceivedCredentials() && ble->isConnected()) {
            if (currentTry == 0) {
                ble->sendMessage("WiFi_try");          // 앱에게 연결 시도 알림
                WiFi.mode(WIFI_STA);
                WiFi.begin(ble->getSSID().c_str(), 
                           ble->getPassword().c_str());
            }
               
            unsigned long startTime = millis();
            while (millis() - startTime < 5000){
                if (WiFi.status() == WL_CONNECTED) break;
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            currentTry ++;
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("[BLE] WiFi Connected via credentials!");
                
                ble->sendMessage("WiFi_connected");

                bootManager->saveWIFICredentials(ble->getSSID(), ble->getPassword());
                if (ctx->supervisorTask != nullptr) {
                    xTaskNotify(ctx->supervisorTask, WIFI_EVT_PROVISIONED, eSetBits);
                }
                ble->stop();
                vTaskDelay(200 / portTICK_PERIOD_MS);
                ctx->bleTask = nullptr;
                vTaskDelete(NULL);
            }
       
            if (currentTry >= maxRetry) {
                Serial.println("[BLE] WiFi Failed. Resetting credentials.");
                ble->sendMessage("WiFi_fail");
                ble->clearReceivedCredentials();
                currentTry = 0;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}



// ===== WIFI 연결 테스크 =====
/*
    WiFi 모드로 연결 후 발생하는 이벤트 처리, 및 커맨드를 해석하여 큐에 삽입
    생산자 - 소비자 패턴을 생각해보자
        - WifiConnectionTask : 생산자
        - MainTask : 소비자
        - BrewTask : 소비자
*/
void WIFIConnectionTask(void* pv) {
    ConnectionContext* ctx  = (ConnectionContext*)pv;
    HttpConnectionManager* wifi = ctx->wifi;
    String serverIP = "192.168.1.1";
    
    wifi -> begin(serverIP, 8080);
    while(true) {
        
        if(!wifi -> isConnected()) {
            Serial.println("[WIFI TASK] Not connected. Retrying in 5 seconds...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        wifi -> poll();

        // sendQueue에 데이터가 있으면 전송
        if(uxQueueMessagesWaiting(gSendQueue) > 0) {
            String dataToSend;
            if (xQueueReceive(gSendQueue, &dataToSend, 0) == pdTRUE) {
                wifi->sendMessage(dataToSend);
                Serial.println("[WIFI TASK] Sent data: " + dataToSend);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


// ===== Supervisor Task =====
void ConnectionSupervisorTask(void* pv) {
    ConnectionContext* ctx = (ConnectionContext*)pv;
    BootManager* boot      = ctx->boot;

    const uint32_t WIFI_EVT_PROVISIONED = (1 << 0);
    const uint32_t WIFI_CHECK_INTERVAL_MS = 5000;
    const uint8_t  WIFI_MAX_DEAD_CHECK = 6; // ~30초

    ctx->supervisorTask = xTaskGetCurrentTaskHandle();

    // 1) 부팅 모드에 따른 초기 Task 선택
    String mode = boot->begin();
    if (mode == WIFI_MODE) {
        Serial.println("[SUP] Boot in WiFi mode");
        startWifiTask(ctx);
    } else {
        Serial.println("[SUP] Boot in BLE mode");
        startBleTask(ctx);
    }

    uint8_t wifiDeadCount = 0;

    while (true) {
        // 2) BLE -> WiFi 연결 성공 이벤트 처리
        uint32_t notified = 0;
        if (xTaskNotifyWait(
                0,
                0xFFFFFFFF,
                &notified,
                WIFI_CHECK_INTERVAL_MS / portTICK_PERIOD_MS
            ) == pdTRUE)
        {
            if (notified & WIFI_EVT_PROVISIONED) {
                Serial.println("[SUP] WiFi provisioned by BLE, starting WiFi task");
                startWifiTask(ctx);
            }
        }

        // 3) WiFi 상태 체크 → 오래 죽어 있으면 BLE 재시작
        if (ctx->wifi != nullptr) {
            if (!ctx->wifi->isConnected()) {
                wifiDeadCount++;
                Serial.printf("[SUP] WiFi disconnected (%d)\n", wifiDeadCount);
                if (wifiDeadCount >= WIFI_MAX_DEAD_CHECK) {
                    Serial.println("[SUP] WiFi seems dead, restarting BLE provisioning");
                    startBleTask(ctx);
                    wifiDeadCount = 0;
                }
            } else {
                wifiDeadCount = 0;
            }
        }
    }
}