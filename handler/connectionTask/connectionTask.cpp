#include "./connectionTask.h"
#include "../common/connection/BLEconnection.h"
#include "../common/connection/WIFIconnection.h"
#include "../common/boot.h"
#include "../common/data_format.h"
/*
    String machine_id = "ESP32_001"; // 실제 머신 ID 변수 사용
    String url = "/ws/machine/" + machine_id;
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
        ctx->bleModeActive = true;
    }
}


/*
    교착상태 발생...
    vTaskDelete 타이밍과 BLE 테스크 내부 삭제 타이밍이 겹치는 것으로 보임 -> Race Condition

*/
static void stopBLETask(ConnectionContext* ctx) {
    if (ctx->bleTask != nullptr){
        Serial.println("[SUP] Stopping BLE task...");
        ctx->ble->stop();
        //vTaskDelete(ctx->bleTask);
        ctx->bleModeActive = false;
        Serial.println("[SUP] BLE task stopped");
    }
}

static void startWifiTask(ConnectionContext* ctx) {
    if (ctx->wifiTask == nullptr) {
        xTaskCreate( WIFIConnectionTask,"WIFIConnectionTask", 16384, ctx,4,&ctx->wifiTask);
        Serial.println("[SUP] WIFI task started");
        ctx->bleModeActive = false;
    }
}

static void stopWifiTask(ConnectionContext* ctx) {
    if (ctx->wifiTask != nullptr) {
        Serial.println("[SUP] Stopping WIFI task...");
        vTaskDelete(ctx->wifiTask);
        ctx->wifiTask = nullptr;
        if (ctx->wifi) ctx->wifi->disconnect(); 
        WiFi.disconnect(true); 
        WiFi.mode(WIFI_OFF);
        Serial.println("[SUP] WIFI task stopped & Radio OFF");
        ctx->bleModeActive = true;
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
        if(!ctx->bleModeActive){
            Serial.println("[BLE Task] Stop requested, shutting down BLE...");
            ble->stop();
            ctx->bleTask = nullptr;
            vTaskDelete(NULL);
        }

        ble->poll();
        if (ble->hasReceivedCredentials() && ble->isConnected()) {
            if (currentTry == 0) {
                Serial.println("[BLE Task] Credentials received & Device connected.");
                Serial.println("[BLE Task] Sending 'WiFi_try' to App and starting WiFi connection...");
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

                bootManager->saveWIFICredentials(ble->getSSID(), ble->getPassword(), ble->getUserEmail());
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
                ble->reAdvertise();
                currentTry = 0;
            }
        }
        if (ble->isConnected() && !ble->hasReceivedCredentials()) {
            static unsigned long lastConnectedCheck = 0;
            if (millis() - lastConnectedCheck > 3000) {  // 3초마다 상태 확인
                Serial.println("[BLE Task] Connected but waiting for credentials...");
                lastConnectedCheck = millis();
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
    HttpConnectionManager* webSockets = ctx->wifi;
    //String serverIP = "172.30.1.74";
    //String serverIP = "172.30.1.79";
    String serverIP = "192.168.137.1";
    //String serverIP = "192.168.0.9";
    //String serverIP = "172.30.1.16";
    static sendItem dataToSend;
    webSockets -> begin(serverIP, 8000, ctx->machine_id, ctx->userEmail);
    while(true) {
        //Serial.println("[WIFI TASK] Polling WebSocket...");
        webSockets -> poll();
        if(uxQueueMessagesWaiting(gSendQueue) > 0) {
            if (xQueueReceive(gSendQueue, &dataToSend, 0) == pdPASS) {
                if(!webSockets->sendMessage(dataToSend)) {
                    Serial.println("[WIFI TASK] Data send failed.");
                }
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}



// ===== Supervisor Task =====
void connectionSupervisorTask(void* pv) {
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
        ctx->bleModeActive = false;
    } else {
        Serial.println("[SUP] Boot in BLE mode");
        startBleTask(ctx);
        ctx->bleModeActive = true;
    }

    uint8_t wifiDeadCount = 0;
    uint8_t websocketDeadCount = 0;

    while (true) {
        // 2) BLE -> WiFi 연결 성공 이벤트 처리
        uint32_t notified = 0;
        if (xTaskNotifyWait( 0, 0xFFFFFFFF,&notified,
                WIFI_CHECK_INTERVAL_MS / portTICK_PERIOD_MS ) == pdTRUE){
            if (notified & WIFI_EVT_PROVISIONED) {
                Serial.println("[SUP] WiFi provisioned by BLE, starting WiFi task");
                ctx->bleModeActive = false;
                startWifiTask(ctx);
                stopBLETask(ctx);
                wifiDeadCount = 0;
                websocketDeadCount = 0;
                wifiDeadCount = 0;
                continue;
            }
        }

        // 3) WiFi 상태 체크 -> 오래 죽어 있으면 BLE 재시작
        if (!ctx->bleModeActive) {
        if (ctx->wifi != nullptr) {
            if (WiFi.status() != WL_CONNECTED) {
                wifiDeadCount++;
                Serial.printf("[SUP] try to connect websockets... (%d)\n", wifiDeadCount);
                if (wifiDeadCount >= WIFI_MAX_DEAD_CHECK) {
                    Serial.println("[SUP] WiFi seems dead, restarting BLE provisioning");
                    ctx->bleModeActive = true;
                    boot->clearWiFiCredentials();
                    stopWifiTask(ctx);
                    ctx->wifi = nullptr;  // 추가: ctx 업데이트
                    startBleTask(ctx);
                    wifiDeadCount = 0;
                    websocketDeadCount = 0;
                }
            } else {
                wifiDeadCount = 0;
            }
        }
    }
        if (ctx->wifi != nullptr && !(ctx->wifi->isConnected()) && !ctx->bleModeActive) {
            websocketDeadCount++;
            Serial.printf("[SUP] WebSocket disconnected (%d/%d)\n", websocketDeadCount, WIFI_MAX_DEAD_CHECK * 2);
            ctx->wifi->reconnect();
            if (websocketDeadCount >= WIFI_MAX_DEAD_CHECK * 2) {
                Serial.println("[SUP] WebSocket seems dead, restarting BLE provisioning");
                boot->clearWiFiCredentials();
                stopWifiTask(ctx);
                ctx->wifi = nullptr;  // 추가: ctx 업데이트
                startBleTask(ctx);
                ctx->bleModeActive = true;
                websocketDeadCount = 0;
                wifiDeadCount = 0;
            }
        } else {
            websocketDeadCount = 0;
        }
    }   
}