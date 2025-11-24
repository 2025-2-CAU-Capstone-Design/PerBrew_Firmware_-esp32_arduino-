#include "./connectionTask.h"


void BleConnectionTask(void* pv) {
    const byte maxRetry = 5;
    byte currentTry = 0;

    ConnectionContext* ctx = (ConnectionContext*)pv;
    BLEConnectionManager* ble = ctx->ble;
    BootManager* bootManager = ctx->boot;
    HttpConnectionManager* wifi = ctx->wifi;

    ble->begin();
    while(true) {
        //bleManger.poll();

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
                ble->stop();
                vTaskDelay(200 / portTICK_PERIOD_MS);
                vTaskDelete(NULL);
            }
       
            if (currentTry >= maxRetry) {
                Serial.println("[BLE] WiFi Failed. Resetting credentials.");
                ble->sendMessage("WiFi_fail");  // 추가됨
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
void WiFiConnectionTask(void* pv) {
    ConnectionContext* ctx = (ConnectionContext*)pv;
    BLEConnectionManager* ble = ctx->ble;
    BootManager* bootManager = ctx->boot;
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
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
