#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

class BLEConnectionManager {
public:
    BLEConnectionManager();

    // BLE 초기화
    void begin();

    // RTOS Task에서 주기적으로 호출 - 혹시 몰라서남겨둠
    void poll();

    // BLE 연결 여부
    bool isConnected() {
        return connected;
    };

    // WiFi SSID/PW 가져오기
    bool hasReceivedCredentials(){
        return credentialsReceived;
    };
    String getSSID() {
        return receivedSSID;
    };
    String getPassword() {
        return receivedPassword;
    };

    // BLE 끄기
    void stop();

private:
    bool connected;
    bool credentialsReceived;

    String receivedSSID;
    String receivedPassword;

    // UUID
    const char* SERVICE_UUID      = "12345678-0000-0000-0000-000000000000";
    const char* CHAR_RX_UUID      = "12345678-1111-0000-0000-000000000000";
    const char* CHAR_TX_UUID      = "12345678-2222-0000-0000-000000000000";

    NimBLEServer* server;
    NimBLECharacteristic* rxChar;
    NimBLECharacteristic* txChar;

    // 내부 콜백 클래스
    class ServerCallbacks : public NimBLEServerCallbacks {
        public:
            void onConnect(NimBLEServer* s){
               BLEConnectionManager::instance->connected = true;
               Serial.println("[BLE] Device connected"); 
            }
            void onDisconnect(NimBLEServer* s) {
                BLEConnectionManager::instance->connected = false;
                Serial.println("[BLE] Device disconnected");
                NimBLEDevice::startAdvertising();
            }
    };

    class RXCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* c);
    };

    // 콜백에서 static 없이 접근할 수 있도록
    static BLEConnectionManager* instance;
};
