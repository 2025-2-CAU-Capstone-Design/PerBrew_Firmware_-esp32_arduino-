#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>


class BLEConnectionManager {
public:
    BLEConnectionManager();

    // BLE 초기화
    void begin();

    // RTOS Task에서 주기적으로 호출 - 필요시 확장용
    void poll();

    // BLE 연결 여부
    bool isConnected() const {
        return connected;
    }

    // WiFi SSID/PW 가져오기
    bool hasReceivedCredentials() const {
        return credentialsReceived;
    }

    // 메시지 수신
    void sendMessage(const String& msg) {
        if (txChar) {
            txChar->setValue(msg.c_str());
            txChar->notify();
        }
    }
    
    void clearReceivedCredentials() {
        credentialsReceived = false;
        receivedSSID = "";
        receivedPassword = "";
    }

    String getSSID() const {
        return receivedSSID;
    }
    String getPassword() const {
        return receivedPassword;
    }
    String getUserEmail() const {
        return userEmail;
    }

    // BLE 끄기
    void stop();

private:
    bool connected;
    bool credentialsReceived;

    String receivedSSID;
    String receivedPassword;
    String userEmail;

    // UUID
    const char* SERVICE_UUID = "12345678-0000-0000-0000-000000000000";
    const char* CHAR_RX_UUID = "12345678-1111-0000-0000-000000000000";
    const char* CHAR_TX_UUID = "12345678-2222-0000-0000-000000000000";

    NimBLEServer* server;
    NimBLEAdvertising* ad; 
    NimBLECharacteristic* rxChar;
    NimBLECharacteristic* txChar;

    // 내부 콜백 클래스들
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }
} serverCallbacks;

    class RXCallbacks : public NimBLECharacteristicCallbacks {
    public:
        void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onRead(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
        }

        void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
            Serial.printf("%s : onWrite(), value: %s\n",
                        pCharacteristic->getUUID().toString().c_str(),
                        pCharacteristic->getValue().c_str());
        }

        private:
            BLEConnectionManager* parent;
    };
    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
        void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
            Serial.printf("%s : onRead(), value: %s\n",
                        pCharacteristic->getUUID().toString().c_str(),
                        pCharacteristic->getValue().c_str());
        }

        void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
            Serial.printf("%s : onWrite(), value: %s\n",
                        pCharacteristic->getUUID().toString().c_str(),
                        pCharacteristic->getValue().c_str());
        }
    } charCallbacks;

    // 콜백 객체 (new/delete 안 쓰고 멤버로 보관)
    ServerCallbacks serverCallbacks;
    RXCallbacks     rxCallbacks;
};
