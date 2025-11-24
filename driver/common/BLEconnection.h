#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>


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

    // BLE 끄기
    void stop();

private:
    bool connected;
    bool credentialsReceived;

    String receivedSSID;
    String receivedPassword;

    // UUID
    const char* SERVICE_UUID = "12345678-0000-0000-0000-000000000000";
    const char* CHAR_RX_UUID = "12345678-1111-0000-0000-000000000000";
    const char* CHAR_TX_UUID = "12345678-2222-0000-0000-000000000000";

    NimBLEServer* server;
    NimBLECharacteristic* rxChar;
    NimBLECharacteristic* txChar;

    // 내부 콜백 클래스들
    class ServerCallbacks : public NimBLEServerCallbacks {
    public:
        explicit ServerCallbacks(BLEConnectionManager* parent)
            : parent(parent) {}

        void onConnect(NimBLEServer* s);
        void onDisconnect(NimBLEServer* s);

    private:
        BLEConnectionManager* parent;
    };

    class RXCallbacks : public NimBLECharacteristicCallbacks {
    public:
        explicit RXCallbacks(BLEConnectionManager* parent)
            : parent(parent) {}

        void onWrite(NimBLECharacteristic* c);

    private:
        BLEConnectionManager* parent;
    };

    // 콜백에서 부모 객체로 호출할 내부 핸들러
    void handleConnect(NimBLEServer* s){
        connected = true;
        Serial.println("[BLE] Client connected");
    };
    void handleDisconnect(NimBLEServer* s){
        connected = false;
        Serial.println("[BLE] Client disconnected");
    };
    void handleRX(const std::string& value);

    // 콜백 객체 (new/delete 안 쓰고 멤버로 보관)
    ServerCallbacks serverCallbacks;
    RXCallbacks     rxCallbacks;
};
