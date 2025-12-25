#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

class BLEConnectionManager {
public:
    BLEConnectionManager();
    void begin();
    void poll();
    void stop();
    void reAdvertise();

    bool isConnected() const { return connected; }
    bool hasReceivedCredentials() const { return credentialsReceived; }
    
    void sendMessage(const String& msg);
    void clearReceivedCredentials();

    String getSSID() const { return receivedSSID; }
    String getPassword() const { return receivedPassword; }
    String getUserEmail() const { return userEmail; }

private:
    bool connected = false;
    bool credentialsReceived = false;
    String receivedSSID;
    String receivedPassword;
    String userEmail;

    static const char* SERVICE_UUID;
    static const char* CHAR_RX_UUID;
    static const char* CHAR_TX_UUID;

    NimBLEAdvertising* advertising = nullptr;
    NimBLEServer* server = nullptr;
    NimBLECharacteristic* rxCharacteristic = nullptr;
    NimBLECharacteristic* txCharacteristic = nullptr;

    class ServerCallbacks : public NimBLEServerCallbacks {
    public:
        explicit ServerCallbacks(BLEConnectionManager* p) : parent(p) {}
        void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
        void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
    private:
        BLEConnectionManager* parent;
    };

    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    public:
        explicit CharacteristicCallbacks(BLEConnectionManager* p) : parent(p) {}
        void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    private:
        BLEConnectionManager* parent;
    };

    ServerCallbacks serverCallbacks;
    CharacteristicCallbacks characteristicCallbacks;

    void handleConnect(NimBLEServer* pServer);
    void handleDisconnect(NimBLEServer* pServer);
    void handleReceivedData(const std::string& value);
};