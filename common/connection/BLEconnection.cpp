#include "BLEconnection.h"

const char* BLEConnectionManager::SERVICE_UUID = "12345678-0000-0000-0000-000000000000";
const char* BLEConnectionManager::CHAR_RX_UUID = "12345678-1111-0000-0000-000000000000";
const char* BLEConnectionManager::CHAR_TX_UUID = "12345678-2222-0000-0000-000000000000";

BLEConnectionManager::BLEConnectionManager() 
    : serverCallbacks(this), characteristicCallbacks(this) {
}

void BLEConnectionManager::begin() {
    Serial.println("[BLE] Initializing BLE...");

    NimBLEDevice::init("PerBrew");
    
    server = NimBLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);

    NimBLEService* service = server->createService(SERVICE_UUID);

    // Receive characteristic (앱에서 데이터를 쓰는 특성)
    rxCharacteristic = service->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    rxCharacteristic->setCallbacks(&characteristicCallbacks);

    // Transmit characteristic (ESP32에서 앱으로 알림을 보내는 특성)
    txCharacteristic = service->createCharacteristic(
        CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    service->start();

    advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("[BLE] Advertising started");
}

void BLEConnectionManager::poll() {
    // 연결 상태 업데이트
    if (server) {
        connected = (server->getConnectedCount() > 0);
    }
}

void BLEConnectionManager::reAdvertise() {
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();
    Serial.println("[BLE] Re-advertising started");
}

void BLEConnectionManager::stop() {
    Serial.println("[BLE] Stopping BLE...");
    advertising->stop();
    connected = false;
}

void BLEConnectionManager::sendMessage(const String& msg) {
    if (txCharacteristic && connected) {
        txCharacteristic->setValue(msg.c_str());
        txCharacteristic->notify();
        Serial.printf("[BLE] Message sent to app: %s\n", msg.c_str());
    } else {
        Serial.println("[BLE] Cannot send message: not connected or txCharacteristic is null");
    }
}

void BLEConnectionManager::clearReceivedCredentials() {
    credentialsReceived = false;
    receivedSSID = "";
    receivedPassword = "";
    userEmail = "";
}

void BLEConnectionManager::ServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    if (parent) {
        parent->handleConnect(pServer);
    }
}

void BLEConnectionManager::ServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    if (parent) {
        parent->handleDisconnect(pServer);
    }
}

void BLEConnectionManager::CharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    if (!parent) return;

    std::string value = pCharacteristic->getValue();
    
    Serial.printf("[BLE] onWrite called - data length: %d\n", value.length());
    if (!value.empty()) {
        Serial.printf("[BLE] Received data: %s\n", value.c_str());
        parent->handleReceivedData(value);
    }
}

void BLEConnectionManager::handleConnect(NimBLEServer* pServer) {
    connected = true;
    Serial.println("[BLE] Client connected");
}

void BLEConnectionManager::handleDisconnect(NimBLEServer* pServer) {
    connected = false;
    Serial.println("[BLE] Client disconnected");
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
}

void BLEConnectionManager::handleReceivedData(const std::string& value) {
    if (value.empty()) {
        Serial.println("[BLE] Received empty data");
        return;
    }

    String receivedData(value.c_str());
    receivedData.trim();

    // "prefix:data" 형식 파싱
    int colonIndex = receivedData.indexOf(':');
    if (colonIndex == -1) {
        Serial.println("[BLE] Error: Invalid data format. Expected 'prefix:data'");
        sendMessage("error:invalid_format");
        return;
    }

    String prefix = receivedData.substring(0, colonIndex);
    String data = receivedData.substring(colonIndex + 1);

    Serial.printf("[BLE] Parsed - Prefix: '%s', Data: '%s'\n", prefix.c_str(), data.c_str());

    if (prefix == "s") {
        receivedSSID = data;
        Serial.printf("[BLE] SSID received: '%s' (length: %d)\n", receivedSSID.c_str(), receivedSSID.length());
    }
    else if (prefix == "p") {
        receivedPassword = data;
        Serial.printf("[BLE] Password received (length: %d)\n", receivedPassword.length());
    }
    else if (prefix == "e") {
        userEmail = data;
        credentialsReceived = true;
        Serial.printf("[BLE] Email received: %s\n", userEmail.c_str());
        Serial.println("[BLE] All credentials received");
        sendMessage("ok");
    }
    else {
        Serial.printf("[BLE] Error: Unknown prefix '%s'\n", prefix.c_str());
    }
}