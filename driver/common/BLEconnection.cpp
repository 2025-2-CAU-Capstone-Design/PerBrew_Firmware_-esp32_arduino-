#include "BLEconnection.h"

BLEConnectionManager::BLEConnectionManager()
    : connected(false), credentialsReceived(false),
      server(nullptr), rxChar(nullptr), txChar(nullptr),
      serverCallbacks(this), rxCallbacks(this)
{}

void BLEConnectionManager::begin() {
    Serial.println("[BLE] Initializing BLE...");

    NimBLEDevice::init("PerBrew-Setup");
    NimBLEDevice::setMTU(64);

    server = NimBLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);

    NimBLEService* service = server->createService(SERVICE_UUID);

    // BLE로 앱에 직접 연결... 앱에서 SSID/PW 전송받음
    // RX (앱 -> ESP32)
    rxChar = service->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );

    // TX (ESP32 -> 앱)
    rxChar->setCallbacks(&rxCallbacks);
    txChar = service->createCharacteristic(
        CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    service->start();

    NimBLEAdvertising* ad = NimBLEDevice::getAdvertising();
    ad->addServiceUUID(SERVICE_UUID);
    ad->start();

    Serial.println("[BLE] Advertising started");
}

void BLEConnectionManager::poll() {
    // BLE는 NimBLE 스택이 내부 loop로 처리하므로 별도 작업은 필요 없음
}


void BLEConnectionManager::stop() {
    Serial.println("[BLE] Stopping BLE...");
    NimBLEDevice::stopAdvertising();

    connected = false;
    // dangling 방지
    server = nullptr;
    rxChar = nullptr;
    txChar = nullptr;
}

void BLEConnectionManager::ServerCallbacks::onConnect(NimBLEServer* s) {
    if (parent) {
        parent->handleConnect(s);
    }
}

void BLEConnectionManager::ServerCallbacks::onDisconnect(NimBLEServer* s) {
    if (parent) {
        parent->handleDisconnect(s);
    }
}

void BLEConnectionManager::RXCallbacks::onWrite(NimBLECharacteristic* c) {
    if (!parent) return;
    parent->handleRX(c->getValue());
}

void BLEConnectionManager::handleRX(const std::string& value) {
    if (value.empty()) {
        return;
    }

    String raw = String(value.c_str());
    Serial.printf("[BLE] Received raw: %s\n", raw.c_str());

    // 포맷: ssid:password (추후 JSON 등으로 확장 가능)
    int sep = raw.indexOf(':');
    if (sep < 0) {
        Serial.println("[BLE] Invalid format");
        return;
    }

    receivedSSID     = raw.substring(0, sep);
    receivedPassword = raw.substring(sep + 1);
    credentialsReceived = true;

    Serial.printf("[BLE] Parsed SSID: %s\n", receivedSSID.c_str());
    Serial.printf("[BLE] Parsed PW: %s\n", receivedPassword.c_str());
}