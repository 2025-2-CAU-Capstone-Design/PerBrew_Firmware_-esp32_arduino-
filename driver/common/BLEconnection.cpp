#include "BLEconnection.h"


BLEConnectionManager* BLEConnectionManager::instance = nullptr;

BLEConnectionManager::BLEConnectionManager()
    : connected(false), credentialsReceived(false),
      server(nullptr), rxChar(nullptr), txChar(nullptr) 
{
    instance = this;
}

void BLEConnectionManager::begin() {
    Serial.println("[BLE] Initializing BLE...");

    NimBLEDevice::init("PerBrew-Setup");
    NimBLEDevice::setMTU(64);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService* service = server->createService(SERVICE_UUID);

    // RX (앱 → ESP32)
    rxChar = service->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    rxChar->setCallbacks(new RXCallbacks());

    // TX (ESP32 → 앱)
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

bool BLEConnectionManager::isConnected() {
    return connected;
}

bool BLEConnectionManager::hasReceivedCredentials() {
    return credentialsReceived;
}

String BLEConnectionManager::getSSID() {

}

String BLEConnectionManager::getPassword() {
    return receivedPassword;
}

void BLEConnectionManager::stop() {
    Serial.println("[BLE] Stopping BLE...");
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::deinit(true);
}


void BLEConnectionManager::RXCallbacks::onWrite(NimBLECharacteristic* c) {
    std::string data = c->getValue();
    if (data.length() == 0) return;

    String json = String(data.c_str());
    Serial.printf("[BLE] Received raw: %s\n", json.c_str());

    // 예상 포맷: ssid:password
    int sep = json.indexOf(':');
    if (sep < 0) {
        Serial.println("[BLE] Invalid format");
        return;
    }

    BLEConnectionManager::instance->receivedSSID = json.substring(0, sep);
    BLEConnectionManager::instance->receivedPassword = json.substring(sep + 1);
    BLEConnectionManager::instance->credentialsReceived = true;

    Serial.printf("[BLE] Parsed SSID: %s\n", BLEConnectionManager::instance->receivedSSID.c_str());
    Serial.printf("[BLE] Parsed PW: %s\n", BLEConnectionManager::instance->receivedPassword.c_str());
}