#include "BLEconnection.h"

BLEConnectionManager::BLEConnectionManager()
    : connected(false), credentialsReceived(false),
      server(nullptr), rxChar(nullptr), txChar(nullptr),
      serverCallbacks(this), rxCallbacks(this)
{}

void BLEConnectionManager::begin() {
    Serial.println("[BLE] Initializing BLE...");

    NimBLEDevice::init("PerBrew-Setup");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); 
    NimBLEDevice::setMTU(517);

    server = NimBLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);

    NimBLEService* service = server->createService(SERVICE_UUID);

    // RX Characteristic 생성 - 두 가지 속성을 모두 지원하도록 설정
    rxChar = service->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE
    );

    rxChar->setCallbacks(&rxCallbacks);
    Serial.println("[BLE] RX Characteristic callbacks set");

    txChar = service->createCharacteristic(
        CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );
    Serial.println("[BLE] TX Characteristic created");

    service->start();

    // 광고 설정
    ad = NimBLEDevice::getAdvertising();
    ad->addServiceUUID(SERVICE_UUID);
    NimBLEAdvertisementData scanResponseData;
    scanResponseData.setName("PerBrew-Setup");
    ad->setScanResponseData(scanResponseData);  
    ad->start();

    Serial.println("[BLE] Advertising started");
}


void BLEConnectionManager::poll() {
    if (server != nullptr) {
        bool wasConnected = connected;
        connected = (server->getConnectedCount() > 0);
        
        // 연결 상태가 변경된 경우에만 로그 출력
        if (connected && !wasConnected) {
            Serial.println("[BLE] Client connected successfully");
        }
        else if (!connected && wasConnected) {
            Serial.println("[BLE] Client disconnected");
            // 연결이 끊어졌을 때 광고 재시작
            if (NimBLEDevice::getAdvertising()->isAdvertising() == false) {
                NimBLEDevice::startAdvertising();
                Serial.println("[BLE] Advertising restarted after disconnect");
            }
        }
    }   
    // 주기적으로 연결 상태 출력 (선택사항)
    static unsigned long lastStatusPrint = 0;
    if (!connected && (millis() - lastStatusPrint > 10000)) {  // 10초마다
        Serial.println("[BLE] Advertising... (waiting for connection)");
        lastStatusPrint = millis();
    }
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
    Serial.println("[BLE] onWrite callback triggered");  // 이 로그가 출력되는지 확인
    if (!parent) {
        Serial.println("[BLE] Error: parent is null in onWrite");
        return;
    }
    std::string value = c->getValue();
    Serial.printf("[BLE] Raw data received in onWrite: length=%d\n", value.length());
    parent->handleRX(value);
}

/* s : p : e*/
void BLEConnectionManager::handleRX(const std::string& value) {
    Serial.printf("[BLE] handleRX called with data length: %d\n", value.length());
    
    if (value.empty()) {
        Serial.println("[BLE] Received empty data");
        return;
    }

    String raw = String(value.c_str()); 
    Serial.println("\n[BLE] --------------------------------");
    Serial.printf("[BLE] Message Received: %s\n", raw.c_str());
    Serial.println("[BLE] --------------------------------\n"); 
    int colonIndex = raw.indexOf(':');
    
    if (colonIndex == -1) {
        Serial.println("[BLE] Error: Invalid format. Expected 'prefix:data' format");
        sendMessage("error:invalid_format");
        return;
    }

    String prefix = raw.substring(0, colonIndex);
    String data = raw.substring(colonIndex + 1);

    if (prefix == "s") {
        receivedSSID = data;
        Serial.printf("[BLE] SSID received: %s\n", receivedSSID.c_str());
        sendMessage("success:ssid");
    }
    else if (prefix == "p") {
        receivedPassword = data;
        Serial.printf("[BLE] Password received: %s\n", receivedPassword.c_str());
        sendMessage("success:password");
    }
    else if (prefix == "e") {
        userEmail = data;
        credentialsReceived = true; // 이메일 수신 시 모든 자격증명 수신 완료로 간주
        Serial.printf("[BLE] Email received: %s\n", userEmail.c_str());
        Serial.println("[BLE] All credentials received.");
        sendMessage("success:all");
    }
    else {
        Serial.println("[BLE] Error: Unknown prefix: " + prefix);
        sendMessage("error:unknown_prefix");
    }
}

/*
void BLEConnectionManager::handleRX(const std::string& value) {
    Serial.printf("[BLE] handleRX called with data length: %d\n", value.length());
    
    if (value.empty()) {
        Serial.println("[BLE] Received empty data");
        return;
    }

    String raw = String(value.c_str()); 
    Serial.println("\n[BLE] --------------------------------");
    Serial.printf("[BLE] Message Received: %s\n", raw.c_str());
    Serial.println("[BLE] --------------------------------\n");
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, raw);

    if (error) {
        Serial.print(F("[BLE] deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }

    // JSON에서 값 추출 (키가 없으면 null 반환되므로 안전하게 처리)
    const char* ssid = doc["ssid"];
    const char* password = doc["password"];
    const char* email = doc["email"];

    // String 변수에 할당 (null인 경우 빈 문자열)
    receivedSSID     = ssid ? String(ssid) : "";
    receivedPassword = password ? String(password) : "";
    userEmail        = email ? String(email) : "";

    // SSID가 유효한 경우에만 수신 완료 처리
    if (receivedSSID.length() > 0) {
        credentialsReceived = true;
        Serial.printf("[BLE] Parsed SSID: %s\n", receivedSSID.c_str());
        Serial.printf("[BLE] Parsed Email: %s\n", userEmail.c_str());
    } else {
        Serial.println("[BLE] Error: SSID missing in JSON data");
    }
}*/