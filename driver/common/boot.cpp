#include "boot.h"
#define WIFI_CONNECT_TIMEOUT 10000  // 10초
    
BootManager::BootManager() : currentMode(ConnectionMode::BLE) {}


/*
  대대적인 수정
  1. preferences.begin()/end()를 각 메서드에서 호출하도록 변경
     - 메서드 호출 시점에만 Preferences를 열고 닫음
  2. ensureMacAddress, ensureUserID, ensureServerAddress 메서드 추가
     - 필수 설정값이 없을 경우 기본값을 생성/저장하도록 구현
  3. setMode, getMode 메서드 구현
  참고 
    https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html
*/

ConnectionMode BootManager::begin() {
    preferences.begin("boot", false);
    
    // WiFi 연결 시도
    if (tryConnectWiFi()) {
        currentMode = ConnectionMode::WIFI;
        Serial.println("[Boot] WiFi Connected");
    } else {
        // WiFi 실패 -> BLE 모드
        startBLEMode();
        currentMode = ConnectionMode::BLE;
        Serial.println("[Boot] BLE Mode Started");
    }
    
    return currentMode;
}

 bool BootManager::tryConnectWiFi() {
    preferences.begin("boot", false);
    bool doesExist = preferences.isKey("ssid");
    if(!doesExist) {
        preferences.end();
        return false;
    }
    String ssid = preferences.getString("ssid", "");
    String password = preferences.getString("password", "");
    
    Serial.printf("[Boot] Trying WiFi: %s\n", ssid.c_str());
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT) {
            WiFi.disconnect();
            return false;
        }
        delay(100);
    }
    preferences.end();
    return true;
}

void BootManager::startBLEMode() {
    BLEDevice::init("PerBrew");
    // BLE 서버 설정은 별도 모듈에서 처리
}

void BootManager::saveWIFICredentials(const String& ssid, const String& password) {
    preferences.begin("boot", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();
    Serial.printf("[Boot] WiFi Saved: %s\n", ssid.c_str());
}

String BootManager::getWiFiSSID() {
    preferences.begin("boot", true);
    String ssid = preferences.getString("ssid", "");
    if(ssid=="") {
        Serial.println("[Boot] No WiFi SSID stored");
        preferences.end();
        return "";
    }
    preferences.end();
    return ssid;
}

String BootManager::getWiFiPassword() {
    preferences.begin("boot", true);
    String password = preferences.getString("password", "");
    if(password=="") {
        Serial.println("[Boot] No WiFi Password stored");
        preferences.end();
        return "";
    }
    preferences.end();
    return password;
}

void BootManager::clearWiFiCredentials() {
    preferences.begin("boot", false);
    preferences.remove("ssid");
    preferences.remove("password");
    preferences.end();
    Serial.println("[Boot] WiFi Credentials Cleared");
}

ConnectionMode BootManager::getCurrentMode() const {
    return currentMode;
}

String BootManager::getMachineID() {
    return machineID;
}

String BootManager::generateMachineID() {
    String id = "";
    const char* chars = "0123456789ABCDEF";
    for (int i = 0; i < 12; i++) {
        id += chars[random(0, 16)];
    }
    return id;
}