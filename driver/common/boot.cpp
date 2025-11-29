#include "boot.h"
#define WIFI_CONNECT_TIMEOUT 10000  // 10초
    
/*
  대대적인 수정
  1. preferences.begin()/end()를 각 메서드에서 호출하도록 변경
     - 메서드 호출 시점에만 Preferences를 열고 닫아야 함.
  2. BLE 모드, WIFI 모드 추가함
     - boot.h/cpp은 첫 부팅 시 eeproom을 확인하는 역할로 한정. 이후 네트워크 관련 테스킹은 ble/wifi를 분리해서 관리
  참고 
    https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html
*/
BootManager::BootManager(){}
String BootManager::begin() {
    preferences.begin("boot", false);
    bool isMachineId = preferences.isKey("machine_id");
    if(!isMachineId){
        machine_id = generatemachine_id();
        preferences.putString("machine_id", machine_id);
        Serial.printf("[Boot] Generated Machine ID: %s\n", machine_id.c_str());
    } else {
        machine_id = preferences.getString("machine_id", "");
        Serial.printf("[Boot] Loaded Machine ID: %s\n", machine_id.c_str());
    }
    bool doesExist = preferences.isKey("ssid");
    preferences.end();
    // WiFi 연결 시도
    if (doesExist) {
        currentMode = WIFI_MODE;
        Serial.println("[Boot] WiFi Connected");
    } else {
        // WiFi 실패 -> BLE 모드
        currentMode = BLE_MODE;
        Serial.println("[Boot] BLE Mode Started");
    }
    
    return currentMode;
}

// not used functions : supervisor에서 직접 접근
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

void BootManager::saveWIFICredentials(const String& ssid, const String& password, const String& userEmail) {
    preferences.begin("boot", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("userEmail", userEmail);
    preferences.end();
    Serial.printf("[Boot] WiFi Saved: %s\n", ssid.c_str());
}


// not used functions : supervisor에서 직접 접근
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

// not used functions : supervisor에서 직접 접근
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

String BootManager::getUserEmail() {
    preferences.begin("boot", true);
    String email = preferences.getString("userEmail", "");
    if(email=="") {
        Serial.println("[Boot] No User Email stored");
        preferences.end();
        return "";
    }
    preferences.end();
    return email;
}


// not used functions : 초기화 코드 반영 안되었음.
void BootManager::clearWiFiCredentials() {
    preferences.begin("boot", false);
    preferences.remove("ssid");
    preferences.remove("password");
    preferences.end();
    Serial.println("[Boot] WiFi Credentials Cleared");
}

String BootManager::getCurrentMode() const {
    return currentMode;
}

String BootManager::getmachine_id() {
    return machine_id;
}

String BootManager::generatemachine_id() {
    String id = "";
    const char* chars = "0123456789ABCDEF";
    for (int i = 0; i < 12; i++) {
        id += chars[random(0, 16)];
    }
    return id;
}