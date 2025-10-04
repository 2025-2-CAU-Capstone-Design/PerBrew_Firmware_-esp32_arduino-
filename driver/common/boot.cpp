#include "boot.h"

BootConfig::BootConfig() : currentMode(BootMode::BLE) {}

void BootConfig::begin() {
    // "boot_config" namespace 열기
    preferences.begin("boot_config", false);

    // 실행 모드 불러오기 (없으면 BLE 기본값)
    int modeVal = preferences.getInt("mode", static_cast<int>(BootMode::BLE));
    currentMode = static_cast<BootMode>(modeVal);

    // MAC 주소 / UserID / ServerAddress 기본값 확인
    ensureMacAddress("boot_config");
    ensureUserID("boot_config");
    ensureServerAddress("boot_config");

    preferences.end();
}

void BootConfig::setValue(const String& ns, const String& key, const String& value) {
    preferences.begin(ns.c_str(), false);
    preferences.putString(key.c_str(), value);
    preferences.end();
}

String BootConfig::getValue(const String& ns, const String& key, const String& defaultValue) {
    preferences.begin(ns.c_str(), true);
    String val = preferences.getString(key.c_str(), defaultValue);
    preferences.end();
    return val;
}

void BootConfig::ensureMacAddress(const String& ns) {
    preferences.begin(ns.c_str(), false);
    String mac = preferences.getString("mac", "");
    if (mac.isEmpty()) {
        uint64_t chipid = ESP.getEfuseMac();
        char macStr[18];
        sprintf(macStr, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
        preferences.putString("mac", String(macStr));
    }
    preferences.end();
}

void BootConfig::ensureUserID(const String& ns) {
    preferences.begin(ns.c_str(), false);
    if (!preferences.isKey("user_id")) {
        preferences.putString("user_id", "");
    }
    preferences.end();
}

void BootConfig::ensureServerAddress(const String& ns) {
    preferences.begin(ns.c_str(), false);
    if (!preferences.isKey("server_addr")) {
        preferences.putString("server_addr", "example.com");
    }
    preferences.end();
}

void BootConfig::setMode(BootMode mode) {
    preferences.begin("boot_config", false);
    preferences.putInt("mode", static_cast<int>(mode));
    preferences.end();
    currentMode = mode;
}

BootMode BootConfig::getMode() const {
    return currentMode;
}

void BootConfig::reset() {
    preferences.begin("boot_config", false);
    preferences.clear();
    preferences.end();
    currentMode = BootMode::BLE;
}