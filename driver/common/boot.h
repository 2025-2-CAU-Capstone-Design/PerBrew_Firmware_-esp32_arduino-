#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <nvs_flash.h>
#include "../driver/data_format.h"
    
/*
    부팅 설정 관리 클래스
    BootManager
    - 설정 파일을 Preferences에 저장
    - 저장 내용:
        1. 실행 모드 (BLE 모드 / WiFi 모드)
        2. 기기 MAC 주소 (없으면 랜덤 생성)
        3. 연결된 사용자 ID
        4. 서버 주소 (기본값)
        5. 기타 key-value 설정*/



class BootManager {
public:
    BootManager();
    
    // 부팅 시 자동으로 WiFi 연결 시도 -> 실패 시 BLE 모드
    String begin();
    
    // WiFi 관리
    void saveWIFICredentials(const String& ssid, const String& password, const String& userEmail);
    String getWiFiSSID();
    String getWiFiPassword();
    String getUserEmail();
    // WiFi 자격증명 삭제
    void clearWiFiCredentials();

    // 현재 연결 모드 반환
    String getCurrentMode() const;

    // 머신 ID 설정/조회
    String getmachine_id();

private:
    Preferences preferences;
    String currentMode;
    String machine_id = "";
    
    // WiFi 연결 시도
    bool tryConnectWiFi();

    // 머신 ID 생성
    String generatemachine_id();
};