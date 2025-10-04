#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <BLEDevice.h>
/*
    부팅 설정 관리 클래스
    BootConfig
    - 설정 파일을 Preferences에 저장
    - 저장 내용:
        1. 실행 모드 (BLE 모드 / WiFi 모드)
        2. 기기 MAC 주소 (없으면 랜덤 생성)
        3. 연결된 사용자 ID
        4. 서버 주소 (기본값: "example.com")
        5. 기타 key-value 설정*/

enum class BootMode{
    BLE,
    WiFi
};

class BootConfig{
    public:
        BootConfig();
        void begin();

        // Key-Value 저장/로드
        void setValue(const String& ns, const String& key, const String& value);
        String getValue(const String& ns, const String& key, const String& defaultValue = "");

        // 필수 설정값 관리
        void ensureMacAddress(const String& ns);
        void ensureUserID(const String& ns);
        void ensureServerAddress(const String& ns);
        // 모드 관리
        void setMode(BootMode mode);
        BootMode getMode() const;

        // 리셋 (모든 값 삭제)
        void reset();

    private:
        Preferences preferences;
        BootMode currentMode;
};