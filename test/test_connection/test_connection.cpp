#include <Arduino.h>
#include <WiFi.h>
#include "../../driver/data_format.h"

// .h 대신 .cpp를 포함하여 구현부를 컴파일 단위에 포함시킵니다.
#include "../../driver/common/boot.cpp"
#include "../../driver/common/BLEconnection.cpp"
#include "../../driver/common/WIFIconnection.cpp"

// === 테스트용 전역 객체 ===
BootManager bootManager;
BLEConnectionManager bleManager;
HttpConnectionManager wifiManager;

// === 전역 큐 및 드라이버 컨텍스트 정의 (WIFIconnection.cpp에서 참조함) ===
QueueHandle_t gRecipeQueue = nullptr;
QueueHandle_t gCommandQueue = nullptr;
QueueHandle_t gSendQueue = nullptr;
DriverContext driver;

// === 상태 정의 ===
enum class TestState {
    IDLE,               // 대기
    BLE_LISTENING,      // BLE 모드: 앱 연결 및 정보 수신 대기
    WIFI_CONNECTING,    // WiFi AP 연결 시도 중
    SERVER_CONNECTING,  // 웹소켓 서버 연결 시도 중
    CONNECTED           // 서버 연결 완료 (통신 가능)
};

TestState currentState = TestState::IDLE;
String targetServerIP = "192.168.56.1"; // 테스트할 서버 IP (메뉴에서 변경 가능)
const uint16_t targetServerPort = 8000;  // FastAPI 기본 포트

// === 함수 선언 ===
void printMenu();
void startBootSequence();
void handleBLEState();
void handleWiFiConnectingState();
void handleServerConnectingState();
void handleConnectedState();
void clearInputBuffer();
void handleForcedWifiMode();

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n\n========================================");
    Serial.println("   PerBrew Connection Flow Test");
    Serial.println("========================================");
    
    // 큐 초기화 (WIFIconnection.cpp에서 사용)
    gRecipeQueue  = xQueueCreate(2, sizeof(RecipeInfo));
    gCommandQueue = xQueueCreate(10, sizeof(String));
    gSendQueue    = xQueueCreate(20, sizeof(String));

    printMenu();
}

void loop() {
    // 1. 사용자 입력 처리
    if (Serial.available()) {
        int choice = Serial.parseInt();
        clearInputBuffer();

        switch (choice) {
            case 1:
                startBootSequence();
                break;
            case 2:
                Serial.println("[CMD] Clearing WiFi Credentials...");
                bootManager.clearWiFiCredentials();
                Serial.println("[CMD] Done. Next boot will be BLE mode.");
                break;
            case 3: {
                Serial.println("[CMD] Enter Server IP (e.g., 192.168.56.1):");
                while (!Serial.available()) delay(10);
                targetServerIP = Serial.readStringUntil('\n');
                targetServerIP.trim();
                Serial.printf("[CMD] Server IP set to: %s\n", targetServerIP.c_str());
                break;
            }
            case 4:
                if (currentState == TestState::CONNECTED) {
                    String msg = "{\"type\":\"TEST\",\"machineID\":\"TEST_DEV\",\"data\":\"Hello\"}";
                    sendItem sendData;
                    SAFE_COPY_TO_BUFFER(sendData, msg);
                    wifiManager.sendMessage(sendData);
                    Serial.println("[CMD] Test message sent.");
                } else {
                    Serial.println("[ERR] Not connected to server.");
                }
                break;
            case 0:
                Serial.println("[CMD] Stopping all connections...");
                bleManager.stop();
                wifiManager.disconnect();
                WiFi.disconnect(true);
                currentState = TestState::IDLE;
                printMenu();
                break;
            case 5:
                handleForcedWifiMode();
                break;
            default:
                Serial.println("[ERR] Invalid choice.");
                printMenu();
                break;
        }
    }

    // 2. 상태별 로직 실행
    switch (currentState) {
        case TestState::BLE_LISTENING:
            handleBLEState();
            break;
        case TestState::WIFI_CONNECTING:
            // WiFi.begin()은 비동기이므로 상태 체크만 수행
            handleWiFiConnectingState();
            break;
        case TestState::SERVER_CONNECTING:
            handleServerConnectingState();
            break;
        case TestState::CONNECTED:
            handleConnectedState();
            break;
        case TestState::IDLE:
        default:
            // Do nothing
            break;
    }

    delay(50); // 루프 속도 조절
}

// === 상세 구현 ===

void printMenu() {
    Serial.println("\n--- Main Menu ---");
    Serial.println("1. Start Boot Sequence (Auto Detect Mode)");
    Serial.println("2. Clear WiFi Credentials (Force BLE Mode)");
    Serial.println("3. Set Server IP (Current: " + targetServerIP + ")");
    Serial.println("4. Send Test Message (Only when Connected)");
    Serial.println("5. forced wifi connection mode");
    Serial.println("0. Stop & Reset");
    Serial.print("Enter choice: ");
}

void clearInputBuffer() {
    while (Serial.available()) Serial.read();
}

void startBootSequence() {
    Serial.println("\n[FLOW] 1. Checking Boot Mode (EEPROM)...");
    
    // BootManager가 내부적으로 WiFi 연결을 시도함 (타임아웃 있음)
    String mode = bootManager.begin();
    Serial.printf("[FLOW] Boot Mode Result: %s\n", mode.c_str());

    if (mode == BLE_MODE) {
        Serial.println("[FLOW] >> Entering BLE Mode");
        bleManager.begin();
        Serial.println("[BLE] Advertising... Connect via App to send WiFi credentials.");
        currentState = TestState::BLE_LISTENING;
    } 
    else if (mode == WIFI_MODE) {
        Serial.println("[FLOW] >> Entering WiFi Mode");
        // BootManager가 이미 WiFi 연결을 시도했으므로 상태 확인
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WIFI] Connected to %s\n", WiFi.SSID().c_str());
            Serial.println("[FLOW] >> Connecting to WebSocket Server...");
            wifiManager.begin(targetServerIP, targetServerPort);
            currentState = TestState::SERVER_CONNECTING;
        } else {
            Serial.println("[ERR] WiFi Mode but not connected? Retrying WiFi...");
            // 저장된 정보로 재시도
            String ssid = bootManager.getWiFiSSID();
            String pw = bootManager.getWiFiPassword();
            WiFi.begin(ssid.c_str(), pw.c_str());
            currentState = TestState::WIFI_CONNECTING;
        }
    }
}

void handleBLEState() {
    // BLE 폴링 (필요 시)
    bleManager.poll();

    // 자격 증명 수신 확인
    if (bleManager.hasReceivedCredentials()) {
        String ssid = bleManager.getSSID();
        String pw = bleManager.getPassword();
        String email = bleManager.getUserEmail();
        
        Serial.println("\n[BLE] Credentials Received!");
        Serial.printf("      SSID: %s\n", ssid.c_str());
        Serial.printf("      PASS: %s\n", pw.c_str());

        // 저장
        bootManager.saveWIFICredentials(ssid, pw, email);
        
        // BLE 종료 및 WiFi 전환
        Serial.println("[BLE] Stopping BLE and switching to WiFi...");
        bleManager.stop();
        
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pw.c_str());
        currentState = TestState::WIFI_CONNECTING;
    }
}

void handleWiFiConnectingState() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        Serial.print(".");
        lastPrint = millis();
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WIFI] Connected!");
        Serial.print("       IP: "); Serial.println(WiFi.localIP());
        
        // 서버 연결 시작
        Serial.println("[FLOW] >> Connecting to WebSocket Server...");
        wifiManager.begin(targetServerIP, targetServerPort);
        currentState = TestState::SERVER_CONNECTING;
    }
    else if (WiFi.status() == WL_CONNECT_FAILED) {
        Serial.println("\n[ERR] WiFi Connect Failed. Going back to IDLE.");
        currentState = TestState::IDLE;
        printMenu();
    }
}

void handleServerConnectingState() {
    wifiManager.poll(); // 웹소켓 처리

    if (wifiManager.isConnected()) {
        Serial.println("\n[SERVER] WebSocket Connected!");
        Serial.println("[FLOW] >> Ready for Communication");
        currentState = TestState::CONNECTED;
        printMenu(); // 메뉴 다시 표시
    }
}

void handleConnectedState() {
    wifiManager.poll(); // 지속적인 폴링 필요

    // 수신 메시지 확인
    String msg = wifiManager.getLastReceivedMessage();
    if (msg.length() > 0) {
        Serial.println("\n[RX] Message Received:");
        Serial.println(msg);
        Serial.print("Enter choice: "); // 프롬프트 복구
    }

    // 연결 끊김 체크
    if (!wifiManager.isConnected()) {
        Serial.println("\n[ERR] Server Disconnected. Retrying...");
        currentState = TestState::SERVER_CONNECTING;
    }
}

void handleForcedWifiMode(){
    bleManager.stop();
    String ssid = "sweetholly";
    String pw = "milk0201";
    String email = "test@test.com"; // Add a default or test email
    Serial.println("[CMD] Forcing WiFi Connection Mode...");
    bootManager.saveWIFICredentials(ssid, pw, email);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pw.c_str());
    currentState = TestState::WIFI_CONNECTING;
}