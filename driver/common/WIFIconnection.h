/*
    1. WIFI 모드
*/

#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h> 
#include "boot.h"
// 전송할 메시지는 각 드라이버에서 정의, 전송 형식은 JSON으로

enum class APIType {
    COMMAND,
    RETURN_STATUS,
    ERROR_REPORT,
};

class HttpConnectionManager {
public:
    HttpConnectionManager();
    
    // 연결 초기화 (BootManager 완료 후 호출)
    bool begin(const String& serverIP, uint16_t port = 8080);
    
    // 메시지 송신
    bool sendMessage(String& jsonData);
    
    // 메시지 라우팅
    void routeMessage();
    
    // 연결 상태 확인
    bool isConnected();
    
    // RTOS Task에서 호출할 폴링 함수
    void poll();
    
    // 연결 종료
    void disconnect();

    void setStaticJsonDoc(StaticJsonDocument<4096>& doc) {
        jsonDoc = doc;
    };

    StaticJsonDocument<4096> getStaticJsonDoc() {
        return jsonDoc;
    };

    void setLastReceivedMessage(const String& message) {
        lastReceivedMessage = message;
    };

    String getLastReceivedMessage() const {
        return lastReceivedMessage;
    };

    void setLastReceivedCommand(const String& command) {
        lastReceivedCommand = command;
    };
    String getLastReceivedCommand() const {
        return lastReceivedCommand;
    };

private:
    WebSocketsClient wsClient;
    String serverUrl;
    uint16_t serverPort;
    StaticJsonDocument<4096> jsonDoc;
    String lastReceivedMessage;
    String lastReceivedCommand;
    bool connected;
    
    void (*messageCallback)(String message) = nullptr;
    
    // WebSocket 이벤트 핸들러 (static)
    static void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
    
    // 이벤트 처리
    void handleEvent(WStype_t type, uint8_t* payload, size_t length);
    
    // 재연결 시도
    bool reconnect();
    
    // static 인스턴스 포인터 (콜백)
    static HttpConnectionManager* instance;
};