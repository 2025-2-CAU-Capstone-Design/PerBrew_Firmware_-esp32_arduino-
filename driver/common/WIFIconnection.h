#pragma once
/*
    1. WIFI 모드
    - websocket 통신
    - json 파싱
    - queue producer
*/

#pragma once
#include <Arduino.h>
#include <./data_format.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h> 
#include <HTTPClient.h>
#include "boot.h"


class HttpConnectionManager {
public:
    HttpConnectionManager();
    
    // 연결 초기화 (BootManager 완료 후 호출)
    bool begin(const String& serverIP, uint16_t port = 8080, String machine_id = "", String userEmail="");
    
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

    void setLastReceivedMessage(const String& message) {
        lastReceivedMessage = message;
    };

    // 마지막 메시지를 반환하고 초기화 (move semantics)
    String getLastReceivedMessage() {
        String temp = lastReceivedMessage;
        lastReceivedMessage = "";
        return temp;
    };
    
    void setRecipeParsing(String *recipeJson);
    RecipeInfo getRecipeParsing();
    
private:
    WebSocketsClient wsClient;
    String serverUrl;
    uint16_t serverPort;
    StaticJsonDocument<4096> jsonDoc;
    String lastReceivedMessage;
    RecipeInfo ParsedRecipe;
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