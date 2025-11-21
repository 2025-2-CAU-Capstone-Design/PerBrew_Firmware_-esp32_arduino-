#include "WIFIconnection.h"

// static 멤버 초기화
HttpConnectionManager* HttpConnectionManager::instance = nullptr;

HttpConnectionManager::HttpConnectionManager() 
    : connected(false), serverPort(8080) {
    instance = this;  // static 포인터에 현재 객체 저장
}

bool HttpConnectionManager::begin(const String& serverIP, uint16_t port) {
    // WiFi 연결 확인
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] WiFi not connected!");
        return false;
    }
    
    serverUrl = serverIP;
    serverPort = port;
    
    // WebSocket 이벤트 핸들러 등록
    wsClient.onEvent(webSocketEvent);
    
    // WebSocket 연결 시작
    wsClient.begin(serverUrl, serverPort, "/ws");
    
    // 재연결 설정
    wsClient.setReconnectInterval(5000);  // 5초마다 재연결 시도
    wsClient.enableHeartbeat(15000, 3000, 2);  // Ping: 15초, Pong timeout: 3초, 재시도: 2회
    
    Serial.printf("[WIFI] Connecting to websockets://%s:%d/ws\n", serverUrl.c_str(), serverPort);
    
    return true;
}

// static 콜백 함수
void HttpConnectionManager::webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    if (instance != nullptr) {
        instance->handleEvent(type, payload, length);
    }
}

// 실제 이벤트 처리
void HttpConnectionManager::handleEvent(WStype_t type, uint8_t* payload, size_t length) {
    setLastReceivedMessage(String((char*)payload));
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WIFI] Disconnected");
            connected = false;
            break;
            
        case WStype_CONNECTED:
            Serial.printf("[WIFI] Connected to: %s\n", payload);
            connected = true;
            break;
            
        case WStype_TEXT:
            Serial.printf("[WIFI] Received: %s\n", payload);
            if(connected){
                routeMessage();
            }
            break;
            
        case WStype_BIN:
            Serial.printf("[WIFI] Binary data received: %u bytes\n", length);
            break;
            
        case WStype_PING:
            Serial.println("[WIFI] Got Ping");
            break;
            
        case WStype_PONG:
            Serial.println("[WIFI] Got Pong");
            break;
            
        case WStype_ERROR:
            Serial.printf("[WIFI] Error: %s\n", payload);
            connected = false;
            break;
        case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break; 
    }
}

bool HttpConnectionManager::sendMessage(String& jsonData) {
    if (!connected) {
        Serial.println("[WIFI] Not connected!");
        return false;
    }
    
    bool success = wsClient.sendTXT(jsonData);
    
    if (success) {
        Serial.printf("[WIFI] Sent: %s\n", jsonData.c_str());
    } else {
        Serial.println("[WIFI] Send Failed!");
    }
    return success;
}

void HttpConnectionManager::routeMessage() {
    auto& doc = jsonDoc;
    auto error = deserializeJson(doc, getLastReceivedMessage());
    if(error){
        Serial.print("[WIFI] JSON Deserialization failed: ");
        Serial.println(error.c_str());
        return;
    }
    
    String type = doc["type"] | "";
        if (type == "START_BREW") {
            setLastReceivedCommand("START_BREW");
        }
        else if (type == "STOP_BREW") {
            setLastReceivedCommand("STOP_BREW");
        }
        else if (type == "REGISTER_MACHINE") {
            setLastReceivedCommand("REGISTER_MACHINE");
        }
        else if (type == "LOADCELL_VALUE") {
            setLastReceivedCommand("LOADCELL_VALUE");
        }
        else if (type == "BREW_STATUS") {
            setLastReceivedCommand("BREW_STATUS");
        }
        else {
            Serial.println("[WIFI] Unknown command type");
        }
}


bool HttpConnectionManager::isConnected() {
    return connected && wsClient.isConnected();
}

void HttpConnectionManager::poll() {
    wsClient.loop();  // WebSocket 이벤트 처리
}

void HttpConnectionManager::disconnect() {
    wsClient.disconnect();
    connected = false;
    Serial.println("[WIFI] Disconnected");
}

bool HttpConnectionManager::reconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] WiFi disconnected");
        return false;
    }
    
    Serial.println("[WIFI] Attempting reconnect...");
    wsClient.disconnect();
    delay(100);
    wsClient.begin(serverUrl, serverPort, "/ws");
    
    return true;
}