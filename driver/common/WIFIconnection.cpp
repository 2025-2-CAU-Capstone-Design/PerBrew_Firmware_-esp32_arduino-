#include "WIFIconnection.h"

// static 멤버 초기화
HttpConnectionManager* HttpConnectionManager::instance = nullptr;

HttpConnectionManager::HttpConnectionManager() 
    : connected(false), serverPort(8000) {
    instance = this;  // static 포인터에 현재 객체 저장
}

bool HttpConnectionManager::begin(const String& serverIP, uint16_t port, String machine_id, String userEmail) {
    // WiFi 연결 확인
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] WiFi not connected!");
        return false;
    }
    serverUrl = serverIP;
    serverPort = port;
    HTTPClient http;
    String registerUrl = "http://" + serverIP + ":" + String(port) + "/machine/register" + machine_id; 
    Serial.printf("[WIFI] Registering machine at: %s\n", registerUrl.c_str());
    
    http.begin(registerUrl);
    http.addHeader("Content-Type", "application/json");

    // WebSocket 이벤트 핸들러 등록
    wsClient.onEvent(webSocketEvent);
    StaticJsonDocument<200> doc;
    doc["machine_id"] = machine_id;
    doc["user_email"] = userEmail;
    String requestBody;
    serializeJson(doc, requestBody);
    int httpResponse = http.POST(requestBody);
    bool registrationSucess =false;
    if(httpResponse > 0) {
        String response = http.getString();
        Serial.printf("[WIFI] HTTP Response code: %d\n", httpResponse);
        Serial.println("[WIFI] Response: " + response);
        
        // 200 OK 또는 201 Created 등 성공 코드 확인
        if (httpResponse == 200 || httpResponse == 201) {
            registrationSucess = true;
        }
    } else {
        Serial.printf("[WIFI] Error on sending POST: %s\n", http.errorToString(httpResponse).c_str());
    }   
    http.end();
    if(!registrationSucess) return false;
    // WebSocket 연결 시작
    String url = "/ws/machine" + machine_id;
    wsClient.begin(serverUrl, serverPort,url.c_str());
    
    // 재연결 설정
    wsClient.setReconnectInterval(5000);  // 5초마다 재연결 시도
    wsClient.enableHeartbeat(15000, 3000, 2);  // Ping: 15초, Pong timeout: 3초, 재시도: 2회
    
    Serial.printf("[WIFI] Connecting to websockets://%s:%d/ws\n", serverUrl.c_str(), serverPort);
    
    return true;
}

// 콜백 함수
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
    // getLastReceivedMessage()는 반환 후 초기화하므로 한 번만 호출
    String receivedMsg = getLastReceivedMessage();
    auto& doc = jsonDoc;
    auto error = deserializeJson(doc, receivedMsg);
    if(error){
        Serial.print("[WIFI] JSON Deserialization failed: ");
        Serial.println(error.c_str());
        return;
    }
    
    String type = doc["type"] | "";
        if(type =="RECIPE_DATA"){
            setRecipeParsing(&receivedMsg);
            RecipeInfo parsed = getRecipeParsing();
            if(gRecipeQueue != nullptr){
                if (xQueueSend(gRecipeQueue, &parsed, 0) != pdTRUE) {
                    Serial.println("[WIFI] Failed to send recipe to queue");
                }
                else {
                    Serial.println("[WIFI] Recipe sent to queue");
                }
            }
        }
        else if (type == "START_BREW" || type == "STOP_BREW" ||
             type == "REGISTER_MACHINE" || type == "LOADCELL_VALUE" ||
             type == "BREW_STATUS")
            {
                String command = type;
                if(gCommandQueue != nullptr){
                    xQueueSend(gCommandQueue, &command, 0);
                    Serial.println("[WIFI] Command sent to queue: " + command);
                }
            }
        else{
            Serial.println("[WIFI] Unknown message type: " + type);
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

void HttpConnectionManager::setRecipeParsing(String *recipeJson) {
    StaticJsonDocument<4096> doc;
    DeserializationError error = deserializeJson(doc, *recipeJson);
    if (error) return;

    ParsedRecipe.rinsing = doc["recipe"]["rinsing"] | true;
    ParsedRecipe.water_temperature_c = doc["recipe"]["water_temperature_c"] | 92;
    ParsedRecipe.dose_g = doc["recipe"]["dose_g"] | 15.0;
    ParsedRecipe.total_brew_time_s = doc["recipe"]["total_brew_time_s"] | 180;

    ParsedRecipe.grind_level = doc["recipe"]["grind_level"] | 50;
    ParsedRecipe.grind_microns = doc["recipe"]["grind_microns"] | 600;

    JsonArray steps = doc["recipe"]["pouring_steps"].as<JsonArray>();
    ParsedRecipe.pouring_steps_count = steps.size();

    for (int i = 0; i < ParsedRecipe.pouring_steps_count; i++) {
        ParsedRecipe.pouring_steps[i].step = steps[i]["step"] | i;
        ParsedRecipe.pouring_steps[i].water_g = steps[i]["water_g"] | 0.0;
        ParsedRecipe.pouring_steps[i].pour_time_s = steps[i]["pour_time_s"] | 0;
        ParsedRecipe.pouring_steps[i].wait_time_s = steps[i]["wait_time_s"] | 0;
        ParsedRecipe.pouring_steps[i].technique = steps[i]["technique"] | "";
    }
}

RecipeInfo HttpConnectionManager::getRecipeParsing() {
    return this->ParsedRecipe;
}