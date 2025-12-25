#include "WIFIconnection.h"

// static 멤버 초기화
HttpConnectionManager* HttpConnectionManager::instance = nullptr;

HttpConnectionManager::HttpConnectionManager() 
    : serverPort(8000) {
    instance = this;  // static 포인터에 현재 객체 저장
}

bool HttpConnectionManager::begin(const String& serverIP, uint16_t port, String machine_id, String userEmail) {
    // WiFi 연결 확인`
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] WiFi not connected!");
        return false;
    }

    /* --- 네트워크 확인 --- */
    // 네트워크 연결성 확인을 위한 추가 디버깅 정보 출력
    Serial.println("=== Network Diagnostics ===");
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print("ESP32 IP Address: ");  // [추가] ESP32 IP 주소 출력
    Serial.println(WiFi.localIP());
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
    Serial.println("==================================================");

    serverUrl = serverIP;
    serverPort = port;
    this->machine_id = machine_id;
    this->userEmail = userEmail;

    IPAddress testIP;
    if (!testIP.fromString(serverIP.c_str())) {
        Serial.println("[WIFI] Invalid server IP address");
        return false;
    }Serial.printf("[WIFI] Attempting to connect to server: %s:%d\n", serverIP.c_str(), port);

    HTTPClient http;
    String registerUrl = "http://" + serverIP + ":" + String(port) + "/machine/" + machine_id + "/register"; 
    Serial.printf("[WIFI] Registering machine at: %s\n", registerUrl.c_str());
    
    http.begin(registerUrl);
    http.setTimeout(10000);  // 10초 타임아웃 설정
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> doc;
    doc["machine_id"] = machine_id;
    doc["email"] = userEmail;
    String requestBody;
    serializeJson(doc, requestBody);
    int httpResponse = http.POST(requestBody);
    delay(3000);
    bool registrationSucess =false;
    if(httpResponse > 0) {
        String response = http.getString();
        Serial.printf("[WIFI] HTTP Response code: %d\n", httpResponse);
        Serial.println("[WIFI] Response: " + response);
        
        // 200 OK 또는 201 Created 등 성공 코드 확인
        if (httpResponse == 200 || httpResponse == 201) {
            registrationSucess = true;
        } else {
            Serial.printf("[WIFI] Registration failed with HTTP status: %d\n", httpResponse);
        }
    }   
    else {
        Serial.printf("[WIFI] Error on sending POST: %s\n", http.errorToString(httpResponse).c_str());
    }      
    http.end();
    if(!registrationSucess) return false;
    // WebSocket 연결 시작
    String path = "/ws/machine/" + machine_id;
    Serial.printf("[DEBUG] Trying WS: ws://%s:%d%s\n",
                  serverIP.c_str(), port, path.c_str());
    wsClient.onEvent(webSocketEvent);
    wsClient.setReconnectInterval(5000);  // 5초마다 재연결 시도
    wsClient.enableHeartbeat(15000, 3000, 2);        
    wsClient.begin(serverIP.c_str(), port, path.c_str());
    delay(3000);
    //wsClient.enableHeartbeat(15000, 3000, 2);  // Ping: 15초, Pong timeout: 3초, 재시도: 2회
    
    Serial.printf("[WIFI] Connecting to websockets://%s:%d/ws/machine/%s\n", serverUrl.c_str(), serverPort, machine_id.c_str());
    
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
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WIFI] Disconnected");
            break;
            
        case WStype_CONNECTED:
            Serial.printf("[WIFI] Connected to: %s\n", payload);
            break;
            
        case WStype_TEXT:
            Serial.printf("[WIFI] Received: %s\n", payload);
            if (payload && length > 0) {
                setLastReceivedMessage(String((char*)payload));
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
            break;
        case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break; 
    }
}

bool HttpConnectionManager::sendMessage(sendItem& sendData) {
    if (wsClient.isConnected() == false) {
        Serial.println("[WIFI] Not connected!");
        return false;
    }

    bool success = wsClient.sendTXT(sendData.buf);
    
    if (success) {
        Serial.printf("[WIFI TASK] Sent data: %s\n", sendData.buf);
    } else {
        Serial.println("[WIFI] Send Failed!");
    }
    return success;
}

void HttpConnectionManager::routeMessage() {
    // getLastReceivedMessage()는 반환 후 초기화하므로 한 번만 호출
    String receivedMsg = getLastReceivedMessage();
    static cmdItem command;
    if (receivedMsg.length() == 0) {
        return;
    }
    
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
             type == "BREW_STATUS" || type == "FINISH_WEIGHING" ||
             type == "FINISH_CLICK_ADJUST" || type == "FINISH_WEIGHING" ||
             type == "START_GRINDING" || type == "START_POURING" ||
             type == "START_RINSING" || type == "JUST_GRINDING" ||
             type == "JUST_POURING" || type == "JUST_TARE_SCALE" ||
             type == "JUST_GRINDING_STOP" || type == "JUST_POURING_STOP" ||
             type == "JUST_TARE_SCALE_STOP" || type =="TARE" ||
             type == "FINISH_GRINDING" || type == "STOP_GRINDING") 
            {        
                if(gCommandQueue == nullptr){
                    Serial.println("[WIFI] ERROR: gCommandQueue is NULL!");
                    return;
                }
                memset(&command, 0, sizeof(cmdItem));
                SAFE_COPY_TO_BUFFER(command, type);
                if(xQueueSend(gCommandQueue, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
                    Serial.println("[WIFI] Failed to send command to queue!");
                } else {
                    Serial.println("[WIFI] Command sent to queue: " + type);
                }
            }
        else{
            Serial.println("[WIFI] Unknown message type: " + type);
        }
}


bool HttpConnectionManager::isConnected() {
    return wsClient.isConnected();
}

void HttpConnectionManager::poll() {
    wsClient.loop();  // WebSocket 이벤트 처리
}

void HttpConnectionManager::disconnect() {
    wsClient.disconnect();
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
    String url = "/ws/machine/" + machine_id;
    wsClient.begin(serverUrl, serverPort, url.c_str());
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