#include "HeaterTask.h"
#include <ArduinoJson.h>
#include "../driver/heater/heater_driver.h"

extern SharedState gShared;
extern QueueHandle_t gSendQueue;

void HeaterTask(void* pv) {
    RecipeInfo recipe = ((DriverContext*)pv)->recipe;
    HeaterDriver* heater = ((DriverContext*)pv)->heater;
    DriverContext* ctx = (DriverContext*)pv;

    heater->begin();

    while (true) {
        // heater update()로 PID 계산 진행
        /*if(!driver->heater->isHeating()){
            driver->heater->begin();
            driver->heater->startHeating(recipe.water_temperature_c);
        }*/
        heater->update();

        float temp = heater->getCurrentTemperature();
        bool stable = heater->isTemperatureStable(1.0);

        // ===== Critical Section =====
        if (xSemaphoreTake(gShared.mutex, portMAX_DELAY) == pdTRUE) {
            gShared.currentTemp = temp;
            gShared.tempStable = stable;
            xSemaphoreGive(gShared.mutex);
        }

        // 데이터 전송 여부
        if (getSendMode() == SendMode::NONE || getSendMode() == SendMode::WEIGHT_ONLY) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue; // 전송 금지
        }
        // WiFi 보고
        String status = "IDLE";
        if(heater->isHeating()) status = "HEATING";
        else if(heater->isCompleted()) status = "COMPLETED";
        
        // JSON 생성
        StaticJsonDocument<256> doc;
        doc["machineID"] = ctx->machineID;
        doc["type"] = "TEMP";
        doc["status"] = status;

        JsonObject data = doc.createNestedObject("data");
        data["value"] = temp;

        String jsonStr;
        serializeJson(doc, jsonStr);
        xQueueSendToBack(gSendQueue, &jsonStr, 0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
