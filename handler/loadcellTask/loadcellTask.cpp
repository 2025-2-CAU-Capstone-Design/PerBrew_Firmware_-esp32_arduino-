#include "LoadCellTask.h"
#include <ArduinoJson.h>
#include "../driver/loadcell/loadcell_driver.h"

extern QueueHandle_t gSendQueue;
extern SharedState gShared;

void LoadCellTask(void* pv) {
    LoadCellDriver* loadcell = ((DriverContext*)pv)->loadcell;
    DriverContext* ctx = (DriverContext*)pv;

    if (!loadcell->begin(128, 5)) {
        Serial.println("[LoadCellTask] Loadcell begin failed");
        vTaskDelete(NULL);   // 더 이상 진행 불가
    }

    while (true) {
        if (getSendMode() == SendMode::NONE){
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue; // 전송 금지
        }
        float w = 0.0f;
        if (loadcell->tryUpdateWeight(100)) {
            w = loadcell->getWeight();
        }

        // ===== Critical Section =====
        if (xSemaphoreTake(gShared.mutex, portMAX_DELAY) == pdTRUE) {
            gShared.currentWeight = w;
            xSemaphoreGive(gShared.mutex);
        }

        // WiFi 송신 큐 전송
        StaticJsonDocument<256> doc;
        doc["machine_id"] = ctx->machine_id;
        doc["type"] = "WEIGHT";

        JsonObject data = doc.createNestedObject("data");
        data["value"] = w;
        data["unit"]  = "g";

        String jsonStr;
        serializeJson(doc, jsonStr);

        // 큐 전송
        xQueueSendToBack(gSendQueue, &jsonStr, 0);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
