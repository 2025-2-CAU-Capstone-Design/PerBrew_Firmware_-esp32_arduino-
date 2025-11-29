#include "../driver/data_format.h"
#include <Arduino.h>
#include "../handler/brewingTask/brewTask.h"
#include "../handler/heaterTask/heaterTask.h"
#include "../handler/loadcellTask/loadcellTask.h"
#include "../handler/connectionTask/connectionTask.h"

#include "../driver/arranging/arranging_driver.h"
#include "../driver/pouring/pouringSection_driver.h"
#include "../driver/grinder/grinder_driver.h"
#include "../driver/loadcell/loadcell_driver.h"
#include "../driver/heater/heater_driver.h"
#include "../driver/common/boot.h"
#include "../driver/common/BLEconnection.h"
#include "../driver/common/WIFIconnection.h"

// ===== 전역 객체 =====
QueueHandle_t gRecipeQueue  = nullptr;
QueueHandle_t gCommandQueue = nullptr;
QueueHandle_t gSendQueue    = nullptr;

DriverContext driver;
BootManager bootManager;
ConnectionContext connCtx;

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    bootManager.begin();
    vTaskDelay(500);

    Serial.println("=== Coffee Machine Boot ===");

    // ----- SharedState 초기화 -----
    gShared.mutex = xSemaphoreCreateMutex();
    gShared.currentWeight = 0.0f;
    gShared.currentTemp   = 20.0f;
    gShared.tempStable    = false;
    gShared.currentSendMode = SendMode::NONE;
    //gShared.machine_id = bootManager.getmachine_id();

    // ----- Queue 생성 -----
    gRecipeQueue  = xQueueCreate(2, sizeof(RecipeInfo));
    gCommandQueue = xQueueCreate(10, sizeof(String));
    gSendQueue    = xQueueCreate(20, sizeof(String));

    // ----- DriverContext 초기화 -----
    driver.arranging = new ArrangingDriver();
    driver.pouring   = new PouringSectionDriver();
    driver.grinder   = new GrinderDriver();
    driver.heater    = new HeaterDriver();
    driver.loadcell  = new LoadCellDriver();
    driver.status    = BrewStatus::IDLE;

    // ----- Machine ID 설정 -----
    driver.machine_id = bootManager.getmachine_id();
    Serial.println("[machine_id] " + driver.machine_id);

    // ----- ConnectionContext 초기화 -----
    connCtx.boot = &bootManager;
    connCtx.ble  = new BLEConnectionManager();
    connCtx.wifi = new HttpConnectionManager();
    connCtx.machine_id = bootManager.getmachine_id();

    connCtx.supervisorTask = nullptr;
    connCtx.bleTask        = nullptr;
    connCtx.wifiTask       = nullptr;

    // ----- Connection Supervisor Task 시작 -----
    xTaskCreate(
        ConnectionSupervisorTask,
        "ConnectionSupervisorTask",
        8192,
        &connCtx,
        2,
        &connCtx.supervisorTask
    );

    // ----- LoadCell, Heater, Brew Task 시작 -----
    xTaskCreatePinnedToCore(LoadCellTask, "LoadCellTask", 4096, &driver, 3, &driver.loadCellTaskHandle, 1);
    xTaskCreate(HeaterTask,   "HeaterTask",   4096, &driver, 3, nullptr);
    xTaskCreate(BrewTask,     "BrewTask",     4096, &driver, 2, nullptr);

    Serial.println("=== Setup Complete ===");
}

void loop() {
    // FreeRTOS 사용 시 loop는 비워둔다
}

