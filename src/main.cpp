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
    gCommandQueue = xQueueCreate(10, sizeof(cmdItem));
    gSendQueue    = xQueueCreate(200, sizeof(sendItem));

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
    connCtx.userEmail  = bootManager.getUserEmail();
    

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





/*    uint64_t chipid;

    chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length: 6 bytes)
    
    Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32)); // print High 2 bytes
    Serial.printf("%08X\n", (uint32_t)chipid); // print Low 4bytes
    Serial.println("---------------------------------");
    
    Serial.printf("Chip Revision %d\n", ESP.getChipRevision());
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Number of Core: %d\n", chip_info.cores);
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println();
    
    Serial.printf("Flash Chip Size = %d byte\n", ESP.getFlashChipSize());
    Serial.printf("Flash Frequency = %d Hz\n", ESP.getFlashChipSpeed());
    Serial.println();
    
    Serial.printf("ESP-IDF version = %s\n", esp_get_idf_version());
    Serial.println();
    
    Serial.printf("Total Heap Size = %d\n", ESP.getHeapSize());
    Serial.printf("Free Heap Size = %d\n", ESP.getFreeHeap());
    Serial.printf("Lowest Free Heap Size = %d\n", ESP.getMinFreeHeap());
    Serial.printf("Largest Heap Block = %d\n", ESP.getMaxAllocHeap());
    Serial.println();
    
    uint8_t mac0[6];
    esp_efuse_mac_get_default(mac0);
    Serial.printf("Default Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac0[0], mac0[1], mac0[2], mac0[3], mac0[4], mac0[5]);
    
    uint8_t mac3[6];
    esp_read_mac(mac3, ESP_MAC_WIFI_STA);
    Serial.printf("[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
    
    uint8_t mac4[6];
    esp_read_mac(mac4, ESP_MAC_WIFI_SOFTAP);
    Serial.printf("[Wi-Fi SoftAP] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);
    
    uint8_t mac5[6];
    esp_read_mac(mac5, ESP_MAC_BT);
    Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac5[0], mac5[1], mac5[2], mac5[3], mac5[4], mac5[5]);
    
    uint8_t mac6[6];
    esp_read_mac(mac6, ESP_MAC_ETH);
    Serial.printf("[Ethernet] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
    */