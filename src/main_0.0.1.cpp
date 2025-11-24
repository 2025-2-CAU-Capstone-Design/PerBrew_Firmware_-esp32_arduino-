#include "../driver/data_format.h"
#include <Arduino.h>
#include "../handler/brewingTask/brewTask.h"
#include "../handler/brewingTask/driverTask.h"


QueueHandle_t gRecipeQueue = nullptr;
QueueHandle_t gCommandQueue = nullptr;

void setup() {
    // ... 기타 초기화 ...

    gRecipeQueue  = xQueueCreate(1, sizeof(RecipeInfo));
    gCommandQueue = xQueueCreate(5, sizeof(int)); // 필요에 맞게 바꿔라

    // DriverContext 초기화 후
    // driver.arranging = &arrangingDriverInstance;
    // driver.pouring   = &pouringDriverInstance;
    // driver.grinder   = &grinderDriverInstance;
    // driver.heater    = &heaterDriverInstance;
    // driver.loadcell  = &loadcellDriverInstance;

    xTaskCreate(DriverTask, "DriverTask", 4096, &driver, 2, nullptr);
    xTaskCreate(BrewTask,   "BrewTask",   4096, &driver, 2, nullptr);
}