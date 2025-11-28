#include "shared_state.h"

SharedState gShared = {
    .currentWeight = 0.0f,
    .currentTemp = 0.0f,
    .tempStable = false,
    .currentSendMode = SendMode::NONE,
    .mutex = nullptr,
};


void setSendMode(SendMode mode) {
    xSemaphoreTake(gShared.mutex, portMAX_DELAY);
    gShared.currentSendMode = mode;
    xSemaphoreGive(gShared.mutex);
}

SendMode getSendMode() {
    SendMode mode;
    xSemaphoreTake(gShared.mutex, portMAX_DELAY);
    mode = gShared.currentSendMode;
    xSemaphoreGive(gShared.mutex);
    return mode;
}