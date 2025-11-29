#pragma once
#include "../../driver/data_format.h"

struct SharedState {
    float currentWeight;
    float currentTemp;
    bool tempStable;
    String machine_id;
    String userEmail;
    SendMode currentSendMode;
    SemaphoreHandle_t mutex;
};

void setSendMode(SendMode mode);   
SendMode getSendMode();
extern SharedState gShared;

