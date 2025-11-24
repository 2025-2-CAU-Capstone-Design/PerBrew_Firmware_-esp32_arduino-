#include "./driverTask.h"

DriverContext driver;

void DriverTask(void* pv) {
    DriverContext* ctx = (DriverContext*)pv;

    ctx->arranging->begin();
    ctx->pouring->begin();
    ctx->grinder->begin();
    ctx->heater->begin();
    ctx->loadcell->begin();  // update는 안 쓰지만 초기화는 필요

    ctx->status = BrewStatus::IDLE;
    
    while (true) {
        ctx->arranging->update();
        ctx->pouring->update();
        ctx->grinder->update();
        ctx->heater->update();

        vTaskDelay( 10 / portTICK_PERIOD_MS );  // 10ms 대기
    }
}
