#pragma once
#include<Arduino.h>
#include <../driver/arranging/arranging_driver.h>
#include <../driver/grinder/grinder_driver.h>
#include <../driver/heater/heater_driver.h>
#include <../driver/loadcell/loadcell_driver.h>
#include <../driver/pouring/pouringSection_driver.h>
#include <../driver/common/WIFIconnection.h>
#include <../driver/common/BLEconnection.h>
#include <../driver/common/boot.h>

#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

// 브루 상태 열거형
enum class BrewStatus {
    IDLE,
    BREWSTART,
    RINSING,
    HEATING,
    GRINDING,
    POURING,
    STOP,
};
// 각 푸어링 단계 정보
struct PouringStep {
    uint8_t step;              // 단계 번호
    float water_g;             // 주수량 (그램)
    uint16_t pour_time_s;      // 주수 시간 (초)
    uint16_t wait_time_s;      // 대기 시간 (초) - step 0에서 사용
    uint16_t bloom_time_s;     // 블룸 시간 (초) - step 1+ 에서 사용
    String technique;          // 주수 기법 (예: "center pour", "spiral outward")
};

// 레시피 정보
struct RecipeInfo {
    bool rinsing;                       // 린싱 여부
    uint8_t water_temperature_c;        // 물 온도 (섭씨)
    float dose_g;                       // 원두 투입량 (그램)
    uint16_t total_brew_time_s;         // 전체 추출 시간 (초)
    uint8_t grind_level;                // 분쇄 레벨 (0-100)
    uint16_t grind_microns;             // 분쇄 입자 크기 (마이크론)
    PouringStep pouring_steps[10];      // 최대 10개의 푸어링 단계
    uint8_t pouring_steps_count;        // 실제 푸어링 단계 개수
};

// 레시피 데이터 (배열 대응)
struct RecipeData {
    RecipeInfo recipe_info;
};

// 드라이버 컨텍스트
struct DriverContext {
    ArrangingDriver* arranging;
    PouringSectionDriver* pouring;
    GrinderDriver* grinder;
    HeaterDriver* heater;
    LoadCellDriver* loadcell;

    QueueHandle_t* queue;

    RecipeInfo recipe;
    BrewStatus status;
};


// 연결 컨텍스트
struct ConnectionContext {
    BLEConnectionManager* ble;
    BootManager* boot;
    HttpConnectionManager* wifi;
    QueueHandle_t* messageQueue;
    QueueHandle_t* recipeQueue;
};



extern DriverContext driver;
extern QueueHandle_t gRecipeQueue;
extern QueueHandle_t gCommandQueue;