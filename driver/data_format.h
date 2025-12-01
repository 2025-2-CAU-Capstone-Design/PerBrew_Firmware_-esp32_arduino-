#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>


class ArrangingDriver;
class GrinderDriver;
class HeaterDriver;
class LoadCellDriver;
class PouringSectionDriver;

class BLEConnectionManager;
class HttpConnectionManager;
class BootManager;

// ===== CONSTANTS =====
#define BLE_MODE "BLE"
#define WIFI_MODE "WIFI"

// ==== String To Data Format
//String을 cmdItem/SendItem 버퍼로 복사
#define SAFE_COPY_TO_BUFFER(dest, src) \
    do { \
        memset((dest).buf, 0, sizeof((dest).buf)); \
        strncpy((dest).buf, (src).c_str(), sizeof((dest).buf) - 1); \
    } while(0)

//char*을 cmdItem/SendItem 버퍼로 복사    
#define SAFE_COPY_CSTR_TO_BUFFER(dest, src) \
    do { \
        memset((dest).buf, 0, sizeof((dest).buf)); \
        strncpy((dest).buf, (src), sizeof((dest).buf) - 1); \
    } while(0)

// ===== Brew Status =====
enum class BrewStatus {
    IDLE,
    BREWSTART,
    RINSING,
    HEATING,
    GRINDING,
    POURING,
    STOP,
};


// ===== Pouring Step =====
struct PouringStep {
    uint8_t step;              // 단계 번호
    float water_g;             // 주수량 (그램)
    uint16_t pour_time_s;      // 주수 시간 (초)
    uint16_t wait_time_s;      // 대기 시간 (초) - step 0에서 사용
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

// ===== Driver Context =====
// (Task 간 공유되는 하드웨어 핸들)
struct DriverContext {
    ArrangingDriver* arranging;
    PouringSectionDriver* pouring;
    GrinderDriver* grinder;
    HeaterDriver* heater;
    LoadCellDriver* loadcell;

    TaskHandle_t loadCellTaskHandle;

    String machine_id;              // 메시지 prefix 용

    RecipeInfo recipe;             // 현재 사용되는 레시피
    BrewStatus status;             // BrewTask 상태
};

enum class SendMode {
    NONE,       // 아무것도 보내지 않음
    WEIGHT_ONLY, // 사용자 원두 측정 모드
    BREWING     // 온도 + 무게 보내기
};


typedef struct {
    char buf[10240]; // 크으으으게 잡자
} sendItem;

typedef struct {
    char buf[256];
} cmdItem;


// 연결 컨텍스트
struct ConnectionContext {
    BLEConnectionManager* ble;
    HttpConnectionManager* wifi;
    BootManager* boot;
    String machine_id;
    String userEmail;
    bool bleModeActive = false;

    TaskHandle_t supervisorTask;
    TaskHandle_t bleTask;
    TaskHandle_t wifiTask;
};

// ===== Global Queues =====
extern QueueHandle_t gRecipeQueue;
extern QueueHandle_t gCommandQueue;
extern QueueHandle_t gSendQueue;

extern DriverContext driver;