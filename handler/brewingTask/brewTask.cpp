#include "./brewTask.h"
#include "../driver/data_format.h"

// 린싱/분쇄/푸어링 상수들
static constexpr uint32_t RINSE_PUMP_PWM      = 180;    // 린싱 펌프 세기
static constexpr uint32_t RINSE_DURATION_MS   = 3000;   // 린싱 시간
static constexpr uint32_t GRIND_MAX_MS        = 12000;  // 최대 분쇄 시간
static constexpr uint32_t LOADCELL_TIMEOUT_MS = 100;    // 로드셀 샘플 타임아웃
static constexpr float    POUR_TARGET_EPSILON = 0.5f;   // 목표 수량 허용 오차(g)

// 모든 기능 중지
static void stopAll(DriverContext* driver) {
    driver->grinder->stopGrinding();
    driver->heater->stopHeating();
    driver->pouring->stopPump();

    driver->loadcell->powerDown();
    // arranging 은 update 루프에서 멈춘 상태 유지
}

// 2-a, 2-b, 2-c: 예열 + 린싱 + 원위치
static void runPreHeatAndRinse(DriverContext* driver, RecipeInfo& recipe) {
    Serial.println("[BREW] === PREP: HEAT + RINSE ===");

    // 로드셀 가동 (실시간 측정은 TelemetryTask에서 활용)
    if (!driver->loadcell->isReady()) {
        driver->loadcell->powerUp();
    }
    driver->loadcell->tare(10);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // 히터 시작
    driver->heater->startHeating(recipe.water_temperature_c);
    driver->grinder->setClicks(recipe.grind_level);
    driver->arranging->move(200);   // 실제 하드웨어에 맞게 조정
    // block 대신 간단한 대기 (나중에 isMoving() 같은 함수 추가해서 개선)
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    // 린싱
    Serial.println("[BREW] Rinsing start");
    driver->pouring->startPump(RINSE_PUMP_PWM);
    uint32_t start = millis();
    while (millis() - start < RINSE_DURATION_MS) {
        driver->loadcell->tryUpdateWeight(LOADCELL_TIMEOUT_MS);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    driver->pouring->stopPump();
    Serial.println("[BREW] Rinsing done");

    Serial.println("[BREW] Waiting for temperature stabilization");
    while (!driver->heater->isTemperatureStable(1.0)) {
        // 중간 STOP 처리
        if (driver->status == BrewStatus::STOP) {
            Serial.println("[BREW] STOP during preheat");
            stopAll(driver);
            return;
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    Serial.println("[BREW] Heater stable");

    // 6) 다시 원두 위치로 arranging 이동
    Serial.println("[BREW] Moving back to bean position");
    driver->arranging->move(-200);   // TODO: 린싱 때 이동한 step 만큼 반대로
    vTaskDelay(1500 / portTICK_PERIOD_MS);
}

// 분쇄 + 서버 이동 + 브루잉
static void runGrindAndBrew(DriverContext* driver, RecipeInfo& recipe) {
    Serial.println("[BREW] === MAIN: GRIND + BREW ===");
    driver->grinder->startGrinding();

    // 정지 명령 -> 정지
    while (driver->grinder->isGrinding()) {
        if (driver->status == BrewStatus::STOP) {
            Serial.println("[BREW] STOP during grinding");
            stopAll(driver);
            return;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    Serial.println("[BREW] Grinding done");

    // 서버(필터) 위치로 arranging 이동
    Serial.println("[BREW] Moving to server position");
    driver->arranging->move(200);    // 실제 하드웨어에 맞게 조정
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    // 5) 레시피 데이터에 맞게 푸어링 수행
    Serial.println("[BREW] Pouring steps start");
    for (uint8_t i = 0; i < recipe.pouring_steps_count; ++i) {
        PouringStep& step = recipe.pouring_steps[i];

        Serial.printf("[BREW] Step %d: target=%.1fg, pour=%ds, wait=%ds, bloom=%ds\n",
                      step.step,
                      step.water_g,
                      step.pour_time_s,
                      step.wait_time_s,
                      step.bloom_time_s);

        // (옵션) 0번 스텝에서 wait_time_s 활용
        if (i == 0 && step.wait_time_s > 0) {
            vTaskDelay(step.wait_time_s * 1000 / portTICK_PERIOD_MS);
        }

        // 블룸 타임
        if (step.bloom_time_s > 0) {
            vTaskDelay(step.bloom_time_s * 1000 / portTICK_PERIOD_MS);
        }

        // 로드셀 영점 조정
        driver->loadcell->tare(10);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // 펌프 ON (PWM 값은 나중에 캘리브레이션)
        driver->pouring->startPump(200);

        // 목표 수량까지 로드셀 모니터링
        while (true) {
            if (driver->status == BrewStatus::STOP) {
                Serial.println("[BREW] STOP during pouring");
                stopAll(driver);
                return;
            }

            driver->loadcell->tryUpdateWeight(LOADCELL_TIMEOUT_MS);
            float w = driver->loadcell->getWeight();

            if (w >= step.water_g - POUR_TARGET_EPSILON) {
                break;
            }

            // pour_time_s를 활용해서 너무 오래 걸리면 탈출하는 로직도 추가 가능
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        driver->pouring->stopPump();

        // 각 스텝 사이 대기
        if (step.pour_time_s > 0) {
            vTaskDelay(step.pour_time_s * 1000 / portTICK_PERIOD_MS);
        }
    }

    Serial.println("[BREW] All pouring steps done");
}

// ===== BrewTask 메인 =====
void BrewTask(void* pv) {
    DriverContext* driver = (DriverContext*)pv;

    // 현재 레시피는 일단 내부 static 변수로 관리 (gRecipeQueue에서 업데이트하거나 Controller가 세팅)
    static RecipeInfo currentRecipe;
    static bool hasRecipe = false;
    static bool preRinseDone = false;

    while (true) {

        // gRecipeQueue를 체크해서 새 레시피가 오면 currentRecipe 갱신 + hasRecipe = true
        if (gRecipeQueue != nullptr) {
            RecipeInfo newRecipe;
            if (xQueueReceive(gRecipeQueue, &newRecipe, 0) == pdTRUE) {
                currentRecipe = newRecipe;
                hasRecipe = true;
                preRinseDone = false;  // 새 레시피가 오면 린싱 상태 초기화
                Serial.printf("[BREW] New recipe received from queue: temp=%dC, grind=%d, dose=%.1fg\n",
                             currentRecipe.water_temperature_c,
                             currentRecipe.grind_level,
                             currentRecipe.dose_g);
            }
        }

        BrewStatus st = driver->status;

        switch (st) {
        case BrewStatus::IDLE:
            break;

        case BrewStatus::RINSING:
            if (!hasRecipe) {
                // 레시피 없으면 할 수 있는 게 없음
                Serial.println("[BREW] RINSING requested but no recipe loaded");
                driver->status = BrewStatus::IDLE;
                break;
            }
            Serial.println("[BREW] Status = RINSING (pre-heat + rinse)");
            runPreHeatAndRinse(driver, currentRecipe);
            preRinseDone = true;
            // 린싱 후에는 다시 IDLE에 두고, 사용자 START_BREW를 기다리게 함
            driver->status = BrewStatus::IDLE;
            break;

        case BrewStatus::BREWSTART:
            if (!hasRecipe) {
                Serial.println("[BREW] BREWSTART but no recipe");
                driver->status = BrewStatus::IDLE;
                break;
            }
            if (!preRinseDone && currentRecipe.rinsing) {
                // 혹시 린싱이 안 되어 있는데 바로 BREWSTART 들어오면,
                // 안전하게 린싱 + 예열부터 한 번 더 수행
                runPreHeatAndRinse(driver, currentRecipe);
            }
            Serial.println("[BREW] Status = BREWSTART (grind + brew)");
            runGrindAndBrew(driver, currentRecipe);
            // 한 잔 끝난 뒤에는 상태 초기화
            stopAll(driver);
            driver->status = BrewStatus::IDLE;
            preRinseDone = false;
            break;

        case BrewStatus::STOP:
            Serial.println("[BREW] STOP requested");
            stopAll(driver);
            driver->status = BrewStatus::IDLE;
            preRinseDone = false;
            break;

        default:
            break;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
