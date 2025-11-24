// Firmware task skeleton for PerBrew (ESP32 Arduino)
// 초기 통합: 블로킹 동작은 전용 태스크 안에서 격리하여 전체 시스템 스톨을 방지
// 추가 개선(비블로킹화, 상태 머신 세분화)은 후속 단계에서 적용 가능

#include <Arduino.h>
#include "driver/pin_map.h"
#include "driver/arranging/arranging_driver.h"
#include "driver/grinder/grinder_driver.h"
#include "driver/heater/heater_driver.h"
#include "driver/loadcell/loadcell_driver.h"
#include "driver/pouring/pouringSection_driver.h"
#include "driver/common/boot.h"
#include "driver/common/WIFIconnection.h"
#include "driver/common/BLEconnection.h"

#ifdef ARDUINO_ARCH_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#endif

// ===== 이벤트 그룹 비트 =====
static EventGroupHandle_t gSystemEvents;
constexpr EventBits_t EVT_BOOT_DONE    = (1 << 0);
constexpr EventBits_t EVT_NET_READY    = (1 << 1);
constexpr EventBits_t EVT_BREW_ACTIVE  = (1 << 2);
constexpr EventBits_t EVT_STOP_REQUEST = (1 << 3);

// ===== 커맨드 정의 =====
enum class CommandType : uint8_t {
	START_BREW,
	STOP_BREW,
	SET_GRIND_CLICKS,
	SET_HEATER_TEMP,
	PUMP_START,
	PUMP_STOP,
	POUR_ROTATE_DEG,
	POUR_TILT_DEG
};

struct Command {
	CommandType type;
	int intValue{0};      // 범용 정수 파라미터
	float floatValue{0};  // 범용 실수 파라미터
};

static QueueHandle_t gCommandQueue; // 브루잉 / 동작 명령 큐

// ===== 공용 상태 구조 =====
struct SystemState {
	GrinderState grinderState = GrinderState::IDLE;
	HeaterState heaterState = HeaterState::IDLE;
	PouringStatus pouringStatus{};
	float currentWeight = 0.0f;
	bool brewingActive = false;
	double targetTemp = 0.0;
	int targetClicks = 0;
};

static SystemState gState; 
static SemaphoreHandle_t gStateMutex; // 상태 보호

// ===== 드라이버 인스턴스 =====
static BootManager bootMgr; // 모드 결정 (WiFi / BLE)
static BLEConnectionManager bleMgr;
static HttpConnectionManager httpMgr;

static ArrangingDriver arranging;
static GrinderDriver grinder;
static HeaterDriver heater;
static LoadCellDriver loadcell;
static PouringSectionDriver pouring;

// ===== 유틸리티 =====
static inline void lockState()   { xSemaphoreTake(gStateMutex, portMAX_DELAY); }
static inline void unlockState() { xSemaphoreGive(gStateMutex); }

// ===== 태스크 프로토타입 =====
void TaskBoot(void*);
void TaskConnection(void*);
void TaskSensors(void*);
void TaskActuators(void*);
void TaskBrewCoordinator(void*);

// ===== 부팅 태스크 =====
void TaskBoot(void* pv) {
	Serial.println("[BootTask] Starting boot sequence...");
	String mode = bootMgr.begin();
	// 하드웨어 초기화
	arranging.begin();
	grinder.begin();
	heater.begin();
	loadcell.begin();
	pouring.begin();

	xEventGroupSetBits(gSystemEvents, EVT_BOOT_DONE);
	Serial.printf("[BootTask] Boot complete. Mode=%s\n", mode.c_str());
	vTaskDelete(nullptr);
}

// ===== 연결 태스크 (BLE 또는 WiFi) =====
void TaskConnection(void* pv) {
	// 부팅 완료 대기
	xEventGroupWaitBits(gSystemEvents, EVT_BOOT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

	String mode = bootMgr.getCurrentMode();
	if (mode == WIFI_MODE) {
		// 서버 IP는 추후 Preferences 또는 수신 데이터로 대체
		httpMgr.begin("192.168.0.10", 8080);
		Serial.println("[ConnTask] WiFi connection init attempted");
	} else {
		bleMgr.begin();
		Serial.println("[ConnTask] BLE advertising started");
	}
	xEventGroupSetBits(gSystemEvents, EVT_NET_READY);

	for (;;) {
		// 폴링 (비블로킹)
		if (mode == WIFI_MODE) {
			httpMgr.poll();
			if (httpMgr.isConnected()) {
				String cmd = httpMgr.getLastReceivedCommand();
				if (cmd == "START_BREW") {
					Command c{CommandType::START_BREW};
					xQueueSend(gCommandQueue, &c, 0);
				} else if (cmd == "STOP_BREW") {
					Command c{CommandType::STOP_BREW};
					xQueueSend(gCommandQueue, &c, 0);
				}
			}
		} else { // BLE 모드에서 자격증명 수신 후 WiFi 전환 준비 가능 (미구현)
			bleMgr.poll();
			if (bleMgr.hasReceivedCredentials()) {
				// TODO: 저장 후 재부팅 또는 WiFi 전환 로직
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

// ===== 센서 태스크 (주기적으로 측정) =====
void TaskSensors(void* pv) {
	xEventGroupWaitBits(gSystemEvents, EVT_BOOT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
	const TickType_t weightInterval = pdMS_TO_TICKS(200);
	const TickType_t heaterInterval = pdMS_TO_TICKS(100);
	TickType_t lastWeight = 0, lastHeater = 0;

	for (;;) {
		TickType_t now = xTaskGetTickCount();
		// 로드셀 (준비되면 읽기, 블로킹은 제한적)
		if (now - lastWeight >= weightInterval) {
			lastWeight = now;
			if (loadcell.isReady()) {
				loadcell.tryUpdateWeight(50, 3); // 최대 50ms 대기
				lockState();
				gState.currentWeight = loadcell.getWeight();
				unlockState();
			}
		}
		// 히터 상태 (PID update는 내부 update 호출로 충분)
		if (now - lastHeater >= heaterInterval) {
			lastHeater = now;
			heater.update();
			lockState();
			gState.heaterState = heater.getState();
			unlockState();
		}
		// 분류: Pouring 스텝퍼 갱신
		pouring.update();
		lockState();
		gState.pouringStatus = pouring.getStatus();
		unlockState();

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// ===== 액추에이터 태스크 (모터류 블로킹 동작 격리) =====
void TaskActuators(void* pv) {
	xEventGroupWaitBits(gSystemEvents, EVT_BOOT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
	for (;;) {
		// Grinder 비블로킹 update
		grinder.update();
		arranging.update();
		// 향후 큐 기반 세밀 명령 처리 가능
		lockState();
		gState.grinderState = grinder.getState();
		unlockState();
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

// ===== 브루잉 코디네이터 태스크 =====
void TaskBrewCoordinator(void* pv) {
	xEventGroupWaitBits(gSystemEvents, EVT_BOOT_DONE | EVT_NET_READY, pdFALSE, pdTRUE, portMAX_DELAY);
	Serial.println("[BrewTask] Ready for commands");
	Command cmd;
	for (;;) {
		if (xQueueReceive(gCommandQueue, &cmd, pdMS_TO_TICKS(50)) == pdPASS) {
			switch (cmd.type) {
				case CommandType::START_BREW: {
					lockState(); gState.brewingActive = true; unlockState();
					xEventGroupSetBits(gSystemEvents, EVT_BREW_ACTIVE);
					// 예시 시퀀스: 그라인드 -> 히터 목표 -> 펌프 준비
					grinder.startGrinding(8000);
					heater.startHeating(92.0); // 고정 목표 예시
					Serial.println("[BrewTask] Brew sequence started");
					break;
				}
				case CommandType::STOP_BREW: {
					lockState(); gState.brewingActive = false; unlockState();
					xEventGroupSetBits(gSystemEvents, EVT_STOP_REQUEST);
					grinder.stopGrinding();
					heater.stopHeating();
					pouring.stopPump();
					Serial.println("[BrewTask] Brew stopped");
					break;
				}
				case CommandType::SET_GRIND_CLICKS: {
					grinder.setClicks(cmd.intValue);
					break;
				}
				case CommandType::SET_HEATER_TEMP: {
					heater.startHeating(cmd.floatValue);
					lockState(); gState.targetTemp = cmd.floatValue; unlockState();
					break;
				}
				case CommandType::PUMP_START: {
					pouring.startPump(cmd.intValue > 0 ? cmd.intValue : 128);
					break;
				}
				case CommandType::PUMP_STOP: {
					pouring.stopPump();
					break;
				}
				case CommandType::POUR_ROTATE_DEG: {
					pouring.rotateNozzleToAngle(cmd.floatValue);
					break;
				}
				case CommandType::POUR_TILT_DEG: {
					pouring.tiltNozzleToAngle(cmd.floatValue);
					break;
				}
			}
		}
		// 간단한 상태 전이 예: 그라인딩 완료 시 다음 단계 로직 추가 가능
		if (gState.grinderState == GrinderState::COMPLETED && gState.brewingActive) {
			// 향후: 프리인퓨전, 물 펌핑 등 단계 관리
		}
	}
}

// ===== Arduino 진입점 =====
void setup() {
	Serial.begin(115200);
	delay(300);
	Serial.println("\n[Setup] PerBrew firmware starting...");

	gSystemEvents = xEventGroupCreate();
	gCommandQueue  = xQueueCreate(10, sizeof(Command));
	gStateMutex    = xSemaphoreCreateMutex();

	// 태스크 생성 (우선순위는 임시값, 필요 시 재조정)
	xTaskCreatePinnedToCore(TaskBoot,          "BootTask",        4096, nullptr, 3, nullptr, 0);
	xTaskCreatePinnedToCore(TaskConnection,    "ConnectionTask",  4096, nullptr, 2, nullptr, 0);
	xTaskCreatePinnedToCore(TaskSensors,       "SensorsTask",     4096, nullptr, 2, nullptr, 1);
	xTaskCreatePinnedToCore(TaskActuators,     "ActuatorsTask",   4096, nullptr, 1, nullptr, 1);
	xTaskCreatePinnedToCore(TaskBrewCoordinator,"BrewTask",       4096, nullptr, 2, nullptr, 1);
}

void loop() {
	// 메인 루프 비움: 모든 로직은 FreeRTOS 태스크에서 처리
	vTaskDelay(pdMS_TO_TICKS(1000));
}

