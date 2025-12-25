

#include "../common/data_format.h"
#include <Arduino.h>
// #include "../handler/heaterTask/heaterTask.h"
// #include "../handler/loadcellTask/loadcellTask.h"
#include "../handler/connectionTask/connectionTask.h"
#include "../handler/brewTask/brewTask.h"

#include "../common/boot.h"
#include "../common/connection/BLEconnection.h"
#include "../common/connection/WIFIconnection.h"
#include "../handler/shared/shared_state.h"


// ===== 전역 객체 =====
QueueHandle_t gRecipeQueue  = nullptr;
QueueHandle_t gCommandQueue = nullptr;
QueueHandle_t gSendQueue    = nullptr;

DriverContext driver;
BootManager bootManager;
ConnectionContext connCtx;

// ===== SETUP =====
void setup() {
    // ----- Queue 생성 -----
    Serial.begin(115200);
    pinMode(pin::Rearranging_Motor_Dir, OUTPUT);
    pinMode(pin::Rearranging_Motor_Step, OUTPUT);
    pinMode(pin::Rearranging_EndStop_DI, INPUT_PULLUP
    );  // External 10k pull-up required

    pinMode(pin::Grinding_Motor_ADC, INPUT);
    pinMode(pin::Grinding_Motor_PWM, OUTPUT);
    pinMode(pin::Pump_PWM, OUTPUT);
    pinMode(pin::Heater_PWM, OUTPUT);

    pinMode(pin::Heater_Thermistor_ADC, INPUT);
    pinMode(pin::Weighing_DT, INPUT);
    pinMode(pin::Weighing_SCK, OUTPUT);
    pinMode(pin::Pouring_Rotation_PWM, OUTPUT);

    gRecipeQueue  = xQueueCreate(2, sizeof(RecipeInfo));
    gCommandQueue = xQueueCreate(10, sizeof(cmdItem));
    gSendQueue    = xQueueCreate(15, sizeof(sendItem));

    // ----- SharedState 초기화 -----
    gShared.mutex = xSemaphoreCreateMutex();
    gShared.currentWeight = 0.0f;
    gShared.currentTemp   = 20.0f;
    gShared.tempStable    = false;
    gShared.currentSendMode = SendMode::NONE;
    gShared.machine_id = bootManager.getmachine_id();
    
    Serial.println("=== Coffee Machine Boot ===");
    bootManager.begin();
    vTaskDelay(500);
    // ----- DriverContext 초기화 -----
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


    xTaskCreatePinnedToCore(
        connectionSupervisorTask,
        "connectionSupervisorTask",
        20480,
        &connCtx,
        2,
        &connCtx.supervisorTask,
        0
    );

    // ----- LoadCell, Heater, Brew Task 시작 -----
    //xTaskCreatePinnedToCore(LoadCellTask, "LoadCellTask", 10240, &driver, 3, &driver.loadCellTaskHandle, 1);
    //xTaskCreatePinnedToCore(HeaterTask,   "HeaterTask",   10240, &driver, 3, nullptr, 1);
    xTaskCreatePinnedToCore(brewTask, "BrewTask",     16384, &driver, 2, nullptr, 1);
    Serial.println("=== Setup Complete ===");
}

void loop() {
    // FreeRTOS 사용 시 loop는 비워둔다
}


/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/AnalogReadSerial/
*/

/*
#include<Arduino.h>

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  analogWrite(23, 255);
  analogWrite(19, 255);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(39);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(100);
  delay(1);  // delay in between reads for stability
}*/


