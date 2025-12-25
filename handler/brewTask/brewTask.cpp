#include "./brewTask.h"

extern QueueHandle_t gRecipeQueue;
extern QueueHandle_t gCommandQueue;
extern QueueHandle_t gSendQueue;
extern SharedState   gShared;
HX711 scale;

// === New: Circular buffer for scale averaging ===
#define SCALE_BUFFER_SIZE 8
static float scaleBuffer[SCALE_BUFFER_SIZE];
static uint8_t scaleBufferIndex = 0;
static uint8_t scaleBufferCount = 0;
static bool scaleBufferInitialized = false;

// Returns averaged weight (fast + noise-reduced)
float getAveragedWeight() {
  float raw = scale.get_units(1);                // single read – very fast
  // Insert into circular buffer
  scaleBuffer[scaleBufferIndex] = raw;
  scaleBufferIndex = (scaleBufferIndex + 1) % SCALE_BUFFER_SIZE;

  if (scaleBufferCount < SCALE_BUFFER_SIZE) {
    scaleBufferCount++;
  }

  // Compute average of valid samples
  float sum = 0.0;
  for (uint8_t i = 0; i < scaleBufferCount; i++) {
    sum += scaleBuffer[i];
  }
  return sum / scaleBufferCount;
}

// Fast-fill buffer after tare (so average is immediately meaningful)
void initScaleBuffer() {
  float current = scale.get_units(1);
  for (uint8_t i = 0; i < SCALE_BUFFER_SIZE; i++) {
    scaleBuffer[i] = current;
  }
  scaleBufferIndex = 0;
  scaleBufferCount = SCALE_BUFFER_SIZE;
  scaleBufferInitialized = true;
}

#define HEATER_KP 120.0f   // Proportional gain – tune this! (20~50 typical for water heating)

uint8_t computeHeaterPWM(float currentTemp, float targetTemp) {
  float error = targetTemp - currentTemp;  // Positive if too cold
  if (error <= 0) return 0;                // Already at or above target → OFF

  float dutyFloat = HEATER_KP * error;
  uint8_t duty = (uint8_t)constrain(dutyFloat, 0.0f, 255.0f);

  // Optional: Cap max power when far below to speed up initial heating
  // Remove or adjust if you want full power when very cold
  // if (error > 10.0f) duty = 255;

  return duty;
}

// === PID Controller for Heater ===
class PIDController {
public:
  float Kp, Ki, Kd;
  float integral = 0.0f;
  float previousError = 0.0f;
  unsigned long lastTime = 0;

  PIDController(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd) {}

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
    lastTime = 0;
  }

  uint8_t compute(float currentTemp, float targetTemp) {
    unsigned long now = millis();
    if (lastTime == 0) {
      lastTime = now;
      previousError = targetTemp - currentTemp;
      return 0;
    }

    float dt = (now - lastTime) / 1000.0f;  // seconds
    if (dt <= 0) dt = 0.001f;               // avoid division by zero
    lastTime = now;

    float error = targetTemp - currentTemp;

    // Proportional
    float P = Kp * error;

    // Integral (with simple anti-windup: clamp if output would saturate)
    integral += error * dt;
    // Rough anti-windup: limit integral term contribution
    float integralContribution = Ki * integral;
    if (integralContribution > 255) integralContribution = 255;
    if (integralContribution < 0) integralContribution = 0;

    // Derivative
    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;
    previousError = error;

    float output = P + integralContribution + D;

    // Final clamp
    if (output > 255) {
      output = 255;
      // Anti-windup: stop integrating if saturated high
      if (Ki > 0 && error > 0) integral -= error * dt;
    } else if (output < 0) {
      output = 0;
      if (Ki > 0 && error < 0) integral -= error * dt;
    }

    return (uint8_t)output;
  }
};

// Global PID instance (shared across all heating phases)
PIDController heaterPID(8.0f, 0.8f, 20.0f);  // Kp=60, Ki=2, Kd=15 – good starting point for weak heater

// Read temperature in °C (averaged)
float readTemperature(int samples = 4) {
  float average = 0.0;
  for (int i = 0; i < samples; i++) {
    average += analogRead(pin::Heater_Thermistor_ADC);
    vTaskDelay(pdMS_TO_TICKS(5));  // Reduced delay for faster response
  }
  average /= samples;

  if (average == 0) return -1;
  float resistance = SERIES_RESISTOR * (ADC_MAX / average - 1.0);

  float steinhart = resistance / THERMISTOR_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

// Stepper control (unchanged)
void setDir(bool highDir) {
    digitalWrite(pin::Rearranging_Motor_Dir, highDir ? HIGH : LOW);
}

void stepOnce() {
    digitalWrite(pin::Rearranging_Motor_Step, HIGH);
    delayMicroseconds(2);
    digitalWrite(pin::Rearranging_Motor_Step, LOW);
}

void home() {
    setDir(false);
    while (digitalRead(pin::Rearranging_EndStop_DI) == HIGH) {
      stepOnce();
      vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool moveSteps(int steps, bool dir, int minDelay = 1200, int startDelay = 5000, int accelSteps = 100) {
    if (steps <= 0) return false;
    
    setDir(dir);
    
    int rampSteps = min(accelSteps, steps / 3);
    int delayDiff = startDelay - minDelay;
    
    for (int i = 0; i < steps; i++) {
      stepOnce();
      
      int currentDelay;
      if (i < rampSteps) {
        currentDelay = startDelay - (delayDiff * i / rampSteps);
      } else if (i >= steps - rampSteps) {
        currentDelay = minDelay + (delayDiff * (steps - 1 - i) / rampSteps);
      } else {
        currentDelay = minDelay;
      }
      
      delayMicroseconds(currentDelay);
    }
    return true;
}

int readEachClick(){
    int adc = analogRead(pin::Click_Motor_Potentiometer_ADC);
    if(adc < 64) return -1;
    
    if (adc < ADC_MIN) adc = ADC_MIN;
    if (adc > ADC_MAX) adc = ADC_MAX;

    float ratio = float(adc - ADC_MIN) / float(ADC_MAX - ADC_MIN);
    ratio = 1.0f - ratio;
    int clicks = int(lround(ratio * (CLICK_MAX - CLICK_MIN))) + CLICK_MIN;
    clicks += BIAS_CLICKS;

    if (clicks < CLICK_MIN) clicks = CLICK_MIN;
    if (clicks > CLICK_MAX) clicks = CLICK_MAX;
    return clicks;
}

int readCurrentClick() {
    const int NUM_SAMPLES = 8;
    const int OUTLIER_THRESHOLD = 10;
    
    int samples[NUM_SAMPLES];
    int validCount = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int value = readEachClick();
        if (value == -1) continue;
        samples[validCount++] = value;
    }
    
    if (validCount == 0) return -1;
    
    // Sort to find median
    for (int i = 0; i < validCount - 1; i++) {
        for (int j = i + 1; j < validCount; j++) {
            if (samples[i] > samples[j]) {
                int temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }
    
    int median = (validCount % 2 == 0)
        ? (samples[validCount/2 - 1] + samples[validCount/2]) / 2
        : samples[validCount/2];
    
    long sum = 0;
    int finalCount = 0;
    for (int i = 0; i < validCount; i++) {
        if (abs(samples[i] - median) < OUTLIER_THRESHOLD) {
            sum += samples[i];
            finalCount++;
        }
    }
    
    if (finalCount > 0) {
        return (int)lround((float)sum / finalCount);
    }
    // fallback
    sum = 0;
    for (int i = 0; i < validCount; i++) sum += samples[i];
    return (int)lround((float)sum / validCount);
}

bool HX711Start(){
    scale.begin(pin::Weighing_DT, pin::Weighing_SCK);
    scale.set_offset(-107000);
    scale.set_scale(420);
    scale.tare();
    delay(1000);
    Serial.print("after tare: ");
    Serial.println(scale.get_units(4));
    delay(1000);
    initScaleBuffer();
    heaterPID.reset();  // Reset PID on startup
    return true;
}

// Send functions (unchanged)
static void sendCurrentClick(DriverContext* driver, const char* status, 
    const int currentClick, const int goalClick) {
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = "CLICK_ADJUST_STATUS";
    doc["status"]    = status;
    doc["value"] = currentClick;
    doc["goal"] = goalClick;
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

static void sendCurrentBeanGram(DriverContext* driver, const char* status,
    const double currentBeanGram, const double goalBeanGram) {
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = "WEIGHING_STATUS";
    doc["status"]    = status;
    doc["value"] = currentBeanGram;
    doc["goal"] = goalBeanGram;
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

static void sendFinishBrewing(DriverContext* driver) {  
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = "FINISH_BREWING";
    doc["status"]    = "FINISH_BREWING";
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

static void sendFinishRinsing(DriverContext* driver) {  
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = "FINISH_RINSING";
    doc["status"]    = "FINISH_RINSING";
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

static void sendBrewingStatus(DriverContext* driver, int currentSteps, bool isPouring) {  
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = "BREWING_STATUS";
    doc["status"]    = "BREWING_STATUS";
    doc["steps"]     = currentSteps;
    doc["isPouring"] = isPouring;
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

static void sendJustStatus(DriverContext* driver, const char* status, float value) {
    static StaticJsonDocument<128> doc;
    static sendItem item;
    doc["machine_id"] = driver->machine_id;
    doc["type"]      = status;
    doc["status"]    = status;
    doc["value"] = value;
    JSON_TO_SENDITEM(item, doc);
    xQueueSendToBack(gSendQueue, &item, 0);
}

void handleCommandQueue(cmdItem item, bool hasRecipe, DriverContext* driver){
    String cmdStr(item.buf);
    if(cmdStr == "START_BREW") {
        if(hasRecipe) {
            driver->status = BrewStatus::STARTBREW;
            Serial.println("[BREW STATUS] START COMMAND RECEIVED");
        } else {
            Serial.println("[BREW STATUS] Cannot Start: No Recipe");
        }
    } 
    else if(cmdStr == "FINISH_CLICK_ADJUST") {
        driver->status = BrewStatus::WEIGHING;
    }
    else if(cmdStr == "FINISH_WEIGHING"){
        driver->status = BrewStatus::IDLE;
    }
    else if(cmdStr == "START_RINSING") {
        driver->status = BrewStatus::RINSING;
    }
    else if(cmdStr == "START_GRINDING") {
        driver->status = BrewStatus::GRINDING;
    }
    else if(cmdStr == "START_POURING") {
        driver->status = BrewStatus::POURING;
    }
    else if(cmdStr == "STOP_BREW") {
        driver->status = BrewStatus::STOP;
    }
    else if(cmdStr == "TARE"){
        driver->status = BrewStatus::TARE;
    }
    else if(cmdStr == "STOP_GRINDING" ){
        driver->status = BrewStatus::STOP_GRINDING;
    }
    else if(cmdStr == "JUST_GRINDING") {
        driver->status = BrewStatus::JUST_GRINDING;
    }
    else if(cmdStr == "JUST_POURING"){
        driver->status = BrewStatus::JUST_POURING;
    }
    else if(cmdStr == "JUST_TARE_SCALE"){
        driver->status = BrewStatus::JUST_TARE_SCALE;
    }
    else if(cmdStr == "JUST_TARE_SCALE_STOP" || cmdStr == "JUST_GRINDING_STOP" || cmdStr == "JUST_POURING_STOP"){
        driver->status = BrewStatus::STOP;
    }
}

void brewTask(void* pv) {
  
  HX711Start();  // also initializes averaging buffer
  DriverContext* driver = (DriverContext*)pv;
  RecipeInfo currentRecipe;
  static StaticJsonDocument<256> doc;
  static sendItem item;
  static cmdItem cmd;
  bool hasRecipe = false;
  bool hasTared = false;
  RecipeInfo next;

  while(true) {
    if(xQueueReceive(gRecipeQueue, &next, 0) == pdTRUE){
      currentRecipe = next;
      hasRecipe = true;
      driver->recipe = currentRecipe;
      Serial.println("[BREW STATUS] Recipe loaded.");
    }
    if(xQueueReceive(gCommandQueue, &cmd, 0) == pdTRUE){
      Serial.println("[BREW STATUS] Command received: " + String(cmd.buf));
      handleCommandQueue(cmd, hasRecipe, driver);
    }

    switch(driver->status){
      case BrewStatus::IDLE:
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        break;

      case BrewStatus::STARTBREW:
        Serial.println("[BREW STATUS] Preparing Brewing...");
        home();
        while(1) { 
          if(moveSteps(50, true)) break;
        }
        driver->status = BrewStatus::CLICKADJUST;
        break;

      case BrewStatus::CLICKADJUST: {
        float current = readCurrentClick();
        if(current == -1){
          Serial.println("[ERROR] Invalid Click Reading!");
          vTaskDelay(150 / portTICK_PERIOD_MS);
          break;
        }
        sendCurrentClick(driver, "CLICK_ADJUSTING", current, driver->recipe.grind_level);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        break;
      }
      
      case BrewStatus::TARE: {
        Serial.println("[BREW STATUS] TARING SCALE...");
        scale.tare();
        initScaleBuffer();                // re-initialize buffer after tare
        vTaskDelay(200 / portTICK_PERIOD_MS);
        sendCurrentBeanGram(driver, "WEIGHING", getAveragedWeight(), driver->recipe.dose_g);
        driver->status = BrewStatus::WEIGHING;
        break;
      }

      case BrewStatus::WEIGHING:
        Serial.println("[BREW STATUS] WEIGHING...");
        if(!hasTared){
          scale.tare();
          initScaleBuffer();
          hasTared = true;
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        sendCurrentBeanGram(driver, "WEIGHING", getAveragedWeight(), driver->recipe.dose_g);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        break;
    case BrewStatus::RINSING: {
        Serial.println("[BREW STATUS] RINSING...");
        float rinseAmount = 200.0;
        float startWeight = getAveragedWeight();
        float targetTemp = driver->recipe.water_temperature_c;
        heaterPID.reset();

        // Initial fast heat-up phase (optional full power until close)
        analogWrite(pin::Heater_PWM, 255);
        while (readTemperature() < targetTemp - 10.0) {
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Start pouring mechanics
        analogWrite(pin::Pouring_DC, 100);
        analogWrite(pin::Pump_PWM, 50);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        analogWrite(pin::Pump_PWM, 20);

        float scaleVal = getAveragedWeight();

        while (scaleVal > startWeight - rinseAmount) {
          float currentTemp = readTemperature();

          // === Proportional Control ===
          uint8_t heaterDuty = computeHeaterPWM(currentTemp, targetTemp);
          analogWrite(pin::Heater_PWM, heaterDuty);

          scaleVal = getAveragedWeight();

          Serial.print("RINSING | Temp: "); Serial.print(currentTemp, 1);
          Serial.print(" °C | Target: "); Serial.print(targetTemp, 1);
          Serial.print(" °C | PID Out: "); Serial.print(heaterDuty);
          Serial.print(" | Scale: "); Serial.println(scaleVal, 1);
          Serial.print(" | Analog Value : "); Serial.println(analogRead(pin::Heater_Thermistor_ADC));

          vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        // Done
        analogWrite(pin::Pump_PWM, 0);
        analogWrite(pin::Heater_PWM, 0);
        analogWrite(pin::Pouring_DC, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sendFinishRinsing(driver);
        driver->status = BrewStatus::IDLE;
        break;
      }

      case BrewStatus::POURING: {
        scale.tare();
        initScaleBuffer();
        Serial.println("[BREW STATUS] POURING...");

        for(int i = 0; i < driver->recipe.pouring_steps_count; i++){
          float targetTemp = driver->recipe.water_temperature_c;  // or per-step target if added later
          float pourAmount = driver->recipe.pouring_steps[i].water_g;
          float currentTarget = -pourAmount;

          sendBrewingStatus(driver, i, true);

          // Start pouring
          scale.tare();
          initScaleBuffer();
          vTaskDelay(20 / portTICK_PERIOD_MS);

          analogWrite(pin::Pump_PWM, 50);
          vTaskDelay(200 / portTICK_PERIOD_MS);
          analogWrite(pin::Pump_PWM, 20);
          
          heaterPID.reset();

          // Heat up with P-control
          while (readTemperature() < 50.0f) {
            float temp = readTemperature();
            uint8_t duty = heaterPID.compute(temp, targetTemp);
            analogWrite(pin::Heater_PWM, duty);
            vTaskDelay(10 / portTICK_PERIOD_MS);
          }
          
          analogWrite(pin::Pouring_DC, 120);

          float scaleVal = getAveragedWeight();

          while (scaleVal > currentTarget) {
            float temp = readTemperature();

            // === Proportional Control during pour ===
            uint8_t heaterDuty = heaterPID.compute(temp, targetTemp);            
            analogWrite(pin::Heater_PWM, heaterDuty);

            scaleVal = getAveragedWeight();

            Serial.print("Step "); Serial.print(i);
            Serial.print(" | Temp: "); Serial.print(temp, 1);
            Serial.print(" °C | Heater: "); Serial.print(heaterDuty);
            Serial.print(" | Scale: "); Serial.print(scaleVal, 1);
            Serial.print(" / "); Serial.println(currentTarget, 1);

            vTaskDelay(20 / portTICK_PERIOD_MS);
          }

          // Stop actuators
          analogWrite(pin::Pump_PWM, 0);
          analogWrite(pin::Heater_PWM, 0);
          analogWrite(pin::Pouring_DC, 0);
          sendBrewingStatus(driver, i, false);

          vTaskDelay(driver->recipe.pouring_steps[i].pour_time_s * 1000 / portTICK_PERIOD_MS);
        }

        analogWrite(pin::Heater_PWM, 0);
        sendFinishBrewing(driver);
        driver->status = BrewStatus::STOP;
        break;
      }
      case BrewStatus::GRINDING:
        while(1){ if(moveSteps(650, true)) break; }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        analogWrite(pin::Grinding_Motor_PWM, 255);
        driver->status = BrewStatus::IDLE;
        break;
      
      case BrewStatus::STOP_GRINDING:
        analogWrite(pin::Grinding_Motor_PWM, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        while(1){ if(moveSteps(650, false)) break; }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        driver->status = BrewStatus::POURING;
        break;
      

      case BrewStatus::STOP:
        analogWrite(pin::Pump_PWM, 0);
        analogWrite(pin::Heater_PWM, 0);
        analogWrite(pin::Grinding_Motor_PWM, 0);
        analogWrite(pin::Pouring_DC, 0);
        heaterPID.reset();
        hasTared = false;
        hasRecipe = false;
        driver->status = BrewStatus::IDLE;
        break;

      case BrewStatus::JUST_GRINDING:
        analogWrite(pin::Grinding_Motor_PWM, 255);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sendJustStatus(driver, "JUST_GRINDING", 1.0);
        break;

      case BrewStatus::JUST_POURING: {
        float targetTemp = 92.0;  // or configurable
        heaterPID.reset();
        while(1){
          float temp = readTemperature();
          uint8_t duty = heaterPID.compute(temp, targetTemp);
          analogWrite(pin::Heater_PWM, duty);
          analogWrite(pin::Pump_PWM, 30);

          float currentPouring = -getAveragedWeight();
          sendJustStatus(driver, "JUST_POURING", currentPouring);

          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        break;
      }

      case BrewStatus::JUST_TARE_SCALE:
        if(!hasTared){
          scale.tare();
          initScaleBuffer();
          hasTared = true;
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        sendJustStatus(driver, "WEIGHING", getAveragedWeight());
        vTaskDelay(100 / portTICK_PERIOD_MS);
        break;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}