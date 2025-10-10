#include "./heater_driver.h"

// 작동 흐름
/*
    1. begin() - 초기화
    2. startHeating(targetTemp) - 목표 온도 설정 및 가열 시작
    3. update() - 주기적으로 호출하여 상태 업데이트 및 제어
    update() 내부에서:
        - updateHeatingState() - 상태 머신 업데이트
        - handleTemperatureControl() - PID 제어 및 PWM 출력
        - handleSafetyCheck() - 안전 체크
*/

namespace {
    // PID 제어 상수
    static double PID_INPUT = 0.0, PID_OUTPUT = 0.0, PID_SETPOINT = 0.0;
    static double KP = 2.0, KI = 5.0, KD = 1.0;
    
    // 히터 관련 상수
    
    const int MAX_PWM_OUTPUT = 255;                    // 최대 PWM 값
    const double MAX_SAFE_TEMP = 105.0;                // 최대 안전 온도
    const double MIN_SAFE_TEMP = -10.0;                // 최소 안전 온도
    const int SAMPLE_COUNT = 10;                       // 온도 측정 샘플 수
    const int MAX_CONSECUTIVE_ERRORS = 5;              // 최대 연속 에러
    const unsigned long PID_UPDATE_INTERVAL = 100;    // PID 업데이트 간격 (ms)
    const unsigned long SAFETY_CHECK_INTERVAL = 1000; // 안전 체크 간격 (ms)
    
    // EMA 필터
    double emaFilter(double x, double& y, double z = 1.0){
        if (!isfinite(y)) { y = x; return y; }
        y = y * (1.0 - z) + (z * x);
        return y;
    }
}


HeaterDriver::HeaterDriver(){}

void HeaterDriver::begin() {
    #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);  // ESP32: 12bit ADC
    analogWriteResolution(8);  // PWM: 8bit
    #endif
    
    pidController = new PID(&PID_INPUT, &PID_OUTPUT, &PID_SETPOINT, KP, KI, KD, DIRECT);
    pidController->SetMode(AUTOMATIC);
    pidController->SetOutputLimits(0, 255); // PWM 출력 범위
    pidController->SetSampleTime(100);

    pinMode(TempSensorPin, INPUT);
    pinMode(HeaterPWMPin, OUTPUT);
    analogWrite(HeaterPWMPin, 0);

    heaterState_ = HeaterState::IDLE;
    
    Serial.println("[Heater] Initialized for ESP32");
}

// === 히터 시작 === 
void HeaterDriver::startHeating(double targetTemp) {
    if(heaterState_ == HeaterState::HEATING || heaterState_ == HeaterState::MAINTAINING) {
        Serial.println("[Heater] Already heating.");
        return; // 이미 가열 중
    }

    heatingStartTime_ = millis();
    targetTemperature_ = targetTemp;
    PID_SETPOINT = targetTemp;
    consecutiveErrors_ = 0;

    heaterState_ = HeaterState::HEATING;
    Serial.println("[Heater] Starting heating to " + String(targetTemp)); // 디버깅용 출력
}

// === 히터 정지 === 
void HeaterDriver::stopHeating() {
    if(heaterState_ == HeaterState::IDLE) {
        Serial.println("[Heater] Not heating.");
        return; // 가열 중이 아님
    }

    analogWrite(HeaterPWMPin, 0);
    lastPWMOutput_ = 0;
    targetTemperature_ = 0;
    PID_SETPOINT = 0;
    consecutiveErrors_ = 0;
    heaterState_ = HeaterState::COMPLETED;

    Serial.println("[Heater] Heating stopped."); // 디버깅용 출력
}

// === 상태 업데이트 === 
void HeaterDriver::update() {
    if (heaterState_ == HeaterState::IDLE || heaterState_ == HeaterState::ERROR) {
        return;  // 동작할 필요 없음
    }
    updateHeatingState();
}

// === 온도 안정성 확인 ===
bool HeaterDriver::isTemperatureStable(double tolerance) const {
    if (heaterState_ != HeaterState::MAINTAINING) {
        return false;
    }
    return abs(currentTemperature_ - targetTemperature_) <= tolerance;
}


// 온도 값 읽어오기
// TMP36 사용 가정, PWM 10bit 가정 (0-1023)
double HeaterDriver::readThermistor(){
    #if defined(ARDUINO_ARCH_ESP32)
    const float ADC_MAX = 4095.0f;  // 12bit
    const float VREF = 3.3f;        // 3.3V
    #else
    const float ADC_MAX = 1023.0f;  // 10bit
    const float VREF = 5.0f;        // 5V
    #endif

    int tempRaw = analogRead(TempSensorPin);
    if(tempRaw <= 0 || tempRaw >= ADC_MAX) return NAN;    // 읽기 실패 (범위 밖)

    float voltage = (VREF * tempRaw) / ADC_MAX;         // 전압 따라 조정
    float temperatureC = (voltage - 0.5) / 0.01;      // TMP36 변환식
    return temperatureC;
}


// === 평균 온도 읽기 ===
double HeaterDriver::readTemperatureAverage(int samples) {
    if (samples <= 0) samples = 1;
    double sum = 0.0;
    int validSamples = 0;
    
    // 노이즈 제거를 위해 samples 수 만큼 읽어서 평균
    for (int i = 0; i < samples; i++) {
        double temp = readThermistor();
        if (isfinite(temp)) {
            sum += temp;
            validSamples++;
        }
        if (i < samples - 1) {
            delayMicroseconds(100); // 샘플 간 짧은 지연
        }
    }
    
    if (validSamples == 0) return NAN;
    return sum / validSamples;
}


// === 상태 머신 업데이트 ===
// update에서 주기적으로 호출하여 상태를 업데이트
void HeaterDriver::updateHeatingState() {
    unsigned long currentTime = millis();

    // === 안전 장치, handleSafetyCheck 호출 ===
    if(currentTime - lastSafetyCheckTime_ >= SAFETY_CHECK_INTERVAL) {
        handleSafetyCheck();
        lastSafetyCheckTime_ = currentTime;

        if(heaterState_ == HeaterState::ERROR) {
            analogWrite(HeaterPWMPin, 0);  // 직접 정지
            lastPWMOutput_ = 0;
            return;
        }
    }

    // === interval 마다 온도 읽기 및 상태 업데이트 ===
    if (currentTime - lastPIDUpdateTime_ >= PID_UPDATE_INTERVAL) {
        lastPIDUpdateTime_ = currentTime;
        handleTemperatureControl();
    }
}

void HeaterDriver::handleTemperatureControl() {
    double readTemp = readTemperatureAverage(SAMPLE_COUNT);
    //온도 읽어오기 실패
    if (!isfinite(readTemp)) {
        consecutiveErrors_++;
        if (consecutiveErrors_ >= MAX_CONSECUTIVE_ERRORS) {
            Serial.println("[Heater] Too many consecutive temperature reading errors");
            heaterState_ = HeaterState::ERROR;
        }
        return;
    }

    consecutiveErrors_ = 0;
    currentTemperature_ = readTemp;
    PID_INPUT = readTemp;
    double tempDiff = currentTemperature_ - targetTemperature_;

    // 목표 온도보다 많이 높은 경우 -> 히터 차단
    if(tempDiff > 2.0){
        analogWrite(HeaterPWMPin, 0);
        lastPWMOutput_ = 0;
        stableCount_ = 0;
        heaterState_ = HeaterState::MAINTAINING;
        Serial.println("[Heater] Target temperature reached. Heater off.");
        return;
    }

    // 목표 온도 근처 도달 확인
    if(abs(tempDiff) <= 1.0) {
        stableCount_++;
        if(stableCount_ >= 5)  // 5회 연속 안정 시 유지 상태로 전환
            heaterState_ = HeaterState::MAINTAINING;

    } else {
        stableCount_ = 0;
            if(tempDiff < 0) {
            heaterState_ = HeaterState::HEATING;
        }
    }

    // 목표 온도보다 낮은 경우
    if(tempDiff <= 1.0){
        pidController -> Compute();
        int pwmOutput = constrain(((int)PID_OUTPUT), 0, MAX_PWM_OUTPUT);
        
        // 온도가 목표 온도에 가까운 경우 -> PWM 제한
        if(tempDiff > 0.5) {
            pwmOutput = min(pwmOutput, 100);
        }
        analogWrite(HeaterPWMPin, pwmOutput);
        lastPWMOutput_ = pwmOutput;
        Serial.println("[Heater] Heating... Current Temp: " + String(currentTemperature_) + ", Target: " + String(targetTemperature_) + "°C, PWM: " + String(pwmOutput));
    }
}


// == 안전 체크 함수 == 
void HeaterDriver::handleSafetyCheck()  {
    double temp = readTemperatureAverage(SAMPLE_COUNT);
    
    // 안전장치 1. 온도 읽기 실패
    if(!isfinite(temp)) {
        consecutiveErrors_++;
        if(consecutiveErrors_ >= MAX_CONSECUTIVE_ERRORS) {
            Serial.println("[Heater] Error: Failed to read temperature multiple times.");
            heaterState_ = HeaterState::ERROR;
        }
        return ;
    }

    // 안전장치 2. 온도 범위 벗어남 (과열)
    if(temp > MAX_SAFE_TEMP){ 
        Serial.println("[Heater] Error: Temperature exceeds maximum safe limit!");
        heaterState_ = HeaterState::ERROR;
        return;
    }

    // 안전장치 3. 온도 범위 벗어남 (저온) -> 이상으로 간주
    if(temp < MIN_SAFE_TEMP){ 
        Serial.println("[Heater] Error: Temperature below minimum safe limit!");
        heaterState_ = HeaterState::ERROR;
        return;
    }
    consecutiveErrors_ = 0; // 정상적으로 읽었으므로 에러 카운트 초기화
}



