#include <Arduino.h>
#include <math.h>
#include "./grinder_driver.h"


namespace {
    constexpr int CLICK_MIN = 0;
    constexpr int CLICK_MAX = 240;

    //실측 필요(포텐셜미터 ADC) , 아두이노 기준에 맞춤(0~1023)
    // ADC 17 = 1click
    //constexpr int ADC_MIN  = 1530;
    //constexpr int ADC_MAX  = 4080;
    constexpr int ADC_MIN  = 0;
    constexpr int ADC_MAX  = 1023;


    //그라인더 관련 상수
    const int GRINDING_PWM = 200;           // 그라인딩 모터 PWM 값
    const int IDLE_CURRENT_THRESHOLD = 50;  // 무부하 전류 임계값 (ADC 값 기준)
    const int SAMPLE_COUNT = 10;            // 전류 측정 샘플 수
    const int STABLE_COUNT_THRESHOLD = 5;   // 연속으로 무부하 상태가 지속되어야 하는 횟수
    const unsigned long CHECK_INTERVAL = 100; // 전류 체크 간격 (ms)
}


GrinderDriver::GrinderDriver():stepper_(AccelStepper::DRIVER, ClickStepPin, ClickDirPin) {}

void GrinderDriver::begin() {

    #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12); // 0..4095
    analogSetPinAttenuation(ClickPotADC, ADC_11db);
    #endif

    // === 스탭모터 + 포텐셜미터 핀 초기화 ===
    pinMode(ClickStepPin, OUTPUT);
    pinMode(ClickDirPin, OUTPUT);
    pinMode(ClickPotADC, INPUT);

    // === 그라인딩 모터 ===
    pinMode(GrindPWMPin, OUTPUT);
    pinMode(GrindMoterADC, INPUT);

    digitalWrite(ClickStepPin, LOW);
    digitalWrite(ClickDirPin,  LOW);

    stepper_.setMaxSpeed(kMaxSpeed_);
    stepper_.setAcceleration(kAcceleration_);

    currentClicks_ = adcToClicks(readADCValue());
    grindingState_ = GrinderState::IDLE;
}

int GrinderDriver::getCurrentClicks() {
    currentClicks_= adcToClicks(readADCValue());
    return currentClicks_;
}

void GrinderDriver::setClicks(int target) {
    long targetClicks = constrain(target, CLICK_MIN, CLICK_MAX);
    int current = getCurrentClicks();

    if (abs(targetClicks - current) < 1) return; // 이미 목표 위치

    // 이동 방향 설정
    int dir = (targetClicks > current) ? 1 : -1;
    stepper_.setSpeed(dir * kMaxSpeed_ * 0.5); // 절반 속도로 이동

    while (true) {
        stepper_.runSpeed(); // 한 스텝 실행
        int now = getCurrentClicks();
        if ((dir > 0 && now >= targetClicks) ||
            (dir < 0 && now <= targetClicks)) {
            break; // 목표 도달
        }
    }
    stepper_.stop();
    currentClicks_ = getCurrentClicks(); 
}
 
// === 전류센서 값 읽기 ===
int GrinderDriver::readCurrentSensor() {
    return analogRead(GrindMoterADC);
}

// === 평균 전류값 읽기 (노이즈 제거용 다중 샘플링) ===
int GrinderDriver::readCurrentAverage(int samples) {
    if (samples <= 0) samples = 1;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(GrindMoterADC);
        if (i < samples - 1) {
            delayMicroseconds(100); // 샘플 간 짧은 지연
        }
    }
    return sum / samples;
}

// === 그라인딩 완료 여부 확인 (전류값 기반) ===
bool GrinderDriver::isGrindingComplete(int threshold) {
    int current = readCurrentAverage(5); // 5개 샘플의 평균
    return (current <= threshold);
}


// === 분쇄 시작 ===
void GrinderDriver::startGrinding(uint32_t max_duration_ms) {
    if (grindingState_ == GrinderState::GRINDING) {
        Serial.println("Already grinding");
        return; // 이미 그라인딩 중
    }

    grindingStartTime_ = millis();
    maxGrindingDuration_ = max_duration_ms;
    stableIdleCount_ = 0;
    lastCurrentReading_ = 0;

    // 그라인딩 모터 시작
    analogWrite(GrindPWMPin, GRINDING_PWM);
    grindingState_ = GrinderState::GRINDING;

    Serial.println("Grinding started..."); // 디버깅용 출력
}

// === 분쇄 중지 ===
void GrinderDriver::stopGrinding() {
    if(grindingState_ != GrinderState::GRINDING) {
        Serial.println("Error: Not currently grinding");
        return; // 그라인딩 중이 아님
    }

    // 그라인딩 모터 정지
    analogWrite(GrindPWMPin, 0);
    grindingState_ = GrinderState::COMPLETED;

    Serial.println("Grinding stopped."); // 디버깅용 출력
}

void GrinderDriver::update() {
    if (grindingState_ == GrinderState::IDLE || grindingState_ == GrinderState::COMPLETED) {
        return;  // 동작할 필요 없음
    }
    updateGrindingState();
}


int GrinderDriver::adcToClicks(int adc){
    float ratio = float(adc - ADC_MIN) / float(ADC_MAX - ADC_MIN);
    int clicks = int(lround(ratio * (CLICK_MAX - CLICK_MIN))) + CLICK_MIN;
    if (clicks < CLICK_MIN) clicks = CLICK_MIN;
    if (clicks > CLICK_MAX) clicks = CLICK_MAX;
    return clicks;
}

int GrinderDriver::clicksToADC(int clicks) {
    float ratio = float(clicks - CLICK_MIN) / float(CLICK_MAX - CLICK_MIN);
    int adc = int(lround(ADC_MIN + ratio * (ADC_MAX - ADC_MIN)));
    if (adc < ADC_MIN) adc = ADC_MIN;
    if (adc >= ADC_MAX) adc = ADC_MAX;
    return adc;
}

void GrinderDriver::updateGrindingState() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - grindingStartTime_;
    
    if (elapsedTime >= maxGrindingDuration_) {
        analogWrite(GrindPWMPin, 0);
        grindingState_ = GrinderState::COMPLETED;
        Serial.printf("[Grinder] Timeout after %lu ms\n", elapsedTime); // 디버깅용 출력
        return;
    }
    
    if (currentTime - lastCurrentCheckTime_ >= CHECK_INTERVAL) {
        lastCurrentCheckTime_ = currentTime;
        handleCurrentMonitoring();
    }
}

void GrinderDriver::handleCurrentMonitoring() {
    long currentSum = 0;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
        currentSum += analogRead(GrindMoterADC);
        delayMicroseconds(100); // 샘플 간 짧은 지연
    }
    lastCurrentReading_ = currentSum / SAMPLE_COUNT;
    
    /*
        무부하 상태 판단...
        무부하 상태가 감지되면 stableIdleCount_ 증가
        stableIdleCount_가 STABLE_COUNT_THRESHOLD 이상이면 그라인딩 완료로 간주
    */
    if(lastCurrentReading_ <= IDLE_CURRENT_THRESHOLD){
        stableIdleCount_++;
        Serial.println("[Grinder] Low current detected, count: " + String(stableIdleCount_)); // 디버깅용 출력
        
        if(stableIdleCount_ >= STABLE_COUNT_THRESHOLD){
            stopGrinding();
            Serial.println("[Grinder] Grinding complete detected by current sensor."); // 디버깅용 출력
        }
    }
    // 부하가 다시 감지된 경우 -> counter 초기화, 계속 분쇄
    else{
        stableIdleCount_ = 0;
        Serial.println("[Grinder] Current normal, resetting count."); // 디버깅용 출력
    }
}