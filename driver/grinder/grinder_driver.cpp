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
}


GrinderDriver::GrinderDriver():stepper_(AccelStepper::DRIVER, ClickStepPin, ClickDirPin) {}

void GrinderDriver::begin() {

    #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12); // 0..4095
    analogSetPinAttenuation(ClickPotADC, ADC_11db);
    #endif

    // 스탭모터 + 포텐셜미터 핀 초기화
    pinMode(ClickStepPin, OUTPUT);
    pinMode(ClickDirPin, OUTPUT);
    pinMode(ClickPotADC, INPUT);

    // 그라인딩 모터
    pinMode(GrindPWMPin, OUTPUT);
    pinMode(GrindMoterADC, INPUT);

    digitalWrite(ClickStepPin, LOW);
    digitalWrite(ClickDirPin,  LOW);

    stepper_.setMaxSpeed(kMaxSpeed_);
    stepper_.setAcceleration(kAcceleration_);

    currentClicks_ = adcToClicks(readADCValue());
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


//테스트용 코드
void GrinderDriver::grind(uint32_t duration_ms){
    while(duration_ms > 0){
        analogWrite(GrindPWMPin, 200); // or LOW based on desired direction
        duration_ms -= 100;
    }
    analogWrite(GrindPWMPin, 0); // Stop the motor
    return;
}