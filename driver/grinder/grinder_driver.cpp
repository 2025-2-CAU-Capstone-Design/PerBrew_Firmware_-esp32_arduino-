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
    constexpr int CLICK_PER_ADC = 4;

    //pulse timing
    constexpr int PULSE_DELAY_US = 500;
    constexpr int FAST_PULSE_DELAY_US = 200;
    constexpr int SLOW_PULSE_DELAY_US = 1000;
    constexpr int PULSE_US = 3;
}


GrinderDriver::GrinderDriver():stepper_(AccelStepper::DRIVER, ClickStepPin, ClickDirPin) {}

void GrinderDriver::begin() {

    #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12); // 0..4095
    analogSetPinAttenuation(ClickPotADC, ADC_11db);
    #endif

    
    pinMode(ClickStepPin, OUTPUT);
    pinMode(ClickDirPin, OUTPUT);
    pinMode(ClickEndStopPin, INPUT_PULLUP);
    pinMode(ClickPotADC, INPUT);
    pinMode(GrindPWMPin, OUTPUT);
    pinMode(GrindMoterADC, INPUT);

    digitalWrite(ClickStepPin, LOW);
    digitalWrite(ClickDirPin,  LOW);

    ;
    stepper_.setMaxSpeed(kMaxSpeed_);
    stepper_.setAcceleration(kAcceleration_);
    stepper_.setCurrentPosition(0); 

    
    setBaseCalibration();
    currentClicks_ = convertADCtoClicks();
}


// endstop으로 원점 측정 (이게 맞나...)
bool GrinderDriver::setBaseCalibration(uint32_t timeout_ms) {
    auto endstopTriggered = [&](){
        return digitalRead(ClickEndStopPin) == LOW; // PULLUP 가정
    };

    // 1) 고속 접근
    uint32_t t0 = millis();
    stepper_.setSpeed(kHomeSpeedFast_);   
    while (!endstopTriggered() && (millis() - t0) < timeout_ms) {
        stepper_.runSpeed();
    }
    if (!endstopTriggered()) return false;

    // 2) 백오프
    stepper_.setCurrentPosition(0);
    stepper_.moveTo(kBackoffSteps_);
    while (stepper_.distanceToGo() != 0) stepper_.run();

    // 3) 저속 재접근(정밀)
    uint32_t t1 = millis();
    stepper_.setSpeed(kHomeSpeedSlow_);
    while (!endstopTriggered() && (millis() - t1) < (timeout_ms / 2)) {
        stepper_.runSpeed();
    }
    if (!endstopTriggered()) return false;

    // 4) 원점 확정
    stepper_.setCurrentPosition(0);
    currentClicks_ = CLICK_MIN;
    return true;
}


// ADC 값을 클릭 수로 변환
int GrinderDriver:: convertADCtoClicks(){
    int adc = readADCAvg(ClickPotADC, 12);
    return adcToClicks(adc);
}

void GrinderDriver::setClicks(int target) {
    if(target < CLICK_MIN) target = CLICK_MIN;
    if(target > CLICK_MAX) target = CLICK_MAX;

    long targetSteps = lroundf(target * stepsPerClick_);
    stepper_.moveTo(targetSteps);
    while (stepper_.distanceToGo() != 0) stepper_.run();
    
    const int targetAdc = clicksToADC(target);
    const int ADC_DEADBAND = 3;
    uint32_t t0 = millis();
    while ((millis() - t0) < 800) {
        int nowAdc = readADCAvg(ClickPotADC, 12);
        int err = targetAdc - nowAdc;
        if (abs(err) <= ADC_DEADBAND) break;
        long tweak = (err > 0) ? 1 : -1;  
        stepper_.moveTo(stepper_.currentPosition() + tweak);
        while (stepper_.distanceToGo() != 0) stepper_.run();
    }

    currentClicks_ = target;
}


int GrinderDriver::readADCAvg(int pin, int samples){
    uint32_t acc = 0;
    for (int i=0;i<samples;i++) { acc += analogRead(pin); delayMicroseconds(50); }
    return (int)(acc / (uint32_t)samples);
}

int GrinderDriver::adcToClicks(int adc){
    if(ADC_MAX <= ADC_MIN ) return getCurrentClicks();
    float ratio = float(adc - ADC_MIN) / float(ADC_MAX - ADC_MIN);
    int clicks = int(lround(ratio * (CLICK_MAX - CLICK_MIN))) + CLICK_MIN;
    if (clicks < CLICK_MIN) clicks = CLICK_MIN;
    if (clicks > CLICK_MAX) clicks = CLICK_MAX;
    return clicks;
}

int GrinderDriver::clicksToADC(int clicks) {
    if(ADC_MAX <= ADC_MIN ) return getCurrentClicks();
    float ratio = float(clicks - CLICK_MIN) / float(CLICK_MAX - CLICK_MIN);
    int adc = int(lround(ADC_MIN + ratio * (ADC_MAX - ADC_MIN)));
    if (adc < ADC_MIN) adc = ADC_MIN;
    if (adc >= ADC_MAX) adc = ADC_MAX;
    return adc;
}

int GrinderDriver::getPulsePerClick() {
    // 최소 1펄스
    if (stepsPerClick_ < 0.5f) return 1; 
    return (int)lroundf(stepsPerClick_);
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