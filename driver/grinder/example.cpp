#include <Arduino.h>
#include <math.h>
#include "./grinder_driver.h"

namespace {
  constexpr int CLICK_MIN = 0;
  constexpr int CLICK_MAX = 240;

  // 스텝 펄스 타이밍
  constexpr int PULSE_US = 3;
  constexpr int PULSE_DELAY_US = 500;

  // 이동 파라미터
  constexpr int  TOL_CLICKS     = 2;     // 허용 오차(클릭)
  constexpr int  NEAR_THRESH    = 10;    // 근접 판단
  constexpr int  BURST_BIG      = 20;    // 멀면 크게
  constexpr int  BURST_SMALL    = 5;     // 가까우면 작게
  constexpr int  MAX_PULSES_REFINE = 3000;
}

int GrinderDriver::readADCAvg_(int pin, int samples) {
  uint32_t acc = 0;
  for (int i=0;i<samples;i++) acc += analogRead(pin);
  return (int)(acc / (uint32_t)samples);
}

int GrinderDriver::clicksFromADC_(int adc) const {
  if (adcMax_ <= adcMin_) return currentClicks_;
  float ratio = float(adc - adcMin_) / float(adcMax_ - adcMin_);
  int clicks = int(lround(ratio * (CLICK_MAX - CLICK_MIN))) + CLICK_MIN;
  if (clicks < CLICK_MIN) clicks = CLICK_MIN;
  if (clicks > CLICK_MAX) clicks = CLICK_MAX;
  return clicks;
}

int GrinderDriver::adcFromClicks_(int clicks) const {
  clicks = constrain(clicks, CLICK_MIN, CLICK_MAX);
  float ratio = float(clicks - CLICK_MIN) / float(CLICK_MAX - CLICK_MIN);
  return int(lround(adcMin_ + ratio * (adcMax_ - adcMin_)));
}

void GrinderDriver::begin() {
    // 핀모드: 엔드스톱은 입력이어야 함 (이전 cpp에 OUTPUT으로 돼 있었음)
    pinMode(ClickStepPin, OUTPUT);
    pinMode(ClickDirPin,  OUTPUT);
    pinMode(ClickEndStopPin, INPUT_PULLUP);
    pinMode(ClickPotADC,   INPUT);

    digitalWrite(ClickStepPin, LOW);
    digitalWrite(ClickDirPin,  LOW);

    #if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12); // 0..4095
    analogSetPinAttenuation(ClickPotADC, ADC_11db);
    #endif

    // 홈+캘리브레이션을 실제로 실행
    homeAndCalibrate();
}

void GrinderDriver::stepOnce(int stepPin, int PULSE_US, int PULSE_DELAY_US) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY_US);
}

// 엔드스톱(LOW가 눌림 가정)으로 최소점 → 포텐셔미터 adcMin_ 취득
// 반대편으로 주행하며 adcMax_와 "클릭당 스텝수" 추정
bool GrinderDriver::homeAndCalibrate(uint32_t timeout_ms) {
    // 1) 엔드스톱 방향으로 이동
    digitalWrite(ClickDirPin, LOW); // 홈 방향(기구에 맞게 LOW/HIGH 선택)
    uint32_t t0 = millis();
    int pulses = 0;
    while (digitalRead(ClickEndStopPin) == HIGH && (millis()-t0) < timeout_ms) {
        stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US);
        pulses++;
    }
    if (digitalRead(ClickEndStopPin) == HIGH) return false; // 타임아웃

    // 2) 스위치 해제 위해 살짝 백오프
    digitalWrite(ClickDirPin, HIGH);
    for (int i=0;i<200;i++) stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US);
    delay(50);

    // 3) adcMin_ 샘플링 → 현재 클릭 = CLICK_MIN
    adcMin_ = readADCAvg_(ClickPotADC, 16);
    currentClicks_ = CLICK_MIN;

    // 4) 반대편으로 달리며 adcMax_와 스텝 스팬 측정
    digitalWrite(ClickDirPin, HIGH); // 최대쪽
    int last = adcMin_;
    int plateauCnt = 0;
    int stepsSpan = 0;
    t0 = millis();
    while ((millis()-t0) < timeout_ms) {
        stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US);
        stepsSpan++;
        int v = readADCAvg_(ClickPotADC, 2);
        if (v > last + 2) {
        plateauCnt = 0;
        last = v;
        } else {
        plateauCnt++;
        if (plateauCnt > 60) break; // 더 이상 증가 안 함 → 끝에 도달
        }
        // 안전 상한
        if (stepsSpan > 20000) break;
    }
    adcMax_ = last;
    if (adcMax_ <= adcMin_) return false;

    // 5) 클릭당 스텝수 계산
    stepsPerClick_ = float(stepsSpan) / float(CLICK_MAX - CLICK_MIN);
    if (stepsPerClick_ < 1.0f) stepsPerClick_ = 1.0f; // 방어

    // 6) 다시 최소로 복귀(원점 고정)
    digitalWrite(ClickDirPin, LOW);
    for (int i=0;i<stepsSpan;i++) stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US);
    currentClicks_ = CLICK_MIN;
    return true;
}

// 현재 ADC를 클릭으로 변환(공개 API)
int GrinderDriver::convertADCtoClicks() {
    int adc = readADCAvg_(ClickPotADC, 8);
    return clicksFromADC_(adc);
}

// 클릭 목표로 이동: 1) 오픈루프 근사 → 2) ADC 피드백으로 정밀 수렴
void GrinderDriver::setClicks(int target) {
    target = constrain(target, CLICK_MIN, CLICK_MAX);

    // 1) 현재 위치 추정
    int current = convertADCtoClicks();
    int diffClicks = target - current;

    // 2) 오픈루프 근사 이동(대략적인 스텝)
    int coarseSteps = int(lround(fabs(diffClicks) * stepsPerClick_ * 0.8f)); // 80%만 먼저
    digitalWrite(ClickDirPin, diffClicks > 0 ? HIGH : LOW);
    for (int i=0;i<coarseSteps; ++i) stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US);

    // 3) 폐루프 정밀 이동
    uint32_t t0 = millis();
    int pulses = 0, stableCnt = 0;
    while ((millis()-t0) < 4000 && pulses < MAX_PULSES_REFINE) {
        int cur = convertADCtoClicks();
        int d = target - cur;
        if (abs(d) <= TOL_CLICKS) {
        if (++stableCnt >= 3) break;
        } else {
        stableCnt = 0;
        digitalWrite(ClickDirPin, d > 0 ? HIGH : LOW);
        int burst = (abs(d) > NEAR_THRESH) ? BURST_BIG : BURST_SMALL;
        for (int i=0;i<burst; ++i) { stepOnce(ClickStepPin, PULSE_US, PULSE_DELAY_US); pulses++; }
        }
    }
    currentClicks_ = convertADCtoClicks();
}
