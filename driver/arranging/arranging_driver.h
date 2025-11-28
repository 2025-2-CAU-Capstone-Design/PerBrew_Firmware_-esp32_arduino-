#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include "../pin_map.h"

class ArrangingDriver {
public:
    ArrangingDriver();

    void begin();
    void setDirection(bool clockwise);  // 방향 설정 (true: 시계, false: 반시계)
    void move(long steps = 200);        // 현재 방향으로 이동
    void update();                      // 메인 루프에서 반복 호출
    void stop();

private:
    AccelStepper stepper_;
    bool directionCW_;                  // true = 시계 방향
    const int rearrangingMotorStepPin = 16;//pin::RearraingMotorStep;
    const int rearrangingMotorDirPin = 4;//pin::RearraingMotorDir;
};
