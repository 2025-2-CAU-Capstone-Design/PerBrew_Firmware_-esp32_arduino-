#include "arranging_driver.h"

namespace {
    constexpr float DEFAULT_SPEED = 1000.0f;       // 스텝/초
    constexpr float DEFAULT_ACCELERATION = 500.0f; // 스텝/초²
}

ArrangingDriver::ArrangingDriver()
    : stepper_(AccelStepper::DRIVER, pin::RearraingMotorStep, pin::RearraingMotorDir),
      directionCW_(true) {}

void ArrangingDriver::begin() {
    stepper_.setMaxSpeed(DEFAULT_SPEED);
    stepper_.setAcceleration(DEFAULT_ACCELERATION);
    stepper_.setCurrentPosition(0);

    pinMode(pin::RearraingMotorStep, OUTPUT);
    pinMode(pin::RearraingMotorDir, OUTPUT);

    Serial.println("ArrangingDriver initialized");
}

void ArrangingDriver::setDirection(bool clockwise) {
    directionCW_ = clockwise;
    Serial.print("Direction set to: ");
    Serial.println(clockwise ? "CW" : "CCW");
}

void ArrangingDriver::move(long steps) {
    long dirSteps = directionCW_ ? steps : -steps;
    stepper_.move(dirSteps);
    Serial.print("Moving ");
    Serial.println(dirSteps);
}

void ArrangingDriver::update() {
    stepper_.run();  // non-blocking 실행
}
