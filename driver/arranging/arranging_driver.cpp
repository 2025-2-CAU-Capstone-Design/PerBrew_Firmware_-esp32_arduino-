#include "arranging_driver.h"

namespace {
    constexpr float DEFAULT_SPEED = 500.0f;       // 스텝/초
    constexpr float DEFAULT_ACCELERATION = 250.0f; // 스텝/초^2
}

ArrangingDriver::ArrangingDriver()
        // AccelStepper constructor for DRIVER mode expects (stepPin, dirPin)
        : stepper_(AccelStepper::DRIVER, 16, 4),
            directionCW_(true) {}

void ArrangingDriver::begin() {
    pinMode(16, OUTPUT);
    pinMode(4, OUTPUT);
    stepper_.setMaxSpeed(DEFAULT_SPEED);
    stepper_.setAcceleration(DEFAULT_ACCELERATION);
    stepper_.setMinPulseWidth(5);; 
    /*while(digitalRead(rearrangingMotorDirPin) == HIGH) {
        stepper_.runSpeed();
    }*/
    stepper_.setCurrentPosition(0);
    Serial.println("ArrangingDriver initialized");
}

void ArrangingDriver::setDirection(bool clockwise) {
    directionCW_ = clockwise;
    Serial.print("Direction set to: ");
    Serial.println(clockwise ? "CW" : "CCW");
}

void ArrangingDriver::move(long steps) {
    long dirSteps = directionCW_ ? steps : -steps;
    stepper_.setSpeed(directionCW_ ? DEFAULT_SPEED : -DEFAULT_SPEED);
    long startPos = stepper_.currentPosition();
    long targetPos = startPos + dirSteps;
    Serial.println("Current Position: " + String(startPos));
    Serial.println("Target Position: " + String(targetPos));
    while (stepper_.currentPosition() != targetPos) {
        stepper_.runSpeed();  // 내가 지정한 속도로 직접 회전
    }
    Serial.print("Moved ");
    Serial.println(dirSteps);
    directionCW_ = !directionCW_;
}

void ArrangingDriver::update() {
    stepper_.run();  // non-blocking 실행
}