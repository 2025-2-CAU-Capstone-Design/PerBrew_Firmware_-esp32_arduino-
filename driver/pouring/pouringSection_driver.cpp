#include "pouringSection_driver.h"

PouringSectionDriver::PouringSectionDriver()
    : 
      tiltStepper_(AccelStepper::DRIVER, pin::PouringAngleStep, pin::PouringAngleDir) 
    {}

PouringSectionDriver::~PouringSectionDriver() {
    stopPump();
    stopRotation();
    stopTilt();
}

bool PouringSectionDriver::begin() {
    // 펌프 핀 설정
    pinMode(PumpPin, OUTPUT);
    analogWrite(PumpPin, 0);
    
   // 회전 모터 dc모터로 변경
    pinMode(PouringRotationPWM, OUTPUT);
    pinMode(PouringRotationDir, OUTPUT);
    analogWrite(PouringRotationPWM, 0);
    digitalWrite(PouringRotationDir, LOW);
    rotationRunning_ = false;


    // 스텝 모터 설정(틸트)
    pinMode(PouringTiltEndStopPin, INPUT_PULLUP);
    tiltStepper_.setMaxSpeed(PouringConfig::DEFAULT_STEP_SPEED);
    tiltStepper_.setAcceleration(PouringConfig::DEFAULT_ACCELERATION);
    tiltStepper_.setCurrentPosition(0);
    // 초기화 완료 대기
    delay(1000);
    

    Serial.println("PouringSectionDriver initialized successfully");
    return true;
}

// === 회전 모터 제어 함수 ===
void PouringSectionDriver::startRotation(bool clockwise, int pwmValue) {
    pwmValue = constrain(pwmValue, 50, 255);
    digitalWrite(PouringRotationDir, clockwise ? HIGH : LOW);
    analogWrite(PouringRotationPWM, pwmValue);
    rotationRunning_ = true;
    status_.isRotating = true;

    Serial.print("[PouringSection] Rotation: ");
    Serial.print(clockwise ? "CW" : "CCW");
    Serial.print(", PWM=");
    Serial.println(pwmValue);
}

void PouringSectionDriver::stopRotation() {
    analogWrite(PouringRotationPWM, 0);
    rotationRunning_ = false;
    status_.isRotating = false;
    Serial.println("[PouringSection] Rotation stopped");
}


// === 스텝 모터 제어 함수 ===
void PouringSectionDriver::setStepperSpeed(float speed) {
    tiltStepper_.setMaxSpeed(speed);
}

void PouringSectionDriver::setStepperAcceleration(float acceleration) {
    tiltStepper_.setAcceleration(acceleration);
}


void PouringSectionDriver::stopTilt() {
    tiltStepper_.stop();
    status_.isTilting = false;
}

void PouringSectionDriver::applyTechnique(PouringTechnique technique) {
    switch(technique) {
        case PouringTechnique::CENTER:
            centerPour(1.0f);
            break;
        case PouringTechnique::SPIRAL:
            spiralPour(1.0f);
            break;
        case PouringTechnique::CIRCULAR:
            circularPour(1.0f);
            break;
    }
}

void PouringSectionDriver::centerPour(float intensity) {
    stopRotation();
    tiltToZero();
    Serial.println("[PouringSection] CENTER Pour");
    status_.isTilting = false;
}

void PouringSectionDriver::spiralPour(float intensity) {
    int rotationPWM = (int)(200 * intensity);
    status_.isTilting = true;
    /* 틸트로직 구현 필요함...*/
    rotationPWM = constrain(rotationPWM, 50, 255);
    startRotation(true, rotationPWM);  
    Serial.println("[PouringSection] SPIRAL Pour");
}

void PouringSectionDriver::circularPour(float intensity) {
    int rotationPWM = (int)(180 * intensity);
    tiltStepper_.setSpeed(100);
    while(digitalRead(PouringTiltEndStopPin) == LOW) {
        tiltStepper_.runSpeed();
    }
    rotationPWM = constrain(rotationPWM, 50, 255);
    startRotation(true, rotationPWM);
    Serial.println("[PouringSection] CIRCULAR Pour");
    status_.isTilting = false;
}


// 노즐을 제로 위치로 이동 -> 첫 구동 때 영점 미리 잡아두기(손으로)
void PouringSectionDriver::tiltToZero() {
    tiltStepper_.setSpeed(100); // 일정 속도로 틸트
    tiltStepper_.moveTo(500); // 충분히 큰 값으로 이동 명령
    while (tiltStepper_.distanceToGo() != 0) {
        tiltStepper_.run();
    }
}


//============================================================================================================================================
// === 펌프 제어 ===

void PouringSectionDriver::startPump(int pwmValue) {
    pwmValue = constrain(pwmValue, PouringConfig::PUMP_MIN_PWM, PouringConfig::PUMP_MAX_PWM);
    analogWrite(PumpPin, pwmValue);
    status_.isPumping = (pwmValue > 0);
    status_.currentPumpPWM = pwmValue;
    
    Serial.print("[PouringSection] Pump started with PWM: ");
    Serial.println(pwmValue);
}

void PouringSectionDriver::stopPump() {
    analogWrite(PumpPin, 0);
    status_.isPumping = false;
    status_.currentPumpPWM = 0;
    Serial.println("[PouringSection] Pump stopped");
}

void PouringSectionDriver::setPumpSpeed(int pwmValue) {
    pwmValue = constrain(pwmValue, PouringConfig::PUMP_MIN_PWM, PouringConfig::PUMP_MAX_PWM);
    analogWrite(PumpPin, pwmValue);
    status_.currentPumpPWM = pwmValue;
    status_.isPumping = (pwmValue > 0);
}

void PouringSectionDriver::setPumpSpeedPercent(float percent) {
    percent = constrain(percent, 0.0, 100.0);
    int pwmValue = (int)(percent * PouringConfig::PUMP_MAX_PWM / 100.0);
    setPumpSpeed(pwmValue);
}



// ============================================================================================================================================
// === 내부 함수들 ===
void PouringSectionDriver::setError(PouringError error) {
    status_.lastError = error;
    if (error != PouringError::NONE) {
        Serial.print("[PouringSection] Error: ");
        Serial.println(static_cast<int>(error));
    }
}

/* 
   void PouringSectionDriver::updateSteppers() {
        tiltStepper_.run();
    }

    void PouringSectionDriver::update() {
        updateSteppers();    
        // 상태 업데이트
        //status_.isRotating = rotationStepper_.isRunning();
        status_.isTilting = tiltStepper_.isRunning();
    }
*/