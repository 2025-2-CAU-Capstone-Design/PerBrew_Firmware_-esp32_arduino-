#include "pouringSection_driver.h"

PouringSectionDriver::PouringSectionDriver()
    : 
    //rotationStepper_(AccelStepper::DRIVER, pin::PouringRotationStep, pin::PouringRotationDir)
      tiltStepper_(AccelStepper::DRIVER, pin::PouringAngleStep, pin::PouringAngleDir) 
    {}

PouringSectionDriver::~PouringSectionDriver() {
    stopPump();
    stopRotation();
    stopTilt();
}

bool PouringSectionDriver::begin() {
    // 초음파 센서 핀 설정
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    digitalWrite(TrigPin, LOW);
    
    // 펌프 핀 설정
    pinMode(PumpPin, OUTPUT);
    analogWrite(PumpPin, 0);
    
    // 스텝 모터 설정(푸어링 -> 스탭모터 버전)
    /*
    rotationStepper_.setMaxSpeed(PouringConfig::DEFAULT_STEP_SPEED);
    rotationStepper_.setAcceleration(PouringConfig::DEFAULT_ACCELERATION);
    rotationStepper_.setCurrentPosition(0);
    */
   // 회전 모터 dc모터로 변경
    pinMode(PouringRotationPWM, OUTPUT);
    pinMode(PouringRotationDir, OUTPUT);
    analogWrite(PouringRotationPWM, 0);
    digitalWrite(PouringRotationDir, LOW);
    rotationRunning_ = false;


    // 스텝 모터 설정(틸트)
    tiltStepper_.setMaxSpeed(PouringConfig::DEFAULT_STEP_SPEED);
    tiltStepper_.setAcceleration(PouringConfig::DEFAULT_ACCELERATION);
    tiltStepper_.setCurrentPosition(0);
    // 초기화 완료 대기
    delay(1000);
    
    // 초기 거리 측정으로 센서 동작 확인
    long testDistance = measureDistanceCM();
    if (testDistance < 0) {
        setError(PouringError::ULTRASONIC_TIMEOUT);
        Serial.println("Warning: Ultrasonic sensor may not be working properly");
        return false;
    }
    Serial.println("PouringSectionDriver initialized successfully");
    return true;
}

long PouringSectionDriver::measureDistanceCM() {
    // 트리거 신호 생성
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    
    // 에코 신호 측정 (타임아웃: 30ms)
    duration_ = pulseIn(EchoPin, HIGH, PouringConfig::ULTRASONIC_TIMEOUT_MS * 1000);
    
    if (duration_ == 0) {
        setError(PouringError::ULTRASONIC_TIMEOUT);
        return -1;
    }
    
    // 거리 계산 (음속 기반, cm 단위)
    long distanceCM = duration_ * PouringConfig::SOUND_SPEED_CM_US / 2.0;
    
    // 유효 범위 검증
    if (!validateDistance(distanceCM)) {
        setError(PouringError::DISTANCE_OUT_OF_RANGE);
        return -1;
    }
    
    status_.currentDistance = distanceCM;
    clearError(); // 성공 시 에러 클리어
    return distanceCM;
}


// === 스텝 모터 제어 함수 ===
void PouringSectionDriver::setStepperSpeed(float speed) {
    tiltStepper_.setMaxSpeed(speed);
}

void PouringSectionDriver::setStepperAcceleration(float acceleration) {
    tiltStepper_.setAcceleration(acceleration);
}

bool PouringSectionDriver::isObjectInRange(int minCM, int maxCM) {
    long distance = measureDistanceCM();
    if (distance < 0) return false;
    return (distance >= minCM && distance <= maxCM);
}


//============================================================================================================================================
// === 노즐 회전 제어 ===
/*
void PouringSectionDriver::rotateNozzle(int steps, bool clockwise) {
    long targetPosition = rotationStepper_.currentPosition() + (clockwise ? steps : -steps);
    rotationStepper_.moveTo(targetPosition);
    status_.isRotating = true;
}

void PouringSectionDriver::rotateNozzleToAngle(float degrees) {
    // 1.8도 스텝 모터 기준 (200 스텝/회전)
    constexpr float STEPS_PER_DEGREE = 200.0f / 360.0f;
    long targetSteps = lround(degrees * STEPS_PER_DEGREE);
    rotationStepper_.moveTo(targetSteps);
    status_.isRotating = true;
}

void PouringSectionDriver::stopRotation() {
    rotationStepper_.stop();
    status_.isRotating = false;
}*/
void PouringSectionDriver::startRotation(bool clockwise, int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);
    digitalWrite(this->PouringRotationPWM, clockwise ? HIGH : LOW);
    analogWrite(this->PouringRotationDir, pwmValue);
    this->rotationRunning_ = true;
    status_.isRotating = rotationRunning_;

    Serial.print("[PouringSection]Rotation start: dir=");
    Serial.print(clockwise ? "CW" : "CCW");
    Serial.print(", PWM=");
    Serial.println(pwmValue);
}

void PouringSectionDriver::stopRotation() {
    analogWrite(this->PouringRotationDir    , 0);
    this->rotationRunning_ = false;
    status_.isRotating = false;
    Serial.println("Rotation stopped");
}


//============================================================================================================================================
// === 노즐 틸트 제어 ===
void PouringSectionDriver::tiltNozzle(long distanceCM) {
    if (distanceCM < PouringConfig::MIN_DISTANCE_CM || 
        distanceCM > PouringConfig::MAX_DISTANCE_CM) {
        setError(PouringError::DISTANCE_OUT_OF_RANGE);
        Serial.println("[PouringSection] Tilt distance out of range");
        return;
    }
    //값 보정 필요
    float mappedAngle = map(distanceCM, 5, 10, 0, 45);
    tiltNozzleToAngle(mappedAngle);
}

void PouringSectionDriver::tiltNozzleToAngle(float degrees) {
    // 1.8도 스텝 모터 기준
    constexpr float STEPS_PER_DEGREE = 200.0f / 360.0f;
    
    Serial.println("[PouringSection] Tilting to angle: " + String(degrees) + " degrees");
    long targetSteps = lround(degrees * STEPS_PER_DEGREE);

    tiltStepper_.moveTo(targetSteps);
    status_.isTilting = true;
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
        case PouringTechnique::SPIRAL_OUT:
            spiralOutPour(1.0f);
            break;
        case PouringTechnique::SPIRAL_IN:
            spiralInPour(1.0f);
            break;
        case PouringTechnique::CIRCULAR:
            circularPour(1.0f);
            break;
    }
}

void PouringSectionDriver::centerPour(float intensity) {
    stopRotation();
    tiltNozzleToAngle(0);  // 중심 위치
    Serial.println("[[PouringSection]] Center Pour");
}

void PouringSectionDriver::spiralOutPour(float intensity) {
    int rotationPWM = (int)(200 * intensity); 
    startRotation(true, rotationPWM);  
    Serial.println("[[PouringSection]] Spiral Out Pour");
}

void PouringSectionDriver::spiralInPour(float intensity) {
    int rotationPWM = (int)(200 * intensity);
    startRotation(false, rotationPWM);  // 반시계방향
    Serial.println("[[PouringSection]] Spiral In Pour");
}

void PouringSectionDriver::circularPour(float intensity) {
    int rotationPWM = (int)(180 * intensity);
    startRotation(true, rotationPWM);
    tiltNozzleToAngle(0);  // 틸트는 중심 고정
    Serial.println("[[PouringSection]] Circular Pour");
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

bool PouringSectionDriver::validateDistance(long distance) {
    return (distance >= PouringConfig::MIN_DISTANCE_CM && 
            distance <= PouringConfig::MAX_DISTANCE_CM);
}

void PouringSectionDriver::updateSteppers() {
    //rotationStepper_.run();
    tiltStepper_.run();
}

void PouringSectionDriver::update() {
    updateSteppers();    
    // 상태 업데이트
    //status_.isRotating = rotationStepper_.isRunning();
    status_.isTilting = tiltStepper_.isRunning();
}