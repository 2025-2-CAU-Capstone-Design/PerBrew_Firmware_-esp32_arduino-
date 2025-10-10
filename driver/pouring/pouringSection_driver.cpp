#include "pouringSection_driver.h"

pouringSectionDriver::pouringSectionDriver()
    : rotationStepper_(AccelStepper::DRIVER, pin::PouringRotationStep, pin::PouringRotationDir),
      tiltStepper_(AccelStepper::DRIVER, pin::PouringAngleStep, pin::PouringAngleDir) {
}

pouringSectionDriver::~pouringSectionDriver() {
    stopPump();
    stopRotation();
    stopTilt();
}

bool pouringSectionDriver::begin() {
    // 초음파 센서 핀 설정
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    digitalWrite(TrigPin, LOW);
    
    // 펌프 핀 설정
    pinMode(PumpPin, OUTPUT);
    analogWrite(PumpPin, 0);
    
    // 스텝 모터 설정
    rotationStepper_.setMaxSpeed(PouringConfig::DEFAULT_STEP_SPEED);
    rotationStepper_.setAcceleration(PouringConfig::DEFAULT_ACCELERATION);
    
    tiltStepper_.setMaxSpeed(PouringConfig::DEFAULT_STEP_SPEED);
    tiltStepper_.setAcceleration(PouringConfig::DEFAULT_ACCELERATION);
    
    // 초기화 완료 대기
    delay(100);
    
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

long pouringSectionDriver::measureDistanceCM() {
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
void pouringSectionDriver::setStepperSpeed(float speed) {
    rotationStepper_.setMaxSpeed(speed);
    tiltStepper_.setMaxSpeed(speed);
}

void pouringSectionDriver::setStepperAcceleration(float acceleration) {
    rotationStepper_.setAcceleration(acceleration);
    tiltStepper_.setAcceleration(acceleration);
}

bool pouringSectionDriver::isObjectInRange(int minCM, int maxCM) {
    long distance = measureDistanceCM();
    if (distance < 0) return false;
    return (distance >= minCM && distance <= maxCM);
}


//============================================================================================================================================
// === 노즐 회전 제어 ===

void pouringSectionDriver::rotateNozzle(int steps, bool clockwise) {
    long targetPosition = rotationStepper_.currentPosition() + (clockwise ? steps : -steps);
    rotationStepper_.moveTo(targetPosition);
    status_.isRotating = true;
}

void pouringSectionDriver::rotateNozzleToAngle(float degrees) {
    // 1.8도 스텝 모터 기준 (200 스텝/회전)
    constexpr float STEPS_PER_DEGREE = 200.0f / 360.0f;
    long targetSteps = lround(degrees * STEPS_PER_DEGREE);
    rotationStepper_.moveTo(targetSteps);
    status_.isRotating = true;
}

void pouringSectionDriver::stopRotation() {
    rotationStepper_.stop();
    status_.isRotating = false;
}

//============================================================================================================================================
// === 노즐 틸트 제어 ===
void pouringSectionDriver::tiltNozzle(long distanceCM) {
    if (distanceCM < PouringConfig::MIN_DISTANCE_CM || distanceCM > PouringConfig::MAX_DISTANCE_CM) {
        setError(PouringError::DISTANCE_OUT_OF_RANGE);
        return;
    }
    //값 보정 필요
    float mappedAngle = map(distanceCM, 2, 20, 0, 45);
    tiltNozzleToAngle(mappedAngle);
}

void pouringSectionDriver::tiltNozzleToAngle(float degrees) {
    // 1.8도 스텝 모터 기준
    constexpr float STEPS_PER_DEGREE = 200.0f / 360.0f;
    long targetSteps = lround(degrees * STEPS_PER_DEGREE);

    tiltStepper_.moveTo(targetSteps);
    status_.isTilting = true;
}

void pouringSectionDriver::stopTilt() {
    tiltStepper_.stop();
    status_.isTilting = false;
}


//============================================================================================================================================
// === 펌프 제어 ===

void pouringSectionDriver::startPump(int pwmValue) {
    pwmValue = constrain(pwmValue, PouringConfig::PUMP_MIN_PWM, PouringConfig::PUMP_MAX_PWM);
    analogWrite(PumpPin, pwmValue);
    status_.isPumping = (pwmValue > 0);
    status_.currentPumpPWM = pwmValue;
    
    Serial.print("Pump started with PWM: ");
    Serial.println(pwmValue);
}

void pouringSectionDriver::stopPump() {
    analogWrite(PumpPin, 0);
    status_.isPumping = false;
    status_.currentPumpPWM = 0;
    Serial.println("Pump stopped");
}

void pouringSectionDriver::setPumpSpeed(int pwmValue) {
    pwmValue = constrain(pwmValue, PouringConfig::PUMP_MIN_PWM, PouringConfig::PUMP_MAX_PWM);
    analogWrite(PumpPin, pwmValue);
    status_.currentPumpPWM = pwmValue;
    status_.isPumping = (pwmValue > 0);
}

void pouringSectionDriver::setPumpSpeedPercent(float percent) {
    percent = constrain(percent, 0.0, 100.0);
    int pwmValue = (int)(percent * PouringConfig::PUMP_MAX_PWM / 100.0);
    setPumpSpeed(pwmValue);
}



// ============================================================================================================================================
// === 내부 함수들 ===
void pouringSectionDriver::setError(PouringError error) {
    status_.lastError = error;
    if (error != PouringError::NONE) {
        Serial.print("PouringSection Error: ");
        Serial.println(getErrorString());
    }
}

bool pouringSectionDriver::validateDistance(long distance) {
    return (distance >= PouringConfig::MIN_DISTANCE_CM && 
            distance <= PouringConfig::MAX_DISTANCE_CM);
}

void pouringSectionDriver::updateSteppers() {
    rotationStepper_.run();
    tiltStepper_.run();
}

void pouringSectionDriver::update() {
    updateSteppers();    
    // 상태 업데이트
    status_.isRotating = rotationStepper_.isRunning();
    status_.isTilting = tiltStepper_.isRunning();
}