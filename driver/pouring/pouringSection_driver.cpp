#include "pouringSection_driver.h"

pouringSectionDriver::pouringSectionDriver(){}

void pouringSectionDriver::begin() {
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    pinMode(NozzleAngleDirPin, OUTPUT);
    pinMode(NozzleAngleStepPin, OUTPUT);
    pinMode(NozzleRotationDirPin, OUTPUT);
    pinMode(NozzleRotationStepPin, OUTPUT);

    digitalWrite(TrigPin, LOW);
    delay(50); 
}

long pouringSectionDriver::measureDistanceCM() {
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    duration_ = pulseIn(EchoPin, HIGH, 30000); // Timeout after 30ms (no object detected)
    long distanceMM = duration_ / 580; // Distance in mm
    if (duration_ == 0) {
        return -1;
    }
    return distanceMM;
}


//돌아가는지만 확인 (테스트용)
void pouringSectionDriver::rotateNozzleTest(int steps, bool direction) {
    digitalWrite(NozzleRotationDirPin, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(NozzleRotationStepPin, HIGH);
        delayMicroseconds(800); // Adjust delay for speed control
        digitalWrite(NozzleRotationStepPin, LOW);
        delayMicroseconds(800); // Adjust delay for speed control
    }
}


//돌아가는지만 확인
void pouringSectionDriver::tiltNozzleTest(int steps, bool direction) {
    digitalWrite(NozzleAngleDirPin, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        long distance = measureDistanceCM();
        digitalWrite(NozzleAngleStepPin, HIGH);
        delayMicroseconds(800); // Adjust delay for speed control
        digitalWrite(NozzleAngleStepPin, LOW);
        delayMicroseconds(800); // Adjust delay for speed control
    }
}

