#pragma once
#include <pin_map.h>
#include <Ultrasonic.h>

class pouringSectionDriver {
    public:
        pouringSectionDriver();
        void begin();
        long measureDistanceCM();
        void rotateNozzleTest(int steps, bool direction);
        void tiltNozzleTest(int steps, bool direction);
        int getDuration() const { return duration_; }
    private:
        const int TrigPin = pin::UltrasonicTrig;
        const int EchoPin = pin::UltrasonicEcho;
        const int NozzleAngleDirPin = pin::PouringAngleDir;
        const int NozzleAngleStepPin = pin::PouringAngleStep;
        const int NozzleRotationDirPin = pin::PouringRotationDir;
        const int NozzleRotationStepPin = pin::PouringRotationStep;
        int duration_ = 0;
};