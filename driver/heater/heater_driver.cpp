#include "./heater_driver.h"

namespace {
    static double PID_INPUT = 0.0, PID_OUTPUT = 0.0, PID_SETPOINT = 0.0;
    static double KP = 2.0, KI = 5.0, KD = 1.0;

    double emaFilter(double x, double& y, double z = 1.0){
        if (!isfinite(y)) { y = x; return y; }
        y = y * (1.0 - z) + (z * x);
        return y;
    }
}


heaterDriver::heaterDriver(){}

void heaterDriver::begin() {
    // PID 컨트롤러 초기화 (튜닝 필요)
    pidController = new PID(&PID_INPUT, &PID_OUTPUT, &PID_SETPOINT, KP, KI, KD, DIRECT);
    pidController->SetMode(AUTOMATIC);
    pidController->SetOutputLimits(0, 255); // PWM 출력 범위
    pidController->SetSampleTime(100);
    pinMode(TempSensorPin, INPUT);
    pinMode(HeaterPWMPin, OUTPUT);
    analogWrite(HeaterPWMPin, 0);
}


void heaterDriver::setTargetTemperature(double target) {
    targetTemperature = target;
    PID_SETPOINT = target;
}

// 온도 값 읽어오기
// TMP36 사용 가정, PWM 10bit 가정 (0-1023)
double heaterDriver::readThermistor(){
    int tempRaw = analogRead(TempSensorPin);
    if(tempRaw <= 0 || tempRaw >= 1023) return NAN; // 읽기 실패 (범위 밖)

    float voltage = (5.0 * tempRaw) / 1023.0; // 전압 따라 조정
    float temperatureC = (voltage - 0.5) / 0.01;      // TMP36 변환식
    return temperatureC;
}


void heaterDriver::updateValue() {
    static double filt = NAN;
    double c = readThermistor();
    if (!isnan(c)) currentTemperature = emaFilter(c, filt, 0.2);

    // 안전정치.... 온도가 너무 이상하면 꺼버리기
    if (isnan(c) || currentTemperature < 0 || currentTemperature > 110) {
        analogWrite(HeaterPWMPin, 0);
        return;
    }

    // PID에 입력
    PID_INPUT = currentTemperature;
    if (pidController->Compute()) {                                   // 주기 맞을 때만 계산 :contentReference[oaicite:6]{index=6}
        analogWrite(HeaterPWMPin, (int)PID_OUTPUT);
    }
}