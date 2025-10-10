#include "./loadcell_driver.h"

LoadCellDriver::LoadCellDriver() {
}

bool LoadCellDriver::begin(uint8_t gain, uint8_t samplesForTare) {
    gain_ = gain;
    scale_.begin(DOUT, SCK, gain_);
    scale_.set_scale(1.0f);
    scale_.set_offset(0);
    if(samplesForTare == 0)
        defaultSamples_ = 1;
    else 
        defaultSamples_ = samplesForTare;
    
    return tare(defaultSamples_);
}

bool LoadCellDriver::tare(uint8_t times) {
    if (!scale_.is_ready()) {
        Serial.println("[LoadCell] Not ready for tare");
        return false;
    }
    
    if (times == 0) times = defaultSamples_;
    
    scale_.tare(times);
    offset_ = scale_.get_offset();
    currentWeight_ = 0.0f;

    Serial.println("[LoadCell] Tare completed"); // 디버깅용 출력
    return true;
}

bool LoadCellDriver::calibrate(float knownMass, uint8_t times) {
    if(times == 0)
        times = defaultSamples_;
    float reading = scale_.read_average(times);
    float offset = scale_.get_offset();
    float diff = reading - offset;                // 영점 보정 값
    float calibratedValue = diff / knownMass;     // 보정된 스케일 값
    if(diff <= 0 || calibratedValue <= 0) {
        return false; // 보정 실패 (영점 이하)
    }
    scale_.set_scale(calibratedValue);
    currentWeight_ = scale_.get_units(times);
    return true;
}

bool LoadCellDriver::tryUpdateWeight(uint32_t timeout_ms, uint8_t times) {
    if (times == 0) times = 1;
    if (!scale_.wait_ready_timeout(timeout_ms)) return false;
    currentWeight_ = scale_.get_units(times); 
    return true;
}