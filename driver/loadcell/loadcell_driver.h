#pragma once
#include "../pin_map.h"
#include <Arduino.h>
#include <vector>
#include "HX711.h"


class LoadCellDriver {
    public:
        LoadCellDriver();
        bool begin(uint8_t gain = 128, uint8_t samplesForTare = 10);                     // HX711 초기화
        bool tare(uint8_t times = 10);     // 영점 조정
        bool calibrate(float knownMass, uint8_t times = 10);

        // 스케일/오프셋 직접 설정 및 조회 (HX711 내부에 적용)
        void setScale(float calibrationFactor) { scale_.set_scale(calibrationFactor); };
        float getScale() { return scale_.get_scale(); };
        void setOffset(long offset) {
            offset_ = offset;
            scale_.set_offset(offset_);
        };
        long getOffset() {return offset_;};


        // 게인 설정 (다음 변환 사이클 이후 반영)
        void setGain(uint8_t gain) {
            currentWeight_ = 0.0f;
            gain_ = gain;
            scale_.set_gain(gain_); 
        };
        uint8_t getGain() const { return gain_; }


        // 가중치 업데이트 (블로킹). times=0이면 defaultSamples_ 사용
        void updateWeightBlocking(uint8_t times = 0) { 
            if (times == 0) times = defaultSamples_;
            currentWeight_ = scale_.get_units(times); 
        };


        // 준비 신호를 timeout_ms 안에 기다렸다가 갱신(준비 못 하면 false)
        bool tryUpdateWeight(uint32_t timeout_ms, uint8_t times = 1);


        // 현재 계산된 무게(단위: setScale에 맞춘 단위)
        float getWeight() const { return currentWeight_; }
        bool isReady() { return scale_.is_ready(); }
        void powerDown() { scale_.power_down(); }
        void powerUp() { scale_.power_up(); }


        // 기본 샘플 수 설정/조회
        void setDefaultSamples(uint8_t n) { 
            if(n == 0) 
                defaultSamples_ = 1;
            else
                defaultSamples_ = n; 
        }
        uint8_t getDefaultSamples() const { return defaultSamples_; }

        
    private:
        // === HX711 관련 변수 ===
        HX711 scale_{};        
        const int DOUT = pin::WeighingDT;
        const int SCK = pin::WeightingSCK;

        // === 상태 변수 ===
        float currentWeight_{0.0f};
        long offset_{0};
        uint8_t gain_{128};
        uint8_t defaultSamples_ = 10;
};