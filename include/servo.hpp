#pragma once
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Servo {
   public:
    explicit Servo(uint pin);        // コンストラクタ
    void set_pulse_us(uint16_t us);  // パルス幅を直接指定（μs単位）
    void set_angle(float degree);    // 角度指定（0〜180°）

   private:
    uint pin_;
    uint slice_num_;
    uint channel_;
};