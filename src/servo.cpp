#include "servo.hpp"

Servo::Servo(uint pin) : pin_(pin) {
    // ピンをPWM機能に設定
    gpio_set_function(pin_, GPIO_FUNC_PWM);

    // PWMスライスを取得
    slice_num_ = pwm_gpio_to_slice_num(pin_);
    channel_ = pwm_gpio_to_channel(pin_);

    // 20ms周期(50Hz)のPWMを設定
    pwm_set_wrap(slice_num_, 24999);                 // TOP値
    pwm_set_clkdiv(slice_num_, 100.0f);              // 分周設定
    pwm_set_chan_level(slice_num_, channel_, 1875);  // 初期値：1.5ms (90°)
    pwm_set_enabled(slice_num_, true);
}

void Servo::set_pulse_us(uint16_t us) {
    // 1カウント = 20ms / 25000 = 0.8us
    uint16_t level = static_cast<uint16_t>(us / 0.8f);
    pwm_set_chan_level(slice_num_, channel_, level);
}

void Servo::set_angle(float degree) {
    // サーボによって異なるが一般的に 1ms=0°, 2ms=180°
    const float min_us = 1000.0f;
    const float max_us = 2000.0f;
    float us = min_us + (degree / 180.0f) * (max_us - min_us);
    set_pulse_us(static_cast<uint16_t>(us));
}
