#include "hand.hpp"

void pump_init(int pump_pin) {
    gpio_set_function(pump_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pump_pin);
    pwm_set_wrap(slice_num, 255);
    pwm_set_clkdiv(slice_num, 4.0f);
    pwm_set_enabled(slice_num, true);
}

// ポンプ速度（デューティ比）設定
void pump_set_speed(int pwm_pin, uint8_t duty) {
    pwm_set_gpio_level(pwm_pin, duty);  // duty = 0〜255
}

void pump_set_direction_pin(int dir_pin) {
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 0);  // LOW固定で正転（AIN2側）
}

void solenoid_init(int solenoid_pin) {
    gpio_init(solenoid_pin);
    gpio_set_dir(solenoid_pin, GPIO_OUT);
    gpio_put(solenoid_pin, 0);  // 吸着状態で初期化
}

void solenoid_input(int state, int solenoid_pin) {
    gpio_put(solenoid_pin, state);
}