#include "hand.hpp"

void pump_init(uint pump_pin) {
    gpio_set_function(pump_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pump_pin);
    pwm_set_wrap(slice_num, 149);                  // 周期
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);  // 初期出力0
    pwm_set_enabled(slice_num, true);
}

void pump_set_speed(uint pump_pin, float duty) {
    float pwm = duty * 150.0f;
    uint slice_num = pwm_gpio_to_slice_num(pump_pin);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm);
}

void pump_set_direction_pin(int dir_pin) {
    gpio_put(dir_pin, 0);  // LOW固定で正転（AIN2側）
}

void solenoid_init(int solenoid_pin1, int solenoid_pin2) {
    gpio_init(solenoid_pin1);
    gpio_init(solenoid_pin2);
    gpio_set_dir(solenoid_pin1, GPIO_OUT);
    gpio_set_dir(solenoid_pin2, GPIO_OUT);
    gpio_set_function(solenoid_pin1, GPIO_FUNC_PWM);
    gpio_put(solenoid_pin1, 1);  // 吸着状態で初期化
    gpio_put(solenoid_pin2, 0);  // 吸着状態で初期化
    sleep_ms(100);               // 初期化後の安定化待ち
}
void solenoid_input(int state, int solenoid_pin1, int solenoid_pin2, uint slice_num) {
    if (state == 0) {
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(solenoid_pin1), 0.0);  // ソレノイドのPWM出力
        gpio_put(solenoid_pin2, 0);                                              // 非吸着状態
    } else if (state == 1) {
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(solenoid_pin1), 149 * 0.9);  // ソレノイドのPWM出力
        gpio_put(solenoid_pin2, 0);                                                    // 吸着状態
    }
}