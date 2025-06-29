#ifndef hand
#define hand

#include "dynamics.hpp"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
typedef enum {
    HAND_IDLE,
    HAND_LOWERING,
    HAND_SUCTION_WAIT,
    HAND_RAISING,
    HAND_RELEASE
} hand_state_t;

// ---- ポンプ制御 ----
void pump_init(int pump_pin);                     // PWM 初期化
void pump_set_speed(int pump_pin, uint8_t duty);  // PWM デューティ設定（0〜255）
void pump_set_direction_pin(int dir_pin);         // 方向ピン初期化＋正転出力

// ---- ソレノイド制御 ----
void solenoid_init(int solenoid_pin);
void solenoid_input(int state, int solenoid_pin);

#endif