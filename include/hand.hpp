#ifndef hand
#define hand

#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

typedef enum {
    HAND_IDLE,
    HAND_LOWERING,
    HAND_SUCTION_WAIT,
    HAND_RAISING,
    HAND_RELEASE,
    HAND_WAITING,
} hand_state_t;

typedef enum {
    HAND_STANDBY,
    CATCHING_POSITION,
    HAND_DROPPING,
    CATCHING_WAIT,
    HAND_LIFTING,
    SHOOTING_POSITION,
    HAND_FOLD,
    HAND_FINISH,
    HAND_END
} QuickArm_state_t;

// ---- ポンプ制御 ----
void pump_init(uint pump_pin);
void pump_set_speed(uint pump_pin, float duty);  // PWM デューティ設定（0〜255)
void pump_set_direction_pin(int dir_pin);        // 方向ピン初期化＋正転出力

// ---- ソレノイド制御 ----
void solenoid_init(int solenoid_pin1, int solenoid_pin2);
void solenoid_input(int state, int solenoid_pin, int solenoid_pin2, uint slice_num);

#endif