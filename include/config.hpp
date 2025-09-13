#ifndef HEADER_H
#define HEADER_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "dynamics.hpp"
#include "dynamixel.hpp"
#include "hand.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "trajectory.hpp"

// PIN設定
constexpr int PUMP_PIN_SUB = 11;
constexpr int SOLENOID_PIN_SUB = 13;

// 手先のmotorに与えるduty比
constexpr int PUMP_PWM = 0.80;

// dynamixelの初期化角度　
constexpr int32_t START_HAND_ANGLE = 2977;
constexpr int32_t START_UP_ANGLE = -802;

// 手先のdynamixcelの角度定数 0~360°
constexpr int32_t CATCH_ANGLE = 3777;
constexpr int32_t SHOOTING_ANGLE = 2590;
constexpr int32_t INTER_POINT = 3900;
constexpr int32_t FOLD_ANGLE = 4751;
// 昇降用dynamixcelの角度定数　
constexpr int32_t UPPER_ANGLE = -802;
constexpr int32_t LOWER_ANGLE = 4674;

// dynamixelのID
constexpr short DXL_ID5 = 0x05;  // 根元
constexpr short DXL_ID6 = 0x06;  // 昇降

extern mutex_t g_hand_mutex;
extern bool g_hand_requested;
extern bool g_has_work;
extern QuickArm_state_t g_quickarm_state;
extern absolute_time_t g_hand_timer;

void exe_QuickArm(hand_state_t* hand_state, bool* has_work, bool* hand_requested, absolute_time_t* hand_timer);

#endif  // HEADER_H