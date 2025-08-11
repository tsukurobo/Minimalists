#ifndef HEADER_H
#define HEADER_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "dynamixel.hpp"
#include "hand.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"

// PIN設定
constexpr int PUMP_PIN = 4;
constexpr int SOLENOID_PIN = 3;

// 手先のmotorに与えるduty比
constexpr int PUMP_PWM = 0.80;

// dynamixelの初期化角度　
constexpr float START_HAND_ANGLE = 90.0f;
constexpr int32_t START_UP_ANGLE = 0xFFFFE6B0;  // 上段

// 手先のdynamixcelの角度定数 0~360°
constexpr float GRAB_ANGLE = 88.51f;
constexpr float RELEASE_ANGLE = GRAB_ANGLE + 90.0f;  // 仮
// 昇降用dynamixcelの角度定数　
constexpr int32_t UP_ANGLE = 0xFFFFE6B0;  // 下段 -4100 上段 -6480
constexpr int32_t DOWN_ANGLE = 0x0D70;    // 3440

// dynamixelのID
constexpr short DXL_ID1 = 0x01;  // 手先
constexpr short DXL_ID2 = 0x02;  // 昇降

extern mutex_t g_hand_mutex;
extern bool g_hand_requested;
extern bool g_has_work;
extern hand_state_t g_hand_state;
extern absolute_time_t g_hand_timer;

void hand_tick(hand_state_t* hand_state, bool* has_work, bool* hand_requested, absolute_time_t* hand_timer);

#endif  // HEADER_H