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
constexpr int PUMP_PIN = 4;
constexpr int SOLENOID_PIN = 3;

// 手先のmotorに与えるduty比
constexpr int PUMP_PWM = 0.80;

// dynamixelの初期化角度　
constexpr float START_HAND_ANGLE = 132.0f;
constexpr int32_t START_UP_ANGLE = 0xFFFFFD7F;

// 手先のdynamixcelの角度定数 0~360°
constexpr float GRAB_ANGLE = 210.0f;
constexpr float RELEASE_ANGLE = 90.0f;
// 昇降用dynamixcelの角度定数　
constexpr int32_t UP_ANGLE = 0xFFFFFED4;  // -300
constexpr int32_t DOWN_ANGLE = 0x2454;    // 9300

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