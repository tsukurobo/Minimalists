#ifndef HEADER_H
#define HEADER_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.hpp"
#include "dynamics.hpp"
#include "dynamixel.hpp"
#include "hand.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "trajectory.hpp"

// PIN設定
constexpr int PUMP_PIN = 10;
constexpr int PUMP_DIR_PIN = 11;
constexpr int SOLENOID_PIN = 12;

// 手先のmotorに与えるduty比
constexpr int PUMP_PWM = 128;

// 手先のdynamixcelの角度定数 0~360°
constexpr float GRAB_ANGLE = 10.0f;
constexpr float RELEASE_ANGLE = 90.0f;
// 昇降用dynamixcelの角度定数　
constexpr float UP_ANGLE = 10.0f;
constexpr float DOWN_ANGLE = 20.0f;

// dynamixelのID
constexpr short DXL_ID0 = 0x01;  // 昇降
constexpr short DXL_ID1 = 0x02;  // 手先

extern mutex_t g_hand_mutex;
extern volatile bool g_hand_requested;
extern volatile bool g_has_work;
extern hand_state_t g_hand_state;
extern absolute_time_t g_hand_timer;

void hand_tick(hand_state_t hand_state, volatile bool has_work, volatile bool hand_requested, absolute_time_t hand_timer);

#endif  // HEADER_H