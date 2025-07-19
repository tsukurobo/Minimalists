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
constexpr int PUMP_PIN1 = 10;
constexpr int PUMP_PIN2 = 11;
constexpr int SOLENOID_PIN1 = 12;
constexpr int SOLENOID_PIN2 = 13;

// 手先のmotorに与えるduty比
constexpr int PUMP_PWM = 0.80;

// 手先のdynamixcelの角度定数 0~360°
constexpr float GRAB_ANGLE = 210.0f;
constexpr float RELEASE_ANGLE = 300.0f;
// 昇降用dynamixcelの角度定数　
constexpr float UP_ANGLE = 10.0f;
constexpr float DOWN_ANGLE = 150.0f;

// dynamixelのID
constexpr short DXL_ID1 = 0x01;  // 手先
constexpr short DXL_ID2 = 0x02;  // 昇降

extern mutex_t g_hand_mutex;
extern volatile bool g_hand_requested;
extern volatile bool g_has_work;
extern hand_state_t g_hand_state;
extern absolute_time_t g_hand_timer;

void hand_tick(hand_state_t hand_state, volatile bool has_work, volatile bool hand_requested, absolute_time_t hand_timer);

#endif  // HEADER_H