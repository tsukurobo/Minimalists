#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <ruckig/ruckig.hpp>

#include "amt223v.hpp"
#include "control_timing.hpp"
#include "debug_manager.hpp"
#include "disturbance_observer.hpp"
#include "dynamixel.hpp"
#include "hand.hpp"
#include "mcp25625.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "robomaster_motor.hpp"
#include "trajectory_sequence_manager.hpp"

constexpr float PI_F = 3.14159265358979323846f;

// 軌道データ点の構造体（制御用の詳細軌道）
typedef struct {
    float position_R;      // R軸目標位置 [rad]
    float velocity_R;      // R軸目標速度 [rad/s]
    float acceleration_R;  // R軸目標加速度 [rad/s^2]
    float position_P;      // P軸目標位置 [rad]
    float velocity_P;      // P軸目標速度 [rad/s]
    float acceleration_P;  // P軸目標加速度 [rad/s^2]
} trajectory_point_t;

// SPI設定構造体
typedef struct {
    spi_inst_t* spi_port;
    uint32_t baudrate;
    int pin_miso;
    int pin_cs[4];    // 最大4つのCSピンをサポート
    int num_cs_pins;  // 実際に使用するCSピンの数
    int pin_sck;
    int pin_mosi;
    int pin_rst;
} spi_config_t;

// ======== マイコン設定 ========
namespace MicrocontrollerConfig {
// システム設定定数
constexpr int SHUTDOWN_PIN = 27;  // 明示的にLOWにしないとPicoが動かない

// 制御周期定数
constexpr float CONTROL_PERIOD_MS = 0.3f;                        // 制御周期 [ms]
constexpr float CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0f;  // 制御周期 [s]

// Core間同期設定
constexpr int SYNC_EVERY_N_LOOPS = 200;             // 200ループごとにCore0に同期信号を送信
constexpr uint32_t SYNC_SIGNAL = 1;                 // 同期信号の値
constexpr uint32_t TRAJECTORY_DATA_SIGNAL = 2;      // 軌道データ送信信号
constexpr uint32_t TRAJECTORY_COMPLETE_SIGNAL = 3;  // 軌道完了信号
}  // namespace MicrocontrollerConfig

// ======== 機械設定 ========
namespace MechanismConfig {
// モータの出力軸から機構の出力軸までのギア比
constexpr float gear_ratio_R = 3.0f;     // M3508出力軸からベース根本(3.0)
constexpr float gear_ratio_P = 1.0f;     // M2006 P36出力軸からラックまで(ギアなし)
constexpr float gear_radius_P = 0.025f;  // ギアの半径 (m) - M2006の出力軸からラックまでの距離が25mm

// R軸（ベース回転）の動力学パラメータ（定数で表現）
constexpr float R_EQ_INERTIA = 0.3279f;                   // 等価慣性モーメント (kg·m^2)
constexpr float R_EQ_DAMPING = 0.4084f;                   // 等価粘性摩擦係数 (N·m·s/rad)
constexpr float R_TORQUE_CONSTANT = 0.3f * gear_ratio_R;  // 等価トルク定数（M3508のトルク定数xギア比） (Nm/A)

// P軸（アーム直動）の動力学パラメータ（定数で表現）
constexpr float P_EQ_INERTIA = 0.00448f;                   // 等価慣性モーメント (kg·m^2)
constexpr float P_EQ_DAMPING = 0.00785f;                   // 粘性摩擦係数 (N·m·s/rad)
constexpr float P_TORQUE_CONSTANT = 0.18f * gear_ratio_P;  // 等価トルク定数（M2006のトルク定数xギア比） (Nm/A)

constexpr float R_MAX_TORQUE = /*3.0f*/ 1.0f * gear_ratio_R;                // R軸最大トルク制限 [Nm] (M3508最大連続トルク 3.0Nm)
constexpr float P_MAX_TORQUE = 1.0f * gear_ratio_P;                         // P軸最大トルク制限 [Nm] (M2006最大連続トルク 1.0Nm)
constexpr float R_MAX_ACCELERATION = R_MAX_TORQUE / R_EQ_INERTIA;           // R軸最大角加速度 [rad/s^2]
constexpr float P_MAX_ACCELERATION = P_MAX_TORQUE / P_EQ_INERTIA;           // P軸最大角加速度 [rad/s^2]
constexpr float R_MAX_VELOCITY = 469.0 / 60.0 * 2.0 * PI_F / gear_ratio_R;  // R軸最大速度制限 [rad/s] Maximum speed at 3N•m: 469rpm
constexpr float P_MAX_VELOCITY = 416.0 / 60.0 * 2.0 * PI_F / gear_ratio_P;  // P軸最大速度制限 [rad/s] Maximum speed at 1N•m: 416 rpm

// モータとエンコーダの符号補正設定
constexpr float ENCODER_R_DIRECTION = 1.0f;  // R軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で右回り、右回りでエンコーダ値が増加
constexpr float ENCODER_P_DIRECTION = 1.0f;  // P軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で根本方向、根本方向でエンコーダ値が減少
}  // namespace MechanismConfig

// ======== 軌道生成設定 ========
// 軌道完了判定の許容誤差
namespace TrajectoryConfig {
constexpr float TRAJECTORY_COMPLETION_TOLERANCE_R = 0.1f;         // R軸完了判定許容誤差 [rad]
constexpr float TRAJECTORY_COMPLETION_TOLERANCE_P = 0.1f;         // P軸完了判定許容誤差 [rad]
constexpr float TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD = 0.1f;  // 完了判定時の速度閾値 [rad/s]

// Ruckig の設定用定数（Ruckigの引数指定がdoubleなのでdoubleで宣言）
namespace RuckigConfig {
// 軌道生成の最大速度
constexpr double R_MAX_VELOCITY = 0.15 * MechanismConfig::R_MAX_VELOCITY;
constexpr double P_MAX_VELOCITY = 0.7 * MechanismConfig::P_MAX_VELOCITY;

// 動き出しの加速は速く、止まるときの減速は遅く
constexpr double R_ACCEL = 0.95 * MechanismConfig::R_MAX_ACCELERATION;
constexpr double R_DECEL = 0.8 * MechanismConfig::R_MAX_ACCELERATION;
constexpr double P_ACCEL = 0.9 * MechanismConfig::P_MAX_ACCELERATION;
constexpr double P_DECEL = 0.8 * MechanismConfig::P_MAX_ACCELERATION;

// 最大ジャーク
constexpr double R_JERK = 100 * R_ACCEL;
constexpr double P_JERK = 100 * P_ACCEL;
}  // namespace RuckigConfig

// 軌道データ配列設定
constexpr u_int16_t MAX_TRAJECTORY_POINTS = 6000;  // 最大軌道点数
}  // namespace TrajectoryConfig
// 軌道データ管理構造体
typedef struct {
    trajectory_point_t points[TrajectoryConfig::MAX_TRAJECTORY_POINTS];
    u_int16_t point_count;
    u_int16_t current_index;
    bool active;
    bool complete;
    float final_target_R;   // 最終目標位置 R軸 [rad]
    float final_target_P;   // 最終目標位置 P軸 [rad]
    bool position_reached;  // 位置到達フラグ
} trajectory_data_t;

// ======== 制御設定 ========
namespace ControlConfig {
// PIDコントローラ（モータ1: 回転軸、モータ2: 直動軸）
// 位置PID制御器（位置[rad] → 目標速度[rad/s]）
constexpr float R_POSITION_KP = 1.25;  // R軸位置PIDの比例ゲイン
constexpr float R_VELOCITY_KP = 0.1;   // R軸速度I-Pの比例ゲイン
constexpr float R_VELOCITY_KI = 0.7;   // R軸速度I-Pの積分ゲイン
constexpr float P_POSITION_KP = 1.25;  // P軸位置PIDの比例ゲイン
constexpr float P_VELOCITY_KP = 0.1;   // P軸速度I-Pの比例ゲイン
constexpr float P_VELOCITY_KI = 1.0;   // P軸速度I-Pの積分ゲイン

// 速度推定のパラメータ
constexpr float R_VELOCITY_CUTOFF_FREQ = 50.0f;  // R軸 角速度のカットオフ周波数 [rad/s]
constexpr float P_VELOCITY_CUTOFF_FREQ = 50.0f;  // P軸 角速度のカットオフ周波数 [rad/s]

// 外乱オブザーバのパラメータ
constexpr float R_DOB_CUTOFF_FREQ = 6.0f;                                         // R軸 外乱オブザーバのカットオフ周波数 [rad/s]
constexpr float sqrtf_R_POSITION_GAIN = 7.0f;                                     // R軸 外乱オブザーバの位置ゲインの平方根
constexpr float R_POSITION_GAIN = sqrtf_R_POSITION_GAIN * sqrtf_R_POSITION_GAIN;  // R軸 外乱オブザーバの位置ゲイン
constexpr float R_VELOCITY_GAIN = 2.0f * sqrtf_R_POSITION_GAIN;                   // R軸 外乱オブザーバの速度ゲイン
constexpr float P_DOB_CUTOFF_FREQ = 4.0f;                                         // P軸 外乱オブザーバのカットオフ周波数 [rad/s]
constexpr float sqrtf_P_POSITION_GAIN = 7.0f;                                     // P軸 外乱オブザーバの位置ゲインの平方根
constexpr float P_POSITION_GAIN = sqrtf_P_POSITION_GAIN * sqrtf_P_POSITION_GAIN;  // P軸 外乱オブザーバの位置ゲイン
constexpr float P_VELOCITY_GAIN = 2.0f * sqrtf_P_POSITION_GAIN;                   // P軸 外乱オブザーバの速度ゲイン
}  // namespace ControlConfig

// ======== Dynamixelの設定 ========
namespace DynamixelConfig {
// PIN設定
constexpr int PUMP_PIN = 4;
constexpr int SOLENOID_PIN = 3;

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
}  // namespace DynamixelConfig
