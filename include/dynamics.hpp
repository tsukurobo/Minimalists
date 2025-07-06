#pragma once

// 各軸の動力学パラメータ構造体
typedef struct {
    double inertia_mass;            // 慣性モーメント (R軸) または質量 (P軸)
    double viscous_friction_coeff;  // 粘性摩擦係数
    double torque_constant;         // モーターのトルク定数 (Nm/A または Nmm/A)
                                    // P軸の場合、力に変換するための係数 (N/A または N/電流値単位)
} DynamicParams;

/**
 * 動力学計算に基づき、フィードフォワード制御量（トルク/力、または電流）を計算する関数。
 * 最終的な電流指令値への変換は呼び出し元で行う。
 * 実際にモーターに送る値は、この ff_torque_force をトルク定数(dyn_params->torque_constant)で割った電流値になります。
 * @param dyn_params 動力学パラメータへのポインタ
 * @param target_vel 目標速度 (単位: rad/s または m/s)
 * @param target_accel 目標加速度 (単位: rad/s^2 または m/s^2)
 * @return フィードフォワード制御量（トルク/力）
 */
double calculate_feedforward_control(
    const DynamicParams *dyn_params,
    double target_vel,
    double target_accel);
