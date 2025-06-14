#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>  // bool型を使用するために必要
// 台形速度軌道生成に必要なパラメータ構造体
typedef struct {
    double max_vel;         // 最大速度 (単位: mm/s または rad/s)
    double max_accel;       // 最大加速度 (単位: mm/s^2 または rad/s^2)
    double start_pos;       // 開始位置 (単位: mm または rad)
    double end_pos;         // 目標位置 (単位: mm または rad)
    double total_dist;      // 総移動距離
    double accel_time;      // 加速・減速にかかる時間
    double const_vel_time;  // 定速期間の時間
    double total_time;      // 移動にかかる総時間
} TrajectoryParams;

/**
 * 台形速度プロファイルのパラメータを計算する関数
 * @param params TrajectoryParams 構造体へのポインタ
 */
void calculate_trapezoidal_params(TrajectoryParams *params);

/**
 * 現在時刻 t における目標位置、目標速度、目標加速度を計算する関数
 * @param params TrajectoryParams 構造体へのポインタ
 * @param current_time 現在時刻 (秒単位)
 * @param target_pos 目標位置へのポインタ
 * @param target_vel 目標速度へのポインタ
 * @param target_accel 目標加速度へのポインタ
 */
void get_trapezoidal_state(const TrajectoryParams *params, double current_time,
                           double *target_pos, double *target_vel, double *target_accel);

// S字プロファイル軌道生成に必要なパラメータ構造体(加速度がなめらか)
typedef struct {
    double max_vel;     // 最大速度
    double max_accel;   // 最大加速度
    double max_jerk;    // 最大ジャーク
    double start_pos;   // 開始位置
    double end_pos;     // 目標位置
    double total_dist;  // 総移動距離
    double t_j;         // ジャーク区間の時間
    double t_a;         // 加速区間の時間
    double t_v;         // 定速区間の時間
    double total_time;  // 総時間
    // S字プロファイルのケース分け用フラグ
    bool is_triangular_accel;  // 加速が三角形プロファイルになるか (最大加速度に達しない)
    bool is_triangular_vel;    // 速度が三角形プロファイルになるか (最大速度に達しない)
} SCurveTrajectoryParams;

/**
 * S字プロファイルのパラメータを計算する関数
 * @param params SCurveTrajectoryParams 構造体へのポインタ
 */
void calculate_scurve_params(SCurveTrajectoryParams *params);

/**
 * S字プロファイルの現在時刻 t における目標位置・速度・加速度を計算する関数(バグがありそう)
 * @param params SCurveTrajectoryParams 構造体へのポインタ
 * @param current_time 現在時刻 (秒単位)
 * @param target_pos 目標位置へのポインタ
 * @param target_vel 目標速度へのポインタ
 * @param target_accel 目標加速度へのポインタ
 * @return 目標ジャーク (double型)
 */
double get_scurve_state(const SCurveTrajectoryParams *params, double current_time,
                        double *target_pos, double *target_vel,
                        double *target_accel);

#endif