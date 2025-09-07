#pragma once

#include <stdbool.h>  // bool型を使用するために必要

/**
 * 台形速度軌道生成クラス
 * 台形速度プロファイルを生成し、目標位置・速度・加速度を計算する
 */
class trajectory_t {
   private:
    float max_vel_;         // 最大速度 (単位: mm/s または rad/s)
    float max_accel_;       // 最大加速度 (単位: mm/s^2 または rad/s^2)
    float max_decel_;       // 最大減速度 (単位: mm/s^2 または rad/s^2)
    float s_curve_ratio_;   // 減速区間におけるS字軌道の割合 (0.0 - 1.0) 速度ベース
    float s_curve_time_;    // S字軌道の時間 (単位: 秒)
    float start_pos_;       // 開始位置 (単位: mm または rad)
    float end_pos_;         // 目標位置 (単位: mm または rad)
    float total_dist_;      // 総移動距離
    float accel_time_;      // 加速・減速にかかる時間
    float decel_time_;      // 減速にかかる時間
    float const_vel_time_;  // 定速期間の時間
    float total_time_;      // 移動にかかる総時間

   public:
    /**
     * コンストラクタ
     * @param max_vel 最大速度
     * @param max_accel 最大加速度
     * @param max_decel 最大減速度
     * @param s_curve_ratio 減速区間におけるS字軌道の割合 (0.0 - 1.0) 速度ベース
     * @param start_pos 開始位置
     * @param end_pos 目標位置
     */
    trajectory_t(float max_vel, float max_accel, float max_decel, float s_curve_ratio, float start_pos, float end_pos);

    /**
     * R軸のS字軌道パラメータを計算する関数
     */
    void calculate_s_curve_trajectory_params();

    /**
     * 現在時刻 t における目標位置、目標速度、目標加速度を計算する関数
     * @param current_time 現在時刻 (秒単位)
     * @param target_pos 目標位置へのポインタ
     * @param target_vel 目標速度へのポインタ
     * @param target_accel 目標加速度へのポインタ
     */
    void get_s_curve_state(float current_time, float* target_pos, float* target_vel, float* target_accel) const;

    // ゲッター関数
    float get_max_vel() const { return max_vel_; }
    float get_max_accel() const { return max_accel_; }
    float get_start_pos() const { return start_pos_; }
    float get_end_pos() const { return end_pos_; }
    float get_total_dist() const { return total_dist_; }
    float get_accel_time() const { return accel_time_; }
    float get_const_vel_time() const { return const_vel_time_; }
    float get_total_time() const { return total_time_; }

    // セッター関数
    void set_max_vel(float max_vel) { max_vel_ = max_vel; }
    void set_max_accel(float max_accel) { max_accel_ = max_accel; }
    void set_start_pos(float start_pos) { start_pos_ = start_pos; }
    void set_end_pos(float end_pos) { end_pos_ = end_pos; }
};
