#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <stdbool.h>  // bool型を使用するために必要

/**
 * 台形速度軌道生成クラス
 * 台形速度プロファイルを生成し、目標位置・速度・加速度を計算する
 */
class trajectory_t {
   private:
    double max_vel_;         // 最大速度 (単位: mm/s または rad/s)
    double max_accel_;       // 最大加速度 (単位: mm/s^2 または rad/s^2)
    double start_pos_;       // 開始位置 (単位: mm または rad)
    double end_pos_;         // 目標位置 (単位: mm または rad)
    double total_dist_;      // 総移動距離
    double accel_time_;      // 加速・減速にかかる時間
    double const_vel_time_;  // 定速期間の時間
    double total_time_;      // 移動にかかる総時間

   public:
    /**
     * コンストラクタ
     * @param max_vel 最大速度
     * @param max_accel 最大加速度
     * @param start_pos 開始位置
     * @param end_pos 目標位置
     */
    trajectory_t(double max_vel, double max_accel, double start_pos, double end_pos);

    /**
     * 台形速度プロファイルのパラメータを計算する関数
     */
    void calculate_trapezoidal_params();

    /**
     * 現在時刻 t における目標位置、目標速度、目標加速度を計算する関数
     * @param current_time 現在時刻 (秒単位)
     * @param target_pos 目標位置へのポインタ
     * @param target_vel 目標速度へのポインタ
     * @param target_accel 目標加速度へのポインタ
     */
    void get_trapezoidal_state(double current_time, double *target_pos, double *target_vel, double *target_accel) const;

    // ゲッター関数
    double get_max_vel() const { return max_vel_; }
    double get_max_accel() const { return max_accel_; }
    double get_start_pos() const { return start_pos_; }
    double get_end_pos() const { return end_pos_; }
    double get_total_dist() const { return total_dist_; }
    double get_accel_time() const { return accel_time_; }
    double get_const_vel_time() const { return const_vel_time_; }
    double get_total_time() const { return total_time_; }

    // セッター関数
    void set_max_vel(double max_vel) { max_vel_ = max_vel; }
    void set_max_accel(double max_accel) { max_accel_ = max_accel; }
    void set_start_pos(double start_pos) { start_pos_ = start_pos; }
    void set_end_pos(double end_pos) { end_pos_ = end_pos; }
};

#endif