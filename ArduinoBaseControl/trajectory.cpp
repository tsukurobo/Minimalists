#include "trajectory.hpp"

#include <math.h>
#include <stdio.h>

trajectory_t::trajectory_t(double max_vel, double max_accel, double start_pos, double end_pos)
    : max_vel_(max_vel),
      max_accel_(max_accel),
      start_pos_(start_pos),
      end_pos_(end_pos),
      total_dist_(0.0),
      accel_time_(0.0),
      const_vel_time_(0.0),
      total_time_(0.0) {
}

// ---台形プロファイルの実装---
void trajectory_t::calculate_trapezoidal_params() {
    total_dist_ = fabs(end_pos_ - start_pos_);
    accel_time_ = max_vel_ / max_accel_;
    double accel_dist = 0.5 * max_accel_ * accel_time_ * accel_time_;

    // 加速距離が総移動距離の半分以上の場合、定速期間は存在しない
    if (2 * accel_dist >= total_dist_) {
        printf("Warning: No constant velocity phase. Adjust max_vel or max_accel.\n");
        const_vel_time_ = 0.0;
        accel_time_ = sqrt(total_dist_ / max_accel_);
        total_time_ = 2 * accel_time_;
    } else {
        const_vel_time_ = (total_dist_ - 2 * accel_dist) / max_vel_;
        total_time_ = 2 * accel_time_ + const_vel_time_;
    }

    // printf("Calculated Trajectory Parameters:\n");
    // printf("  Total Distance: %.3f\n", total_dist_);
    // printf("  Accel/Decel Time: %.3f s\n", accel_time_);
    // printf("  Constant Velocity Time: %.3f s\n", const_vel_time_);
    // printf("  Total Time: %.3f s\n", total_time_);
}

void trajectory_t::get_trapezoidal_state(double current_time, double *target_pos, double *target_vel, double *target_accel) const {
    double direction = (end_pos_ > start_pos_) ? 1.0 : -1.0;

    // 現在時間が総移動時間より大きい場合は、目標位置で停止
    if (current_time >= total_time_) {
        *target_pos = end_pos_;
        *target_vel = 0.0;
        *target_accel = 0.0;
        return;
    }

    if (current_time <= accel_time_) {
        // 加速期間
        *target_accel = direction * max_accel_;
        *target_vel = direction * max_accel_ * current_time;
        *target_pos = start_pos_ + direction * 0.5 * max_accel_ * current_time * current_time;
    } else if (current_time <= accel_time_ + const_vel_time_) {
        // 定速期間
        double t_const = current_time - accel_time_;
        *target_accel = 0.0;
        *target_vel = direction * max_vel_;
        *target_pos = start_pos_ + direction * (0.5 * max_accel_ * accel_time_ * accel_time_ + max_vel_ * t_const);
    } else {
        // 減速期間
        double t_decel = current_time - (accel_time_ + const_vel_time_);
        *target_accel = -direction * max_accel_;
        *target_vel = direction * (max_vel_ - max_accel_ * t_decel);
        *target_pos = end_pos_ - direction * 0.5 * max_accel_ * (accel_time_ - t_decel) * (accel_time_ - t_decel);
    }
}
// ---台形プロファイルの実装ここまで---
