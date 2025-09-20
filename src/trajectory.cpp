#include "trajectory.hpp"

#include <math.h>
#include <stdio.h>

trajectory_t::trajectory_t(float max_vel, float max_accel, float max_decel, float s_curve_ratio, float start_pos, float end_pos, float threshold_dist)
    : max_vel_{max_vel},
      max_accel_{max_accel},
      max_decel_{max_decel},
      s_curve_ratio_{s_curve_ratio},
      start_pos_{start_pos},
      end_pos_{end_pos},
      threshold_dist_{threshold_dist},
      total_dist_{0.0},
      accel_time_{0.0},
      const_vel_time_{0.0},
      total_time_{0.0} {
}

void trajectory_t::calculate_s_curve_trajectory_params() {
    // R軸の計算 (インデックス0)
    float s_curve_ratio_sq = s_curve_ratio_ * s_curve_ratio_;  // s_curve_ratioの二乗
    total_dist_ = fabsf(end_pos_ - start_pos_);
    if (total_dist_ < threshold_dist_) {
        // 移動距離が非常に小さい場合、すべての時間を0に設定
        accel_time_ = 0.0f;
        decel_time_ = 0.0f;
        const_vel_time_ = 0.0f;
        s_curve_time_ = 0.0f;
        total_time_ = 0.0f;
        return;
    } else {
        accel_time_ = max_vel_ / max_accel_;
        decel_time_ = (max_vel_ * (1 - s_curve_ratio_)) / max_decel_;
        s_curve_time_ = 2.0f * (max_vel_ * s_curve_ratio_) / max_decel_;                        // S字軌道の時間
        float accel_dist_0 = 0.5f * max_accel_ * accel_time_ * accel_time_;                     // 加速距離
        float decel_dist_0 = 0.5f * max_vel_ * max_vel_ * (1 - s_curve_ratio_sq) / max_decel_;  // 減速距離
        float s_curve_dist_0 = max_decel_ * s_curve_time_ * s_curve_time_ / 6.0f;               // S字軌道の距離

        if (accel_dist_0 + decel_dist_0 + s_curve_dist_0 >= total_dist_) {
            printf("Warning: No constant velocity phase. Adjust max_vel or max_accel.\n");
            const_vel_time_ = 0.0f;
            // s_curve_ratioの二乗
            float inter_value = 0.5f / max_accel_ + 0.5f * (1 - s_curve_ratio_sq) / max_decel_ + (2.0f * s_curve_ratio_sq / 3.0f / max_decel_);  // 分母の計算
            max_vel_ = sqrtf(total_dist_ / inter_value);                                                                                         // 最大速度の再計算
            accel_time_ = max_vel_ / max_accel_;
            decel_time_ = max_vel_ * (1 - s_curve_ratio_) / max_decel_;
            s_curve_time_ = 2.0f * max_vel_ * s_curve_ratio_ / max_decel_;
            total_time_ = accel_time_ + decel_time_ + s_curve_time_;
        } else {
            const_vel_time_ = (total_dist_ - accel_dist_0 - decel_dist_0 - s_curve_dist_0) / max_vel_;
            total_time_ = accel_time_ + const_vel_time_ + decel_time_ + s_curve_time_;
        }
    }
}

void trajectory_t::get_s_curve_state(float current_time, float& target_pos, float& target_vel, float& target_accel) const {
    float direction = (end_pos_ > start_pos_) ? 1.0f : -1.0f;

    if (current_time <= accel_time_) {
        // 加速期間
        target_accel = direction * max_accel_;
        target_vel = direction * max_accel_ * current_time;
        target_pos = start_pos_ + direction * 0.5f * max_accel_ * current_time * current_time;
    } else if (current_time <= accel_time_ + const_vel_time_) {
        // 定速期間
        float t_const = current_time - accel_time_;
        target_accel = 0.0f;
        target_vel = direction * max_vel_;
        target_pos = start_pos_ + direction * (0.5f * max_accel_ * accel_time_ * accel_time_ + max_vel_ * t_const);
    } else if (current_time <= accel_time_ + const_vel_time_ + decel_time_) {
        // 減速期間
        float t_decel = current_time - (accel_time_ + const_vel_time_);
        target_accel = -direction * max_decel_;
        target_vel = direction * (max_vel_ - max_decel_ * t_decel);
        target_pos = start_pos_ + direction * (0.5f * max_accel_ * accel_time_ * accel_time_ + max_vel_ * const_vel_time_ + 0.5f * (max_vel_ + (max_vel_ - max_decel_ * t_decel)) * t_decel);
    } else if (current_time <= accel_time_ + const_vel_time_ + decel_time_ + s_curve_time_) {
        // S字軌道期間
        float t_s_curve = current_time - (accel_time_ + const_vel_time_ + decel_time_);
        target_accel = -direction * max_decel_ * (1 - (t_s_curve / s_curve_time_));
        target_vel = direction * 0.5f * (s_curve_time_ - t_s_curve) * (s_curve_time_ - t_s_curve) * max_decel_ / s_curve_time_;
        target_pos = end_pos_ - direction * (1.0f / 6.0f) * max_decel_ * (s_curve_time_ - t_s_curve) * (s_curve_time_ - t_s_curve) * (s_curve_time_ - t_s_curve) / s_curve_time_;
    } else {
        // 終了後は目標位置に固定
        target_accel = 0.0f;
        target_vel = 0.0f;
        target_pos = end_pos_;
    }
}
// ---S字プロファイルの実装ここまで---
