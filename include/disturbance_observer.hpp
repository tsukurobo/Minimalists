#pragma once

#include "low_pass_filter.hpp"

class disturbance_observer_t {
   public:
    disturbance_observer_t(float inertia, float velocity_cutoff_freq, float dob_cutoff_freq);
    void reset();
    float update(float control_torque, float motor_velocity);

   private:
    float inertia_;
    float dob_cutoff_freq_;

    float LPF_control_torque_;  // 制御トルクのローパスフィルタ出力
    float dob_rightloop_;       // 右ループの計算結果
    float disturbance_;         // 外乱トルク

    low_pass_filter_t leftloop_filter;  // 左ループのローパスフィルタ
    low_pass_filter_t dob_filter;       // 外乱オブザーバのローパスフィルタ
};