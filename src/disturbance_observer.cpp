#include "disturbance_observer.hpp"

disturbance_observer_t::disturbance_observer_t(float inertia, float cutoff_freq, float dt)
    : inertia_(inertia), cutoff_freq_(cutoff_freq), dt_(dt), LPF_control_torque_(0.0f), dob_2_(0.0f) {}

void disturbance_observer_t::reset() {
    LPF_control_torque_ = 0.0f;
    dob_2_ = 0.0f;
}

float disturbance_observer_t::update(float control_torque, float motor_velocity) {
    // Low-pass filter
    float dot_control_torque = (control_torque - LPF_control_torque_) * cutoff_freq_;
    LPF_control_torque_ += dot_control_torque * dt_;

    // Disturbance observer logic
    float dob_rightloop = inertia_ * cutoff_freq_ * motor_velocity;
    float dob_0 = LPF_control_torque_ + dob_rightloop;
    float dob_1 = (dob_0 - dob_2_) * cutoff_freq_;
    dob_2_ += dob_1 * dt_;
    float disturbance = dob_2_ - dob_rightloop;

    return disturbance;
}