#include "disturbance_observer.hpp"

disturbance_observer_t::disturbance_observer_t(float inertia, float leftloop_cutoff_freq, float dob_cutoff_freq)
    : inertia_(inertia), leftloop_filter(leftloop_cutoff_freq), dob_filter(dob_cutoff_freq), dob_cutoff_freq_(dob_cutoff_freq), LPF_control_torque_(0.0f) {}

void disturbance_observer_t::reset() {
    LPF_control_torque_ = 0.0f;
    dob_rightloop_ = 0.0f;
    disturbance_ = 0.0f;
    leftloop_filter.reset();
    dob_filter.reset();
}

float disturbance_observer_t::update(float control_torque, float motor_velocity) {
    // Low-pass filter
    leftloop_filter.update(control_torque);
    LPF_control_torque_ = leftloop_filter.get_lpf_value();

    // Disturbance observer logic
    dob_rightloop_ = inertia_ * dob_cutoff_freq_ * motor_velocity;
    dob_filter.update(LPF_control_torque_ + dob_rightloop_);
    disturbance_ = dob_filter.get_lpf_value() - dob_rightloop_;

    return disturbance_;
}