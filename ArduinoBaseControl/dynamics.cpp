#include "dynamics.hpp"

dynamics_t::dynamics_t(float inertia_mass, float viscous_friction_coeff, float torque_constant)
    : inertia_mass_(inertia_mass),
      viscous_friction_coeff_(viscous_friction_coeff),
      torque_constant_(torque_constant) {
}

float dynamics_t::calculate_feedforward_control(float target_vel, float target_accel) const {
    float ff_torque_force;  // フィードフォワードとして計算されるトルク

    // 慣性項
    ff_torque_force = inertia_mass_ * target_accel;

    // 粘性摩擦項
    ff_torque_force += viscous_friction_coeff_ * target_vel;

    return ff_torque_force;
}

float dynamics_t::convert_to_current_command(float feedforward_torque_force) const {
    return feedforward_torque_force / torque_constant_;
}