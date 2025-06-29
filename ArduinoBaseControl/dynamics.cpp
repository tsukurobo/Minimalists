#include "dynamics.hpp"

dynamics_t::dynamics_t(double inertia_mass, double viscous_friction_coeff, double torque_constant)
    : inertia_mass_(inertia_mass),
      viscous_friction_coeff_(viscous_friction_coeff),
      torque_constant_(torque_constant) {
}

double dynamics_t::calculate_feedforward_control(double target_vel, double target_accel) const {
    double ff_torque_force;  // フィードフォワードとして計算されるトルクまたは力

    // 慣性項
    ff_torque_force = inertia_mass_ * target_accel;

    // 粘性摩擦項
    ff_torque_force += viscous_friction_coeff_ * target_vel;

    return ff_torque_force;
}

double dynamics_t::convert_to_current_command(double feedforward_torque_force) const {
    return feedforward_torque_force / torque_constant_;
}