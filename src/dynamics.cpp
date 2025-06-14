#include "dynamics.hpp"

double calculate_feedforward_control(const DynamicParams *dyn_params,
                                     double target_vel,
                                     double target_accel) {
    double ff_torque_force;  // フィードフォワードとして計算されるトルクまたは力

    // 慣性項
    ff_torque_force = dyn_params->inertia_mass * target_accel;

    // 粘性摩擦項
    ff_torque_force += dyn_params->viscous_friction_coeff * target_vel;

    return ff_torque_force;
}