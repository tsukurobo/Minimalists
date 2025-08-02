#pragma once

class disturbance_observer_t {
   public:
    disturbance_observer_t(float inertia, float cutoff_freq, float dt);
    void reset();
    float update(float control_torque, float motor_velocity);

   private:
    float inertia_;
    float cutoff_freq_;
    float dt_;

    float LPF_control_torque_;
    float dob_2_;
};