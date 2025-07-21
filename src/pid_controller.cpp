#include "pid_controller.hpp"

#include <stdio.h>

#include <algorithm>
#include <cmath>

// ===== PIDController クラス実装 =====

PIDController::PIDController(double kp_val, double ki_val, double kd_val, double sample_time)
    : kp(kp_val), ki(ki_val), kd(kd_val), dt(sample_time), prev_error(0.0), integral(0.0), prev_input(0.0), output_min(-1e6), output_max(1e6), integral_min(-1e6), integral_max(1e6), use_derivative_on_input(true), first_run(true) {
}

void PIDController::setGains(double kp_val, double ki_val, double kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
}

void PIDController::setOutputLimits(double min_val, double max_val) {
    if (min_val > max_val) return;
    output_min = min_val;
    output_max = max_val;

    // 現在の積分値も制限内に収める
    integral = std::clamp(integral, integral_min, integral_max);
}

void PIDController::setIntegralLimits(double min_val, double max_val) {
    if (min_val > max_val) return;
    integral_min = min_val;
    integral_max = max_val;

    // 現在の積分値を制限内に収める
    integral = std::clamp(integral, integral_min, integral_max);
}

void PIDController::setDerivativeOnInput(bool enable) {
    use_derivative_on_input = enable;
}

void PIDController::setSampleTime(double sample_time) {
    if (sample_time > 0) {
        dt = sample_time;
    }
}

void PIDController::reset() {
    prev_error = 0.0;
    integral = 0.0;
    prev_input = 0.0;
    first_run = true;
}

double PIDController::compute(double setpoint, double input) {
    double error = setpoint - input;

    // 積分項の計算
    integral += error * dt;
    integral = std::clamp(integral, integral_min, integral_max);

    // 微分項の計算
    double derivative = 0.0;
    if (!first_run) {
        if (use_derivative_on_input) {
            // 微分先行型：目標値変化の影響を受けない
            derivative = -(input - prev_input) / dt;
        } else {
            // 通常微分：偏差の微分
            derivative = (error - prev_error) / dt;
        }
    }

    // PID出力計算
    double output = kp * error + ki * integral + kd * derivative;

    // 出力制限
    output = std::clamp(output, output_min, output_max);

    // 次回のために現在値を保存
    prev_error = error;
    prev_input = input;
    first_run = false;

    return output;
}

double PIDController::computeIP(double setpoint, double input) {
    double error = setpoint - input;

    // printf("IP Control: Setpoint=%.4f, Input=%.4f, Error=%.4f\n", setpoint, input, error);

    // 積分項の計算
    integral += error * dt;
    integral = std::clamp(integral, integral_min, integral_max);

    // I-P制御出力計算
    // I項は目標値、P項は偏差
    double output = ki * integral - kp * input;

    // 出力制限
    output = std::clamp(output, output_min, output_max);

    // 次回のために現在値を保存
    prev_error = error;
    prev_input = input;
    first_run = false;

    return output;
}

double PIDController::computePID(double setpoint, double input) {
    double error = setpoint - input;

    // 積分項の計算
    integral += error * dt;
    integral = std::clamp(integral, integral_min, integral_max);

    // 微分項の計算（入力に対する微分先行型）
    double derivative = 0.0;
    if (!first_run) {
        derivative = -(input - prev_input) / dt;
    }

    // PI-D出力計算
    double output = kp * error + ki * integral + kd * derivative;

    // 出力制限
    output = std::clamp(output, output_min, output_max);

    // 次回のために現在値を保存
    prev_error = error;
    prev_input = input;
    first_run = false;

    return output;
}

void PIDController::printDebugInfo(const char* prefix, double error, double output) const {
    printf("%s: Error=%.4f, Integral=%.4f, Output=%.4f (Kp=%.3f, Ki=%.3f, Kd=%.3f)\n",
           prefix, error, integral, output, kp, ki, kd);
}

// ===== VelocityIPController クラス実装 =====

VelocityIPController::VelocityIPController(double ki_val, double kp_val, double sample_time)
    : PIDController(kp_val, ki_val, 0.0, sample_time) {
    // 速度制御では微分項は使用しない
    setDerivativeOnInput(false);
}

double VelocityIPController::computeVelocity(double target_velocity, double current_velocity) {
    // I-P制御で速度制御
    return computeIP(target_velocity, current_velocity);
}

// ===== PositionPIDController クラス実装 =====

PositionPIDController::PositionPIDController(double kp_val, double ki_val, double kd_val, double sample_time)
    : PIDController(kp_val, ki_val, kd_val, sample_time) {
    // 位置制御では微分先行型を使用（目標値変化に対する微分キックを防ぐ）
    setDerivativeOnInput(true);
}

double PositionPIDController::computePosition(double target_position, double current_position) {
    // 標準PID制御で位置制御
    return computePID(target_position, current_position);
}
