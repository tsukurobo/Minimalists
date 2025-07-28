/**
 * @file debug_manager.cpp
 * @brief デバッグ管理クラスの実装
 */

#include "debug_manager.hpp"

#include <stdio.h>

#include <cmath>
#include <cstring>

DebugManager::DebugManager(DebugLevel level, float status_interval)
    : current_level(level), status_output_interval(status_interval), last_status_output_time(0.0f), prev_trajectory_active_R(false), prev_trajectory_active_P(false), limits_displayed(false), trajectory_test_enabled(true), time_counter(0.0f), forward_direction(true), initial_pos_R(0.0f), initial_pos_P(0.0f), initial_pos_set(false) {
}

void DebugManager::log(DebugLevel level, const char* format, ...) {
    if (level > current_level) return;

    va_list args;
    va_start(args, format);
    print_with_level_prefix(level, format, args);
    va_end(args);
}

void DebugManager::error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    print_with_level_prefix(DebugLevel::ERROR, format, args);
    va_end(args);
}

void DebugManager::warn(const char* format, ...) {
    va_list args;
    va_start(args, format);
    print_with_level_prefix(DebugLevel::WARN, format, args);
    va_end(args);
}

void DebugManager::info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    print_with_level_prefix(DebugLevel::INFO, format, args);
    va_end(args);
}

void DebugManager::debug(const char* format, ...) {
    va_list args;
    va_start(args, format);
    print_with_level_prefix(DebugLevel::DEBUG, format, args);
    va_end(args);
}

void DebugManager::check_trajectory_state_changes(bool traj_active_R, bool traj_active_P,
                                                  float current_pos_R, float current_pos_P,
                                                  float final_target_pos_R, float final_target_pos_P,
                                                  float gear_radius_P) {
    // 軌道開始検出
    if (!prev_trajectory_active_R && traj_active_R) {
        info("R-axis trajectory STARTED: %.3f → %.3f rad (%.1f° → %.1f°)",
             current_pos_R, final_target_pos_R,
             current_pos_R * 180.0 / M_PI, final_target_pos_R * 180.0 / M_PI);
    }

    if (!prev_trajectory_active_P && traj_active_P) {
        info("P-axis trajectory STARTED: %.3f → %.3f rad (%.1f → %.1f mm)",
             current_pos_P, final_target_pos_P,
             current_pos_P * gear_radius_P * 1000.0, final_target_pos_P * gear_radius_P * 1000.0);

        debug("DEBUG: P-axis trajectory params - dist=%.3f rad (%.1f mm)",
              final_target_pos_P - current_pos_P,
              (final_target_pos_P - current_pos_P) * gear_radius_P * 1000.0);
    }

    // 軌道完了検出
    if (prev_trajectory_active_R && !traj_active_R) {
        info("R-axis trajectory COMPLETED");
    }

    if (prev_trajectory_active_P && !traj_active_P) {
        info("P-axis trajectory COMPLETED");
    }

    // 前回状態を保存
    prev_trajectory_active_R = traj_active_R;
    prev_trajectory_active_P = traj_active_P;
}

void DebugManager::print_trajectory_limits(float max_vel_R, float max_accel_R,
                                           float max_vel_P, float max_accel_P,
                                           float gear_radius_P) {
    if (!limits_displayed) {
        info("=== Trajectory Limits ===");
        info("R_MAX_VELOCITY: %.3f rad/s (%.1f °/s)", max_vel_R, max_vel_R * 180.0 / M_PI);
        info("R_MAX_ACCELERATION: %.3f rad/s^2 (%.1f °/s^2)", max_accel_R, max_accel_R * 180.0 / M_PI);
        info("P_MAX_VELOCITY: %.3f rad/s (%.1f mm/s)", max_vel_P, max_vel_P * gear_radius_P * 1000.0);
        info("P_MAX_ACCELERATION: %.3f rad/s^2 (%.1f mm/s^2)", max_accel_P, max_accel_P * gear_radius_P * 1000.0);
        limits_displayed = true;
    }
}

void DebugManager::print_trajectory_status(const TrajectoryDebugInfo& r_info,
                                           const TrajectoryDebugInfo& p_info) {
    info("\n=== Trapezoidal Profile Control Status ===");
    info("Trajectory Status: R=%s, P=%s",
         r_info.trajectory_active ? "ACTIVE" : "STOPPED",
         p_info.trajectory_active ? "ACTIVE" : "STOPPED");

    // R軸情報の表示
    info("Final Target:      R=%.3f [%s] (%.1f°), P=%.3f [%s] (%.1f mm)",
         r_info.final_target_pos, r_info.unit_name,
         r_info.final_target_pos * 180.0 / M_PI,
         p_info.final_target_pos, p_info.unit_name,
         p_info.final_target_pos * p_info.gear_radius * 1000.0);

    info("Trajectory Target: R=%.3f [%s] (%.1f°), P=%.3f [%s] (%.1f mm)",
         r_info.trajectory_target_pos, r_info.unit_name,
         r_info.trajectory_target_pos * 180.0 / M_PI,
         p_info.trajectory_target_pos, p_info.unit_name,
         p_info.trajectory_target_pos * p_info.gear_radius * 1000.0);

    info("Current Position:  R=%.3f [%s] (%.1f°), P=%.3f [%s] (%.1f mm)",
         r_info.current_pos, r_info.unit_name,
         r_info.current_pos * 180.0 / M_PI,
         p_info.current_pos, p_info.unit_name,
         p_info.current_pos * p_info.gear_radius * 1000.0);

    // 速度情報
    float traj_vel_mm = p_info.trajectory_target_vel * p_info.gear_radius * 1000.0;
    float current_vel_mm = p_info.current_vel * p_info.gear_radius * 1000.0;
    float final_vel_mm = p_info.final_target_vel * p_info.gear_radius * 1000.0;

    info("Target Velocity:   R=%.2f [rad/s], P=%.2f [rad/s] (%.1f mm/s)",
         r_info.trajectory_target_vel, p_info.trajectory_target_vel, traj_vel_mm);
    info("Current Velocity:  R=%.2f [rad/s], P=%.2f [rad/s] (%.1f mm/s)",
         r_info.current_vel, p_info.current_vel, current_vel_mm);
    info("Final Target Vel:  R=%.2f [rad/s], P=%.2f [rad/s] (%.1f mm/s)",
         r_info.final_target_vel, p_info.final_target_vel, final_vel_mm);
}

void DebugManager::print_system_status(const SystemDebugInfo& sys_info) {
    // エンコーダ情報
    info("P-axis multiturn: %d turns, single angle: %.1f°, [Valid: R=%s P=%s]",
         sys_info.encoder_p_turn_count, sys_info.encoder_p_single_angle_deg,
         sys_info.encoder_r_valid ? "OK" : "ERR",
         sys_info.encoder_p_valid ? "OK" : "ERR");

    // 制御出力情報
    info("Control Output: TorqR=%.2f CurR=%.2fA TorqP=%.2f CurP=%.2fA [LED:%s]",
         sys_info.target_torque_R, sys_info.target_current_R,
         sys_info.target_torque_P, sys_info.target_current_P,
         get_led_status_string(sys_info.led_status));

    info("Control Status: Violations:%d CAN_Errors:%d",
         sys_info.timing_violations, sys_info.can_errors);

    // エラー警告
    if (sys_info.can_errors > 0) {
        warn("WARNING: CAN transmission errors: %d", sys_info.can_errors);
    }
}

void DebugManager::check_abnormal_values(float traj_target_pos_P, float gear_radius_P) {
    if (std::abs(traj_target_pos_P) > 1000.0) {
        warn("WARNING: Abnormal P-axis trajectory target detected! Value=%.3f rad (%.1f mm)",
             traj_target_pos_P, traj_target_pos_P * gear_radius_P * 1000.0);
    }
}

void DebugManager::set_initial_positions(float pos_R, float pos_P) {
    if (!initial_pos_set) {
        initial_pos_R = pos_R;
        initial_pos_P = pos_P;
        initial_pos_set = true;
        const float gear_radius_P = 0.025;  // 25mm
        info("Set initial positions: R=%.3f rad, P=%.3f rad (%.1f mm)",
             initial_pos_R, initial_pos_P, initial_pos_P * gear_radius_P * 1000.0);
        info("NOTE: This position will be used as the reference (0mm) for all movements");
    }
}

bool DebugManager::should_start_trajectory_test(float current_time) {
    if (!trajectory_test_enabled) return false;

    // 初回テスト用の状態管理
    static bool first_test_done = false;
    static float last_test_time = 0.0f;

    // 初回は2秒後に実行
    if (!first_test_done && time_counter >= 2.0) {
        first_test_done = true;
        last_test_time = time_counter;
        return true;
    }

    // 2回目以降は10秒間隔で実行
    if (first_test_done && (time_counter - last_test_time >= 10.0)) {
        last_test_time = time_counter;
        return true;
    }

    return false;
}

bool DebugManager::should_set_initial_trajectory() {
    if (!trajectory_test_enabled) return false;

    // 0.5秒後に初期軌道を設定（システム安定化を待つ）
    static bool initial_trajectory_set = false;

    if (!initial_trajectory_set && time_counter >= 0.5) {
        initial_trajectory_set = true;
        return true;
    }

    return false;
}
void DebugManager::get_test_trajectory_targets(bool is_forward, float& target_R, float& target_P) {
    const float gear_radius_P = 0.025;  // 25mm

    if (is_forward) {
        // 前進方向の軌道（基準位置 → +550mm）
        target_R = initial_pos_R + 1.0 / 2.0 * M_PI;  // 90度回転

        float target_P_m = 0.55;  // 550mm
        float target_P_rad = target_P_m / gear_radius_P;
        target_P = initial_pos_P + target_P_rad;
    } else {
        // 後退方向の軌道（550mm → 基準位置）
        target_R = initial_pos_R;
        target_P = initial_pos_P;
    }
}

void DebugManager::print_trajectory_test_info(bool is_forward, float current_pos_P,
                                              float target_P, float gear_radius_P) {
    if (is_forward) {
        info("Started FORWARD trajectory at t=%.1fs:", time_counter);
        info("  Current position: P=%.3f rad (%.1f mm)",
             current_pos_P, current_pos_P * gear_radius_P * 1000.0);
        info("  Initial position: P=%.3f rad (%.1f mm)",
             initial_pos_P, initial_pos_P * gear_radius_P * 1000.0);
        info("  Target position:  P=%.3f rad (%.1f mm)",
             target_P, target_P * gear_radius_P * 1000.0);
        info("  Movement distance: %.1f mm", 0.55 * 1000.0);
    } else {
        info("Started BACKWARD trajectory at t=%.1fs:", time_counter);
        info("  Current position: P=%.3f rad (%.1f mm)",
             current_pos_P, current_pos_P * gear_radius_P * 1000.0);
        info("  Target position:  P=%.3f rad (%.1f mm)",
             initial_pos_P, initial_pos_P * gear_radius_P * 1000.0);
        info("  Movement distance: %.1f mm",
             (initial_pos_P - current_pos_P) * gear_radius_P * 1000.0);
    }
}

bool DebugManager::should_output_status(float current_time) {
    if (current_time - last_status_output_time >= status_output_interval) {
        last_status_output_time = current_time;
        return true;
    }
    return false;
}

void DebugManager::print_trajectory_progress(float current_pos_R, float current_pos_P,
                                             float trajectory_final_target_R, float trajectory_final_target_P,
                                             bool trajectory_position_reached,
                                             int trajectory_current_index, int trajectory_point_count,
                                             float gear_radius_P) {
    // 位置誤差計算
    float position_error_R = std::abs(trajectory_final_target_R - current_pos_R);
    float position_error_P = std::abs(trajectory_final_target_P - current_pos_P);

    // 軌道進行状況の出力
    if (trajectory_point_count > 0) {
        info("Trajectory progress: %d/%d points, Position errors: R=%.4f rad, P=%.1f μm",
             trajectory_current_index, trajectory_point_count,
             position_error_R, position_error_P * gear_radius_P * 1000000.0);

        if (trajectory_current_index >= trajectory_point_count) {
            info("Trajectory completion: Target R=%.3f rad, P=%.1f mm, Reached=%s",
                 trajectory_final_target_R,
                 trajectory_final_target_P * gear_radius_P * 1000.0,
                 trajectory_position_reached ? "Yes" : "No");
        }
    }
}

void DebugManager::print_with_level_prefix(DebugLevel level, const char* format, va_list args) {
    printf("[%s] ", get_level_string(level));
    vprintf(format, args);
    printf("\n");
}

const char* DebugManager::get_level_string(DebugLevel level) {
    switch (level) {
        case DebugLevel::ERROR:
            return "ERROR";
        case DebugLevel::WARN:
            return "WARN ";
        case DebugLevel::INFO:
            return "INFO ";
        case DebugLevel::DEBUG:
            return "DEBUG";
        case DebugLevel::TRACE:
            return "TRACE";
        default:
            return "UNKNW";
    }
}
