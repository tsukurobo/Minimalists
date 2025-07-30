/**
 * @file trajectory_sequence_manager.cpp
 * @brief 軌道シーケンス管理クラスの実装
 */

#include "trajectory_sequence_manager.hpp"

#include <cmath>

TrajectorySequenceManager::TrajectorySequenceManager(DebugManager* debug_mgr)
    : waypoint_count(0), current_waypoint_index(0), sequence_active(false), sequence_complete(false), wait_duration(1.0f), debug_manager(debug_mgr) {
}

void TrajectorySequenceManager::initialize() {
    waypoint_count = 0;
    current_waypoint_index = 0;
    sequence_active = false;
    sequence_complete = false;
    wait_duration = 1.0f;  // デフォルト1秒待機
}

bool TrajectorySequenceManager::add_waypoint(float pos_R, float pos_P, float end_effector_angle) {
    if (waypoint_count >= MAX_WAYPOINTS) {
        if (debug_manager) {
            debug_manager->error("Cannot add waypoint: maximum capacity (%d) reached", MAX_WAYPOINTS);
        }
        return false;
    }

    waypoints[waypoint_count] = trajectory_waypoint_t(pos_R, pos_P, end_effector_angle);
    waypoint_count++;

    return true;
}

void TrajectorySequenceManager::setup_sequence(const trajectory_waypoint_t* waypoint_array, int count) {
    initialize();

    // テスト用の軌道点を追加（3次元：R軸角度[rad]、P軸距離[mm→rad変換]、手先角度[rad]）
    // P軸の距離[mm]をrad単位に変換: distance_mm / 1000.0 / gear_radius_P

    // ウェイポイント1: 3cm前進
    add_waypoint(0.0f, (0.03f / gear_radius_P), 0.0f);

    // ウェイポイント2: 45度回転（3cm位置を維持）
    add_waypoint(M_PI / 4.0f, (0.03f / gear_radius_P), 0.0f);

    // ウェイポイント3: さらに2cm前進（計5cm、45度回転状態を維持）
    add_waypoint(M_PI / 4.0f, (0.05f / gear_radius_P), 0.0f);

    // ウェイポイント4: 0度に戻る（5cm位置を維持）
    add_waypoint(0.0f, (0.05f / gear_radius_P), 0.0f);

    // ウェイポイント5: 原点に戻る
    add_waypoint(0.0f, 0.0f, 0.0f);

    if (debug_manager) {
        debug_manager->info("Test trajectory sequence setup: %d waypoints", waypoint_count);
        debug_manager->info("  Waypoint sequence: 3cm forward → 45° rotate → 5cm forward → 0° rotate → origin");
    }
}

void TrajectorySequenceManager::start_sequence() {
    current_waypoint_index = 0;
    sequence_active = true;
    sequence_complete = false;

    if (debug_manager) {
        debug_manager->info("Started trajectory sequence with %d waypoints", waypoint_count);
    }
}

bool TrajectorySequenceManager::get_next_waypoint(float& target_R, float& target_P) {
    if (!sequence_active || current_waypoint_index >= waypoint_count) {
        return false;
    }

    int index = current_waypoint_index;
    target_R = waypoints[index].position_R;
    target_P = waypoints[index].position_P;

    return true;
}

void TrajectorySequenceManager::advance_to_next_waypoint() {
    current_waypoint_index++;

    if (current_waypoint_index >= waypoint_count) {
        sequence_active = false;
        sequence_complete = true;
        if (debug_manager) {
            debug_manager->info("Trajectory sequence completed");
        }
    } else {
        if (debug_manager) {
            debug_manager->info("Advanced to waypoint %d/%d",
                                current_waypoint_index + 1, waypoint_count);
        }
    }
}

bool TrajectorySequenceManager::is_sequence_active() {
    return sequence_active;
}

bool TrajectorySequenceManager::is_sequence_complete() {
    return sequence_complete;
}

int TrajectorySequenceManager::get_current_waypoint_index() {
    return current_waypoint_index;
}

int TrajectorySequenceManager::get_waypoint_count() {
    return waypoint_count;
}

void TrajectorySequenceManager::set_wait_duration(float duration) {
    wait_duration = duration;
}
