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
    sequence_active = true;
    sequence_complete = false;
    wait_duration = 1.0f;  // デフォルト1秒待機
}

void TrajectorySequenceManager::setup_sequence(const trajectory_waypoint_t* waypoint_array, int count) {
    if (count > MAX_WAYPOINTS) {
        if (debug_manager) {
            debug_manager->error("Cannot setup sequence: waypoint count (%d) exceeds maximum (%d)", count, MAX_WAYPOINTS);
        }
        return;
    }

    initialize();

    // ウェイポイントを設定
    for (int i = 0; i < count; i++) {
        waypoints[i] = waypoint_array[i];
    }
    waypoint_count = count;

    if (debug_manager) {
        debug_manager->info("Trajectory sequence setup: %d waypoints", waypoint_count);
    }
}

bool TrajectorySequenceManager::get_next_waypoint(float& target_R, float& target_P) {
    if (!sequence_active || current_waypoint_index >= waypoint_count) {
        return false;
    }

    sequence_active = true;
    sequence_complete = false;

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
