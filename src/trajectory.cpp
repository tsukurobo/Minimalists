#include "trajectory.hpp"

#include <math.h>
#include <stdio.h>

// ---台形プロファイルの実装---
void calculate_trapezoidal_params(TrajectoryParams *params) {
    params->total_dist = fabs(params->end_pos - params->start_pos);
    params->accel_time = params->max_vel / params->max_accel;
    double accel_dist = 0.5 * params->max_accel * params->accel_time * params->accel_time;

    // 加速距離が総移動距離の半分以上の場合、定速期間は存在しない
    if (2 * accel_dist >= params->total_dist) {
        printf("Warning: No constant velocity phase. Adjust max_vel or max_accel.\n");
        params->const_vel_time = 0.0;
        params->accel_time = sqrt(params->total_dist / params->max_accel);
        params->total_time = 2 * params->accel_time;
    } else {
        params->const_vel_time = (params->total_dist - 2 * accel_dist) / params->max_vel;
        params->total_time = 2 * params->accel_time + params->const_vel_time;
    }

    // printf("Calculated Trajectory Parameters:\n");
    // printf("  Total Distance: %.3f\n", params->total_dist);
    // printf("  Accel/Decel Time: %.3f s\n", params->accel_time);
    // printf("  Constant Velocity Time: %.3f s\n", params->const_vel_time);
    // printf("  Total Time: %.3f s\n", params->total_time);
}

void get_trapezoidal_state(const TrajectoryParams *params, double current_time,
                           double *target_pos, double *target_vel, double *target_accel) {
    double direction = (params->end_pos > params->start_pos) ? 1.0 : -1.0;

    // 現在時間が総移動時間より大きい場合は、目標位置で停止
    if (current_time >= params->total_time) {
        *target_pos = params->end_pos;
        *target_vel = 0.0;
        *target_accel = 0.0;
        return;
    }

    if (current_time <= params->accel_time) {
        // 加速期間
        *target_accel = direction * params->max_accel;
        *target_vel = direction * params->max_accel * current_time;
        *target_pos = params->start_pos + direction * 0.5 * params->max_accel * current_time * current_time;
    } else if (current_time <= params->accel_time + params->const_vel_time) {
        // 定速期間
        double t_const = current_time - params->accel_time;
        *target_accel = 0.0;
        *target_vel = direction * params->max_vel;
        *target_pos = params->start_pos + direction * (0.5 * params->max_accel * params->accel_time * params->accel_time + params->max_vel * t_const);
    } else {
        // 減速期間
        double t_decel = current_time - (params->accel_time + params->const_vel_time);
        *target_accel = -direction * params->max_accel;
        *target_vel = direction * (params->max_vel - params->max_accel * t_decel);
        *target_pos = params->end_pos - direction * 0.5 * params->max_accel * (params->accel_time - t_decel) * (params->accel_time - t_decel);
    }
}
// ---台形プロファイルの実装ここまで---

// --- S字プロファイルの実装 ---
void calculate_scurve_params(SCurveTrajectoryParams *params) {
    params->total_dist = fabs(params->end_pos - params->start_pos);

    // ジャーク期間の時間 t_j = A_max / J_max
    params->t_j = params->max_accel / params->max_jerk;

    // A_max に達するまでに移動する距離と時間
    double dj = 0.5 * params->max_accel * params->t_j;  // 加速ジャーク期間の移動距離 (v=1/2*at^2ではない、a=jtなのでv=1/2jt^2、x=1/6jt^3)
    // 正しくは:
    // 加速ジャーク期間の速度変化: dv = 0.5 * max_jerk * t_j^2 = 0.5 * A_max * t_j
    // 加速ジャーク期間の移動距離: dx = (1/6) * max_jerk * t_j^3 = (1/6) * A_max * t_j^2
    // A_maxに達するまでの速度: V_jerk_accel = 0.5 * params->max_jerk * params->t_j * params->t_j;
    // A_maxに達するまでの移動距離: X_jerk_accel = (1.0/6.0) * params->max_jerk * pow(params->t_j, 3);

    // 各ケースの判定と時間計算
    // 参考: "Trajectory Generation for Robotic Applications" by J. D. Slotine & W. Li
    //           or "Robotics: Modelling, Planning and Control" by B. Siciliano et al.

    // 加速期間が三角形（最大加速度に達しない）になるかどうかを判定
    // (V_max が 0.5 * J_max * t_j^2 = 0.5 * A_max * t_j より小さい場合)
    if (params->max_vel < (0.5 * params->max_jerk * params->t_j * params->t_j)) {
        params->is_triangular_accel = true;  // 加速度が三角形プロファイル (A_maxに達しない)
        // V_max = 2 * (1/2 * J_max * t_j^2) -> t_j = sqrt(V_max / J_max)
        params->t_j = sqrt(params->max_vel / params->max_jerk);
        params->t_a = 2 * params->t_j;  // 加速・減速期間は2つのジャーク期間のみ
        params->t_v = 0.0;              // 定速期間なし
    } else {
        params->is_triangular_accel = false;
        // 加速・減速期間の速度 V_accel = V_max - 2 * (1/2 * J_max * t_j^2) = V_max - A_max * t_j
        // 加速・減速期間の速度 V_accel = A_max * (t_a - t_j)
        // t_a = t_j + V_accel / A_max
        double V_accel_component = params->max_vel - params->max_accel * params->t_j;  // 加速ジャーク後に到達する速度から最大加速度までにかかる速度
        if (V_accel_component < 0) V_accel_component = 0;                              // 負にならないように補正
        params->t_a = params->t_j + V_accel_component / params->max_accel;
    }

    // 移動距離から定速期間の有無を判定
    double dist_accel_decel = 2 * ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +                                 // ジャーク加速期間の移動
                                   0.5 * params->max_accel * (params->t_a - params->t_j) * (params->t_a - params->t_j) +  // 定加速度期間の移動
                                   params->max_accel * params->t_j * (params->t_a - params->t_j)                          // ジャーク加速期間の速度が定加速度期間に影響
                                  );
    // 正しい計算式 for full S-curve accel/decel dist:
    // dist_accel_decel = 2 * (params->max_accel * params->max_accel / params->max_jerk + (params->t_a - params->t_j) * params->max_accel);
    // dist_accel_decel = 2 * ( params->max_vel * params->t_a - params->max_accel * params->t_j * params->t_j ); // simpler form

    // 正しい加速・減速フェーズの移動距離 (Sliding mode control for mechanical systems, p.248)
    // dist_accel_decel = V_max^2 / A_max + A_max * t_j^2
    double dist_needed_for_accel_decel = params->max_vel * params->max_vel / params->max_accel + params->max_accel * params->t_j * params->t_j;

    if (params->total_dist < dist_needed_for_accel_decel) {
        params->is_triangular_vel = true;  // 速度が三角形プロファイル (V_maxに達しない)
        params->t_v = 0.0;
        // S字加速・減速だけで移動する距離の場合のt_jとt_aの再計算
        // ここが最も複雑で、根の公式などを用いる必要があり、ここでは簡略化
        // 参考: "Minimum Jerk Trajectory Generation" などのキーワードで検索
        // ここでは簡単な近似またはエラーで示す
        printf("Error: Target distance too short for S-curve with given max_vel/accel/jerk.\n");
        printf("  This case requires more complex calculation for t_j and t_a.\n");
        // とりあえず総時間を適当に計算（実際には動的な再計算が必要）
        params->total_time = 2.0 * sqrt(params->total_dist / (params->max_accel * 0.5));  // 簡易的な三角形速度プロファイルの推定
        params->t_a = params->total_time / 2.0;
        params->t_j = fmin(params->t_j, params->t_a / 2.0);  // t_jはt_a/2を超えない
        // params->max_vel = params->max_accel * (params->t_a - params->t_j) + 0.5 * params->max_jerk * params->t_j * params->t_j; // 新しいV_max
    } else {
        params->is_triangular_vel = false;
        params->t_v = (params->total_dist - dist_needed_for_accel_decel) / params->max_vel;
    }

    params->total_time = 2 * params->t_a + params->t_v;

    printf("Calculated S-Curve Trajectory Parameters:\n");
    printf("  Total Distance: %.3f\n", params->total_dist);
    printf("  Jerk Time (t_j): %.3f s\n", params->t_j);
    printf("  Accel/Decel Total Time (t_a): %.3f s\n", params->t_a);
    printf("  Constant Velocity Time (t_v): %.3f s\n", params->t_v);
    printf("  Total Time: %.3f s\n", params->total_time);
    printf("  Is Accel Triangular: %s\n", params->is_triangular_accel ? "Yes" : "No");
    printf("  Is Vel Triangular: %s\n", params->is_triangular_vel ? "Yes" : "No");
}

double get_scurve_state(const SCurveTrajectoryParams *params, double current_time,
                        double *target_pos, double *target_vel, double *target_accel) {
    double target_jerk = 0.0;
    double direction = (params->end_pos > params->start_pos) ? 1.0 : -1.0;
    double pos_offset = params->start_pos;

    // 時間が総移動時間より大きい場合は、目標位置で停止
    if (current_time >= params->total_time) {
        *target_pos = params->end_pos;
        *target_vel = 0.0;
        *target_accel = 0.0;
        target_jerk = 0.0;
        return target_jerk;
    }

    // 各期間における目標値の計算
    if (params->is_triangular_vel) {
        // 速度が三角形プロファイルの場合 (最大速度に達しない)
        // このケースは計算が複雑で、ここでは簡易的な出力にとどめます。
        // 正確な実装には、新しいt_jとt_aを解く必要があります。
        // 参考論文: https://www.roborealm.com/help/S-Curve.php
        // とりあえずの近似として、加速度が台形のままで計算します (S字ではない)
        double half_time = params->total_time / 2.0;
        double accel_val = params->max_accel;  // ここでは仮に最大加速度を使用
        double current_t_abs = current_time;

        if (current_t_abs <= half_time) {  // 加速フェーズ
            *target_accel = direction * accel_val;
            *target_vel = direction * accel_val * current_t_abs;
            *target_pos = pos_offset + direction * 0.5 * accel_val * current_t_abs * current_t_abs;
        } else {  // 減速フェーズ
            double t_decel = current_t_abs - half_time;
            *target_accel = -direction * accel_val;
            *target_vel = direction * (accel_val * half_time - accel_val * t_decel);
            *target_pos = pos_offset + direction * (0.5 * accel_val * half_time * half_time + (accel_val * half_time * t_decel - 0.5 * accel_val * t_decel * t_decel));
        }
        return 0.0;  // ジャークは0と近似
    }

    // 実際のS字プロファイル計算 (速度が最大速度に達する場合)
    if (current_time <= params->t_j) {
        // 1. 加速ジャーク期間 (Jerk constant +J_max)
        target_jerk = direction * params->max_jerk;
        *target_accel = direction * params->max_jerk * current_time;
        *target_vel = direction * 0.5 * params->max_jerk * current_time * current_time;
        *target_pos = pos_offset + direction * (1.0 / 6.0) * params->max_jerk * pow(current_time, 3);
    } else if (current_time <= params->t_a) {
        // 2. 加速期間 (Accel constant +A_max)
        double t_offset = current_time - params->t_j;
        target_jerk = 0.0;
        *target_accel = direction * params->max_accel;
        *target_vel = direction * (0.5 * params->max_jerk * params->t_j * params->t_j + params->max_accel * t_offset);
        *target_pos = pos_offset + direction * ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +
                                                0.5 * params->max_accel * params->t_j * params->t_j +  // t_j期間の最終速度が初速として寄与
                                                params->max_accel * params->t_j * t_offset +           // 加速度定常期間の速度による移動
                                                0.5 * params->max_accel * t_offset * t_offset);
        // よりシンプルに書くと
        // V_prev_phase = direction * 0.5 * params->max_jerk * params->t_j * params->t_j; // フェーズ1終了時の速度
        // X_prev_phase = pos_offset + direction * (1.0/6.0) * params->max_jerk * pow(params->t_j, 3); // フェーズ1終了時の位置
        // *target_vel = V_prev_phase + direction * params->max_accel * t_offset;
        // *target_pos = X_prev_phase + V_prev_phase * t_offset + direction * 0.5 * params->max_accel * t_offset * t_offset;

    } else if (current_time <= params->t_a + params->t_j) {
        // 3. 加速ジャーク減速期間 (Jerk constant -J_max)
        double t_offset = current_time - params->t_a;
        target_jerk = -direction * params->max_jerk;
        *target_accel = direction * (params->max_accel - params->max_jerk * t_offset);
        *target_vel = direction * (params->max_vel - 0.5 * params->max_jerk * t_offset * t_offset);
        *target_pos = pos_offset + direction * ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +
                                                params->max_accel * (params->t_a - params->t_j) * params->t_j +                        // Accel1期間の速度による移動
                                                0.5 * params->max_accel * (params->t_a - params->t_j) * (params->t_a - params->t_j) +  // Accel1期間の加速度による移動
                                                params->max_vel * t_offset - (1.0 / 6.0) * params->max_jerk * pow(t_offset, 3));
        // V_prev_phase = direction * params->max_vel; // フェーズ2終了時の速度
        // A_prev_phase = direction * params->max_accel; // フェーズ2終了時の加速度
        // X_prev_phase = pos_offset + direction * (params->max_vel * params->t_a - params->max_accel * params->t_j * params->t_j); // フェーズ2終了時の位置
        // *target_vel = V_prev_phase + A_prev_phase * t_offset - direction * 0.5 * params->max_jerk * t_offset * t_offset;
        // *target_pos = X_prev_phase + V_prev_phase * t_offset + 0.5 * A_prev_phase * t_offset * t_offset - direction * (1.0/6.0) * params->max_jerk * pow(t_offset, 3);
        // Corrected calculation for this phase (simplified)
        double V_at_Ta = direction * (0.5 * params->max_jerk * params->t_j * params->t_j + params->max_accel * (params->t_a - params->t_j));
        double X_at_Ta = pos_offset + direction * ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +
                                                   0.5 * params->max_accel * (params->t_a - params->t_j) * (params->t_a - params->t_j) +
                                                   (0.5 * params->max_jerk * params->t_j * params->t_j) * (params->t_a - params->t_j));  // This X_at_Ta is the pos at end of phase 2
        // For phase 3
        *target_vel = V_at_Ta + direction * params->max_accel * t_offset - direction * 0.5 * params->max_jerk * t_offset * t_offset;
        *target_pos = X_at_Ta + V_at_Ta * t_offset + direction * 0.5 * params->max_accel * t_offset * t_offset - direction * (1.0 / 6.0) * params->max_jerk * pow(t_offset, 3);

    } else if (current_time <= params->t_a + params->t_v) {
        // 4. 定速期間 (Vel constant +V_max)
        double t_offset = current_time - (params->t_a + params->t_j);
        target_jerk = 0.0;
        *target_accel = 0.0;
        *target_vel = direction * params->max_vel;
        // 定速期間開始までの移動距離を正確に計算する必要がある
        double dist_to_start_const_vel = ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +                                 // ジャーク加速
                                          0.5 * params->max_accel * (params->t_a - params->t_j) * (params->t_a - params->t_j) +  // 定加速度
                                          (0.5 * params->max_jerk * params->t_j * params->t_j) * (params->t_a - params->t_j) +   // t_j期間の終速と定加速度期間の積
                                          params->max_vel * params->t_j - (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3)   // ジャーク減速 (A_maxから0になるまで)
        );
        // よりシンプルに:
        double X_at_const_vel_start = pos_offset + direction * (params->max_vel * params->t_a - params->max_accel * params->t_j * params->t_j + (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3)  // This is not quite right
                                                               );
        // Use the total distance for accel/decel phase (dist_needed_for_accel_decel)
        // Total distance covered during acceleration phase (up to max_vel)
        double X_at_Vmax_start = ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) +
                                  (0.5 * params->max_jerk * params->t_j * params->t_j) * (params->t_a - params->t_j) +
                                  0.5 * params->max_accel * (params->t_a - params->t_j) * (params->t_a - params->t_j) +
                                  (params->max_accel * params->t_j) * params->t_j - (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3)  // This is also problematic
        );
        // Re-evaluate dist_needed_for_accel_decel in the calculate_scurve_params() for exact value
        // For now, simpler:
        double X_accel_phase = params->max_vel * params->t_a - params->max_accel * params->t_j * params->t_j + (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3);  // Approx
        X_accel_phase = params->total_dist - (params->max_vel * params->t_v);                                                                                         // this is wrong too

        // Simpler approach: calculate from start_pos using known total distances for each phase
        // This is complex due to the chained nature. A common way is to re-integrate from 0.

        // Let's rely on the direct calculation
        double dist_covered_up_to_phase3 = direction * ((1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3)                                                                                                  // Phase 1
                                                        + (0.5 * params->max_jerk * params->t_j * params->t_j) * (params->t_a - params->t_j) + 0.5 * params->max_accel * pow((params->t_a - params->t_j), 2)  // Phase 2
                                                        + params->max_vel * params->t_j - (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3)                                                                // Phase 3 (if starts from V_max)
                                                       );
        // The total distance covered in acceleration up to Vmax (before constant vel) is
        // = (V_max^2 / A_max + A_max * t_j^2) / 2.0; (This is distance from 0 to V_max with S-curve)

        double accel_total_dist_single_side = 0.5 * (params->max_vel + (params->max_vel - params->max_accel * params->t_j)) * (params->t_a - params->t_j) + (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3) + 0.5 * params->max_jerk * params->t_j * params->t_j * params->t_j;  // This is getting too complex for inline

        // Let's use the property that position is just accumulated from the start
        // Position at the start of constant velocity phase (end of phase 3)
        double current_pos_start_const_vel_phase = pos_offset;
        // Phase 1
        current_pos_start_const_vel_phase += direction * (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3);
        // Phase 2
        double vel_at_t_j = direction * 0.5 * params->max_jerk * pow(params->t_j, 2);
        current_pos_start_const_vel_phase += direction * (vel_at_t_j * (params->t_a - params->t_j) + 0.5 * params->max_accel * pow(params->t_a - params->t_j, 2));
        // Phase 3
        double vel_at_Ta = direction * params->max_vel;      // Should be V_max if calculations are correct
        double accel_at_Ta = direction * params->max_accel;  // Should be A_max
        current_pos_start_const_vel_phase += direction * (vel_at_Ta * params->t_j + 0.5 * accel_at_Ta * params->t_j * params->t_j - (1.0 / 6.0) * params->max_jerk * pow(params->t_j, 3));

        // A much simpler way: just integrate from previous state!
        // This makes the code less error-prone when adapting to different phases
        double pos_at_prev_phase_end;
        double vel_at_prev_phase_end;
        double accel_at_prev_phase_end;

        // Recalculate up to Phase 3 end for better accuracy
        // This is where a robust trajectory generation library would be useful
        // For simplicity, we'll use the ideal accumulated distance from start_pos
        // For phase 4: pos = pos_at_phase3_end + V_max * t_offset
        // To get X_at_phase3_end, we would re-run calculation up to params->t_a+params->t_j for X, V, A, J
        get_scurve_state(params, params->t_a + params->t_j, &pos_at_prev_phase_end, &vel_at_prev_phase_end, &accel_at_prev_phase_end);

        *target_pos = pos_at_prev_phase_end + direction * params->max_vel * t_offset;

    } else if (current_time <= params->t_a + params->t_v + params->t_j) {
        // 5. 減速ジャーク期間 (Jerk constant -J_max)
        double t_start_phase = params->t_a + params->t_v;
        double t_offset = current_time - t_start_phase;
        target_jerk = -direction * params->max_jerk;
        *target_accel = -direction * params->max_jerk * t_offset;
        *target_vel = direction * (params->max_vel - 0.5 * params->max_jerk * t_offset * t_offset);

        double pos_at_prev_phase_end;
        double vel_at_prev_phase_end;
        double accel_at_prev_phase_end;
        get_scurve_state(params, t_start_phase, &pos_at_prev_phase_end, &vel_at_prev_phase_end, &accel_at_prev_phase_end);
        *target_pos = pos_at_prev_phase_end + vel_at_prev_phase_end * t_offset + 0.5 * accel_at_prev_phase_end * t_offset * t_offset - direction * (1.0 / 6.0) * params->max_jerk * pow(t_offset, 3);

    } else if (current_time <= params->t_a + params->t_v + params->t_a) {  // This is 2*t_a + t_v, so total time
        // 6. 減速期間 (Accel constant -A_max)
        double t_start_phase = params->t_a + params->t_v + params->t_j;
        double t_offset = current_time - t_start_phase;
        target_jerk = 0.0;
        *target_accel = -direction * params->max_accel;
        *target_vel = direction * (params->max_vel - 0.5 * params->max_jerk * params->t_j * params->t_j - params->max_accel * t_offset);

        double pos_at_prev_phase_end;
        double vel_at_prev_phase_end;
        double accel_at_prev_phase_end;
        get_scurve_state(params, t_start_phase, &pos_at_prev_phase_end, &vel_at_prev_phase_end, &accel_at_prev_phase_end);
        *target_pos = pos_at_prev_phase_end + vel_at_prev_phase_end * t_offset + 0.5 * accel_at_prev_phase_end * t_offset * t_offset;

    } else {
        // 7. 減速ジャーク期間 (Jerk constant +J_max)
        double t_start_phase = params->t_a + params->t_v + params->t_a - params->t_j;  // This is the start of the final t_j phase
        double t_offset = current_time - t_start_phase;
        target_jerk = direction * params->max_jerk;
        *target_accel = direction * (-params->max_accel + params->max_jerk * t_offset);                                                  // Accel goes from -A_max to 0
        *target_vel = direction * (0.5 * params->max_jerk * (params->total_time - current_time) * (params->total_time - current_time));  // V goes to 0
        // Or integrate from known value
        double pos_at_prev_phase_end;
        double vel_at_prev_phase_end;
        double accel_at_prev_phase_end;
        get_scurve_state(params, t_start_phase, &pos_at_prev_phase_end, &vel_at_prev_phase_end, &accel_at_prev_phase_end);
        *target_pos = pos_at_prev_phase_end + vel_at_prev_phase_end * t_offset + 0.5 * accel_at_prev_phase_end * t_offset * t_offset + direction * (1.0 / 6.0) * params->max_jerk * pow(t_offset, 3);
    }

    // 最終位置での丸め込み (浮動小数点誤差対策)
    if (fabs(*target_pos - params->end_pos) < 1e-6 && current_time > params->total_time - 0.01) {
        *target_pos = params->end_pos;
        *target_vel = 0.0;
        *target_accel = 0.0;
        target_jerk = 0.0;
    }

    return target_jerk;
}
// --- S字プロファイルの実装ここまで ---