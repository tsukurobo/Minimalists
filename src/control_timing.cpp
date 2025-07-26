#include "control_timing.hpp"

#include <stdint.h>
#include <stdio.h>

// 制御タイミング構造体の初期化
void init_control_timing(control_timing_t* timing) {
    timing->next_control_time = get_absolute_time();
    timing->loop_start_time = get_absolute_time();
    timing->timing_violation_count = 0;
    timing->led_mode = LED_OFF;
    timing->overflow_mode = OVERFLOW_CONTINUOUS;  // デフォルトは連続オーバーフロー
    timing->min_wait_us = 1000;                 // デフォルト1ms
}

// 制御周期開始処理（ループの最初に呼ぶ）
void control_timing_start(control_timing_t* timing, float control_period_ms) {
    timing->loop_start_time = get_absolute_time();
    // 次回制御時刻を制御周期分だけ進める
    uint64_t period_us = (uint64_t)(control_period_ms * 1000);
    timing->next_control_time = delayed_by_us(timing->next_control_time, period_us);
}

// 制御周期終了処理（ループの最後に呼ぶ）
void control_timing_end(control_timing_t* timing, float control_period_ms) {
    absolute_time_t current_time = get_absolute_time();  // 1回だけ取得

    // 周期超過判定（1回の時刻差分計算のみ）
    bool timing_violation = absolute_time_diff_us(timing->next_control_time, current_time) > 0;

    // // 処理時間を表示（デバッグ用）
    // int64_t processing_time_us = absolute_time_diff_us(timing->loop_start_time, current_time);
    // printf("PROCESSING TIME: %fms\n", processing_time_us / 1000.0);

    if (timing_violation) {
        // === 周期超過時の軽量処理 ===
        timing->timing_violation_count++;
        timing->led_mode = LED_ON;

        // // デバッグ出力
        // int64_t processing_time_us = absolute_time_diff_us(timing->loop_start_time, current_time);
        // printf("TIMING VIOLATION: %.1fms\n", processing_time_us / 1000.0);

        // 軽量版オーバーフロー処理（条件分岐最小化）
        uint64_t period_us = (uint64_t)(control_period_ms * 1000);

        if (timing->overflow_mode == OVERFLOW_CONTINUOUS) {
            // 即座に次周期開始（最軽量）
            timing->next_control_time = delayed_by_us(current_time, period_us);
        } else {
            // 最小待機 or 半周期待機（統一処理）
            uint64_t wait_us = (timing->overflow_mode == OVERFLOW_HALF_PERIOD) ? (period_us / 2) : timing->min_wait_us;

            absolute_time_t wait_end = delayed_by_us(current_time, wait_us);
            timing->next_control_time = delayed_by_us(current_time, period_us);
            busy_wait_until(wait_end);
        }
    } else {
        // === 正常時の軽量処理 ===
        timing->led_mode = LED_OFF;
        busy_wait_until(timing->next_control_time);  // 直接待機
    }

    // LED制御
    gpio_put(PICO_DEFAULT_LED_PIN, timing->led_mode == LED_ON);
}
// 制御状態のデバッグ情報取得
const char* get_led_status_string(led_mode_t mode) {
    return (mode == LED_ON) ? "ON" : "OFF";
}

// 詳細デバッグ情報を取得（必要時のみ呼び出し）
void get_timing_stats(const control_timing_t* timing, float control_period_ms, timing_stats_t* stats) {
    absolute_time_t current_time = get_absolute_time();

    stats->processing_time_us = absolute_time_diff_us(timing->loop_start_time, current_time);
    stats->processing_time_ms = stats->processing_time_us / 1000.0;
    stats->control_period_ms = control_period_ms;
    stats->violation_count = timing->timing_violation_count;
    stats->is_violation = (stats->processing_time_ms > control_period_ms);
    stats->cpu_usage_percent = (stats->processing_time_ms / control_period_ms) * 100.0;
}

// 制御周期超過時の動作モードを設定
void set_timing_overflow_mode(control_timing_t* timing, timing_overflow_mode_t mode, uint32_t min_wait_us) {
    timing->overflow_mode = mode;
    timing->min_wait_us = min_wait_us;
}
