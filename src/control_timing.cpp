#include "control_timing.hpp"

#include <stdint.h>
#include <stdio.h>

// 制御タイミング構造体の初期化
void init_control_timing(control_timing_t* timing) {
    timing->next_control_time = get_absolute_time();
    timing->loop_start_time = get_absolute_time();
    timing->timing_violation_count = 0;
    timing->led_toggle_time = get_absolute_time();
    timing->led_state = false;
    timing->led_mode = LED_OFF;
    timing->overflow_mode = OVERFLOW_MIN_WAIT;  // デフォルトは最小待機
    timing->min_wait_us = 1000;                 // デフォルト1ms
}

// 制御周期開始処理（ループの最初に呼ぶ）
void control_timing_start(control_timing_t* timing, double control_period_ms) {
    timing->loop_start_time = get_absolute_time();

    // 前回の制御周期からの実際の経過時間をチェック
    int64_t actual_period_us = absolute_time_diff_us(timing->next_control_time, timing->loop_start_time);
    if (actual_period_us > (int64_t)(control_period_ms * 1000 * 1.1)) {  // 10%超過で警告
        timing->timing_violation_count++;
        timing->led_mode = LED_SLOW_BLINK;                // 軽微な超過 - ゆっくり点滅
        if (timing->timing_violation_count % 100 == 1) {  // 100回に1回警告表示
            printf("WARNING: Control timing violation! Actual period: %.2f ms (target: %.1f ms)\n",
                   actual_period_us / 1000.0, control_period_ms);
        }
    } else {
        timing->led_mode = LED_OFF;  // 正常時 - LED消灯
    }

    // 次回の制御時刻を設定
    timing->next_control_time = delayed_by_us(timing->next_control_time, (uint64_t)(control_period_ms * 1000));
}

// LED制御処理
void update_led_control(control_timing_t* timing) {
    absolute_time_t current_led_time = get_absolute_time();

    switch (timing->led_mode) {
        case LED_OFF:
            gpio_put(PICO_DEFAULT_LED_PIN, 0);  // LED消灯
            break;

        case LED_SLOW_BLINK:
            // 500ms周期で点滅
            if (absolute_time_diff_us(timing->led_toggle_time, current_led_time) >= 500000) {
                timing->led_state = !timing->led_state;
                gpio_put(PICO_DEFAULT_LED_PIN, timing->led_state);
                timing->led_toggle_time = current_led_time;
            }
            break;

        case LED_FAST_BLINK:
            // 100ms周期で高速点滅
            if (absolute_time_diff_us(timing->led_toggle_time, current_led_time) >= 100000) {
                timing->led_state = !timing->led_state;
                gpio_put(PICO_DEFAULT_LED_PIN, timing->led_state);
                timing->led_toggle_time = current_led_time;
            }
            break;
    }
}

// 制御周期終了処理（ループの最後に呼ぶ）
void control_timing_end(control_timing_t* timing, double control_period_ms) {
    // 制御周期調整
    absolute_time_t processing_end_time = get_absolute_time();
    int64_t processing_time_us = absolute_time_diff_us(timing->loop_start_time, processing_end_time);

    if (absolute_time_diff_us(get_absolute_time(), timing->next_control_time) > 0) {
        // 処理時間が制御周期を超過した場合
        timing->timing_violation_count++;
        timing->led_mode = LED_FAST_BLINK;  // 重大な超過 - 高速点滅
        printf("CRITICAL: Processing time exceeded control period! Processing: %.2f ms\n",
               processing_time_us / 1000.0);

        // 制御周期を超過した場合の対応策を設定に基づいて選択
        switch (timing->overflow_mode) {
            case OVERFLOW_MIN_WAIT:
                // 最小待機時間を設ける（推奨）
                {
                    absolute_time_t min_wait_time = delayed_by_us(get_absolute_time(), timing->min_wait_us);
                    timing->next_control_time = delayed_by_us(get_absolute_time(), (uint64_t)(control_period_ms * 1000));
                    busy_wait_until(min_wait_time);
                }
                break;

            case OVERFLOW_CONTINUOUS:
                // 完全に連続実行を許可
                timing->next_control_time = delayed_by_us(get_absolute_time(), (uint64_t)(control_period_ms * 1000));
                break;

            case OVERFLOW_HALF_PERIOD:
                // 制御周期の半分だけ待機
                {
                    absolute_time_t half_period_wait = delayed_by_us(get_absolute_time(), (uint64_t)(control_period_ms * 500));
                    timing->next_control_time = delayed_by_us(get_absolute_time(), (uint64_t)(control_period_ms * 1000));
                    busy_wait_until(half_period_wait);
                }
                break;
        }
    } else {
        // 正常時は次回制御時刻まで待機
        busy_wait_until(timing->next_control_time);
    }

    // LED制御処理
    update_led_control(timing);
}

// 制御状態のデバッグ情報取得
const char* get_led_status_string(led_mode_t mode) {
    static const char* led_status[] = {"OFF", "SLOW_BLINK", "FAST_BLINK"};
    return led_status[mode];
}

// 制御周期超過時の動作モードを設定
void set_timing_overflow_mode(control_timing_t* timing, timing_overflow_mode_t mode, uint32_t min_wait_us) {
    timing->overflow_mode = mode;
    timing->min_wait_us = min_wait_us;
}
