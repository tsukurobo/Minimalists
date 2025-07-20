#ifndef CONTROL_TIMING_HPP
#define CONTROL_TIMING_HPP

#include "pico/stdlib.h"

// LED制御モード
enum led_mode_t {
    LED_OFF,
    LED_SLOW_BLINK,
    LED_FAST_BLINK
};

// 制御周期超過時の動作モード
enum timing_overflow_mode_t {
    OVERFLOW_MIN_WAIT,    // 最小待機時間を設ける（推奨）
    OVERFLOW_CONTINUOUS,  // 連続実行を許可
    OVERFLOW_HALF_PERIOD  // 制御周期の半分だけ待機
};

// 制御周期管理構造体
typedef struct {
    absolute_time_t next_control_time;
    absolute_time_t loop_start_time;
    int timing_violation_count;

    // LED制御用
    absolute_time_t led_toggle_time;
    bool led_state;
    led_mode_t led_mode;

    // 制御周期超過時の動作設定
    timing_overflow_mode_t overflow_mode;
    uint32_t min_wait_us;  // 最小待機時間（マイクロ秒）
} control_timing_t;

// 制御周期管理関数の宣言

/**
 * @brief 制御タイミング構造体の初期化
 * @param timing 初期化する制御タイミング構造体のポインタ
 */
void init_control_timing(control_timing_t* timing);

/**
 * @brief 制御周期開始処理（ループの最初に呼ぶ）
 * @param timing 制御タイミング構造体のポインタ
 * @param control_period_ms 制御周期 [ms]
 */
void control_timing_start(control_timing_t* timing, double control_period_ms);

/**
 * @brief LED制御処理
 * @param timing 制御タイミング構造体のポインタ
 */
void update_led_control(control_timing_t* timing);

/**
 * @brief 制御周期終了処理（ループの最後に呼ぶ）
 * @param timing 制御タイミング構造体のポインタ
 * @param control_period_ms 制御周期 [ms]
 */
void control_timing_end(control_timing_t* timing, double control_period_ms);

/**
 * @brief 制御状態のデバッグ情報文字列取得
 * @param mode LED制御モード
 * @return LED状態を表す文字列
 */
const char* get_led_status_string(led_mode_t mode);

/**
 * @brief 制御状態のデバッグ情報文字列取得
 * @param mode LED制御モード
 * @return LED状態を表す文字列
 */
const char* get_led_status_string(led_mode_t mode);

/**
 * @brief 制御周期超過時の動作モードを設定
 * @param timing 制御タイミング構造体のポインタ
 * @param mode 超過時の動作モード
 * @param min_wait_us 最小待機時間（マイクロ秒、OVERFLOW_MIN_WAITモード時のみ使用）
 */
void set_timing_overflow_mode(control_timing_t* timing, timing_overflow_mode_t mode, uint32_t min_wait_us = 1000);

#endif  // CONTROL_TIMING_HPP
