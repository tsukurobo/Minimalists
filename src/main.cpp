#include <stdio.h>
#include <stdlib.h>

#include "dynamics.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "trajectory.hpp"

// 共有データ構造体
typedef struct
{
    int motor_speed;
    int sensor_value;
} robot_state_t;

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

// Core 1: 通信・デバッグ出力担当
void core1_entry(void) {
    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(250);

        mutex_enter_blocking(&g_state_mutex);
        int speed = g_robot_state.motor_speed;
        int sensor = g_robot_state.sensor_value;
        mutex_exit(&g_state_mutex);

        // デバッグ出力
        printf("[DEBUG] speed=%d, sensor=%d\n", speed, sensor);

        // 通信処理（例: USB出力やUART送信など）ここに追加可能

        busy_wait_until(next_time);
    }
}

int main(void) {
    stdio_init_all();  // UARTなど初期化
    sleep_ms(2000);    // シリアル接続待ち

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // ミューテックス初期化
    mutex_init(&g_state_mutex);
    g_robot_state.motor_speed = 0;
    g_robot_state.sensor_value = 0;

    // Core1で実行する関数を起動
    multicore_launch_core1(core1_entry);

    while (1) {
        absolute_time_t next_time = make_timeout_time_ms(500);  // 今から500ms後

        // LEDを点滅させる
        static bool led_on = false;
        led_on = !led_on;
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
        // ループごとに点灯/消灯を切り替え（点滅周期は制御周期の2倍）

        // センサ読み取りや制御計算の疑似処理
        int new_sensor = rand() % 100;
        int new_speed = new_sensor * 2;

        // 状態を更新（排他制御あり）
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.motor_speed = new_speed;
        g_robot_state.sensor_value = new_sensor;
        mutex_exit(&g_state_mutex);

        // 実際のモータ制御などをここで行う（PWM制御など）
        // motor_set_speed(new_speed);

        busy_wait_until(next_time);  // 500ms待機
    }

    return 0;
}
