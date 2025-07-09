#include <stdio.h>
#include <stdlib.h>

#include "dynamics.hpp"
#include "mcp25625.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"

// PicoのSPI設定
#define SPI_PORT spi1                    // SPI1を使用
constexpr int SPI_BAUDRATE = 1'000'000;  // 1MHz
constexpr int PIN_MISO = 8;
constexpr int PIN_CS = 9;
constexpr int PIN_SCK = 10;
constexpr int PIN_MOSI = 11;
constexpr int PIN_RST = 20;
constexpr int PIN_INT = 21;  // 割り込みは今回不使用

// MCP25625オブジェクトを作成
mcp25625_t can(SPI_PORT, PIN_CS, PIN_RST);

// ベースのモータから出力軸までのギア比
double gear_ratio_R = 3591.0 / 187.0 * 3.0;  // M3508(3591.0/187.0) * M3508出力軸からベース根本(3.0)
double gear_ratio_P = 36.0;                  // M2006 P36のギア比

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, gear_ratio_P);  // motor_id=2

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
    // CANの初期化（リトライ付き）
    while (!can.init(CAN_1000KBPS)) {
        printf("MCP25625 Initialization failed. Retrying in 2 seconds...\n");
        for (int i = 0; i < 4; ++i) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(250);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(250);
        }
    }
    printf("MCP25625 Initialized successfully!\n");

    double target_current[4] = {0.0, 0.0, 0.0, 0.0};  // モータ1~4の目標電流値(A)

    while (true) {
        absolute_time_t next_time = make_timeout_time_ms(250);

        // モーター1の目標電流値(A)を設定（例: -10A〜10Aで変化）
        target_current[0] += 0.5;
        if (target_current[0] > 10.0) target_current[0] = -10.0;

        // --- 送信処理 ---
        if (send_all_motor_currents(&can, target_current)) {
            printf("Sent current (A): %.2f\n", target_current[0]);
        } else {
            printf("Failed to send current command.\n");
        }

        sleep_ms(10);  // 送信間隔を調整

        // --- 受信処理 ---
        if (motor1.receive_feedback()) {
            printf("Motor1 Feedback -> Angle: %.2f [rad], Speed: %.2f [rad/s]\n",
                   motor1.get_continuous_angle(), motor1.get_angular_velocity());
        } else {
            printf("No CAN message received for Motor1.\n");
        }

        // mutex_enter_blocking(&g_state_mutex);
        // int speed = g_robot_state.motor_speed;
        // int sensor = g_robot_state.sensor_value;
        // mutex_exit(&g_state_mutex);

        // // デバッグ出力
        // printf("[DEBUG] speed=%d, sensor=%d\n", speed, sensor);

        // 通信処理（例: USB出力やUART送信など）ここに追加可能

        busy_wait_until(next_time);
    }
}

int main(void) {
    stdio_init_all();  // UARTなど初期化
    // SPIの初期化
    spi_init(SPI_PORT, SPI_BAUDRATE);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    sleep_ms(2000);  // シリアル接続待ち

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
