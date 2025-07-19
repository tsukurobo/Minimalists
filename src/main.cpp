#include <stdio.h>
#include <stdlib.h>

#include "amt223v.hpp"
#include "dynamics.hpp"
#include "mcp25625.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"

// SPI設定構造体
typedef struct {
    spi_inst_t* spi_port;
    uint32_t baudrate;
    int pin_miso;
    int pin_cs[4];    // 最大4つのCSピンをサポート
    int num_cs_pins;  // 実際に使用するCSピンの数
    int pin_sck;
    int pin_mosi;
    int pin_rst;
} spi_config_t;

// PicoのSPI設定
constexpr int SHUTDOWN_PIN = 27;  // 明示的にLOWにしないとPicoが動かない

// CAN IC用SPI設定
static const spi_config_t can_spi_config = {
    .spi_port = spi1,       // SPI1を使用
    .baudrate = 1'000'000,  // 1MHz
    .pin_miso = 8,
    .pin_cs = {5},     // CSピン1つ
    .num_cs_pins = 1,  // CSピン数
    .pin_sck = 10,
    .pin_mosi = 11,
    .pin_rst = 20,
};

// MCP25625オブジェクトを作成（CAN SPI設定を使用）
mcp25625_t can(can_spi_config.spi_port, can_spi_config.pin_cs[0], can_spi_config.pin_rst);

// AMT223-V エンコーダマネージャを作成
AMT223V_Manager encoder_manager(spi1,       // SPI1を使用
                                1'000'000,  // 1MHz
                                16,         // MISO pin
                                18,         // SCK pin
                                19);        // MOSI pin

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
    double encoder1_angle;  // エンコーダ1の角度 [rad]
    double encoder2_angle;  // エンコーダ2の角度 [rad]
} robot_state_t;

// SPI初期化関数
bool init_spi(const spi_config_t* config) {
    // SPIの初期化
    spi_init(config->spi_port, config->baudrate);
    gpio_set_function(config->pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(config->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(config->pin_mosi, GPIO_FUNC_SPI);

    // 複数のCSピンを設定
    for (int i = 0; i < config->num_cs_pins; i++) {
        gpio_init(config->pin_cs[i]);
        gpio_set_dir(config->pin_cs[i], GPIO_OUT);
        gpio_put(config->pin_cs[i], 1);  // CS初期状態はHIGH
        printf("CS%d pin %d initialized\n", i, config->pin_cs[i]);
    }

    // リセットピンがある場合の設定
    if (config->pin_rst >= 0) {
        gpio_init(config->pin_rst);
        gpio_set_dir(config->pin_rst, GPIO_OUT);
        gpio_put(config->pin_rst, 1);  // リセット解除
        printf("Reset pin %d initialized\n", config->pin_rst);
    }

    return true;
}

// 複数のSPIデバイスを初期化する関数
bool init_all_spi_devices() {
    // CAN IC用SPI初期化
    if (!init_spi(&can_spi_config)) {
        printf("Failed to initialize CAN SPI!\n");
        return false;
    }
    printf("CAN SPI initialized successfully!\n");

    return true;
}

// エンコーダの初期化関数
bool init_encoders() {
    // エンコーダを追加
    int encoder1_index = encoder_manager.add_encoder(7);  // CS pin 7
    int encoder2_index = encoder_manager.add_encoder(6);  // CS pin 6

    if (encoder1_index < 0 || encoder2_index < 0) {
        printf("Failed to add encoders!\n");
        return false;
    }

    // SPI初期化
    if (!encoder_manager.init_spi()) {
        printf("Failed to initialize encoder SPI!\n");
        return false;
    }

    // 全エンコーダ初期化
    if (!encoder_manager.init_all_encoders()) {
        printf("Failed to initialize encoders!\n");
        return false;
    }

    return true;
}

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

// Core 1: 通信・デバッグ出力担当
void core1_entry(void) {
    // CANの初期化（リトライ付き）
    while (!can.init(CAN_1000KBPS)) {
        printf("MCP25625 Initialization failed. Retrying in 2 seconds...\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(10);
    }
    printf("MCP25625 Initialized successfully!\n");

    double target_current[4] = {0.0, 0.0, 0.0, 0.0};  // モータ1~4の目標電流値(A)

    while (true) {
        absolute_time_t next_time = make_timeout_time_ms(250);

        // モーター1の目標電流値(A)を設定（例: -1A〜1Aで変化）
        target_current[0] += 0.1;
        if (target_current[0] > 1.0) target_current[0] = -1.0;

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

        // --- エンコーダ読み取り処理 ---
        double enc1_angle = 0.0, enc2_angle = 0.0;
        bool enc1_ok = encoder_manager.read_encoder(0);  // エンコーダ0
        bool enc2_ok = encoder_manager.read_encoder(1);  // エンコーダ1

        if (enc1_ok) {
            enc1_angle = encoder_manager.get_encoder_angle_rad(0);
            printf("Encoder1: %.4f [rad] (%.1f [deg])\n",
                   enc1_angle, encoder_manager.get_encoder_angle_deg(0));
        } else {
            printf("Failed to read Encoder1\n");
        }

        if (enc2_ok) {
            enc2_angle = encoder_manager.get_encoder_angle_rad(1);
            printf("Encoder2: %.4f [rad] (%.1f [deg])\n",
                   enc2_angle, encoder_manager.get_encoder_angle_deg(1));
        } else {
            printf("Failed to read Encoder2\n");
        }

        // 共有データ更新
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.encoder1_angle = enc1_angle;
        g_robot_state.encoder2_angle = enc2_angle;
        mutex_exit(&g_state_mutex);

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
    gpio_init(SHUTDOWN_PIN);
    gpio_set_dir(SHUTDOWN_PIN, GPIO_OUT);
    gpio_put(SHUTDOWN_PIN, 0);  // HIGHにしておくとPicoが動かないのでLOWに設定
    sleep_ms(2000);             // 少し待機して安定化

    // 全SPIデバイスの初期化
    if (!init_all_spi_devices()) {
        return -1;
    }

    // エンコーダの初期化
    if (!init_encoders()) {
        return -1;
    }

    sleep_ms(2000);  // シリアル接続待ち

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // ミューテックス初期化
    mutex_init(&g_state_mutex);
    g_robot_state.motor_speed = 0;
    g_robot_state.sensor_value = 0;
    g_robot_state.encoder1_angle = 0.0;
    g_robot_state.encoder2_angle = 0.0;

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
