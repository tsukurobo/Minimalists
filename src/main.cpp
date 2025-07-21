#include <stdio.h>
#include <stdlib.h>

#include <cmath>

#include "amt223v.hpp"
#include "control_timing.hpp"
#include "dynamics.hpp"
#include "mcp25625.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "pid_controller.hpp"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"

// 制御周期定数
constexpr double CONTROL_PERIOD_MS = 2.0;                        // 制御周期 [ms]
constexpr double CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0;  // 制御周期 [s]

// システム設定定数
constexpr int SHUTDOWN_PIN = 27;          // 明示的にLOWにしないとPicoが動かない
constexpr int DEBUG_PRINT_INTERVAL = 25;  // デバッグ出力間隔（制御周期の倍数）

// モータとエンコーダの符号補正設定
constexpr double ENCODER_R_DIRECTION = 1.0;   // R軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で右回り、右回りでエンコーダ値が増加
constexpr double ENCODER_P_DIRECTION = -1.0;  // P軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で根本方向、根本方向でエンコーダ値が減少

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
AMT223V_Manager encoder_manager(spi0,       // SPI0を使用
                                1'000'000,  // 2MHz
                                16,         // MISO pin
                                18,         // SCK pin
                                19);        // MOSI pin

// ベースのモータから出力軸までのギア比
constexpr double gear_ratio_R = 3591.0 / 187.0 * 3.0;  // M3508(3591.0/187.0) * M3508出力軸からベース根本(3.0)
constexpr double gear_ratio_P = 36.0;                  // M2006 P36のギア比
constexpr double gear_radius_P = 0.025;                // ギアの半径 (m) - M2006の出力軸からラックまでの距離が25mm

// R軸（ベース回転）の動力学パラメータ
dynamics_t dynamics_R(
    0.024371,                 // 等価慣性モーメント (kg·m^2)
    0.036437,                 // 等価粘性摩擦係数 (N·m·s/rad)
    0.3 * gear_ratio_R * 0.7  // 等価トルク定数（M3508のトルク定数xギア比x伝達効率）(Nm/A)
);

// P軸（アーム直動）の動力学パラメータ
dynamics_t dynamics_P(
    0.3,                                        // 等価慣性モーメント (kg·m^2)
    0.002651,                                   // 粘性摩擦係数 (N·m·s/rad)
    0.18 * gear_ratio_P * 0.66 / gear_radius_P  // 力定数（M2006のトルク定数xギア比x伝達効率/ギアの半径）(N/A)
);

// R軸（ベース回転）の軌道生成パラメータ
trajectory_t trajectory_R(
    6.0,      // 最大角速度 (rad/s)
    20.0,     // 最大角加速度 (rad/s^2)
    0.0,      // 開始位置 (rad)
    2 * M_PI  // 目標位置 (rad)
);

// P軸（アーム直動）の軌道生成パラメータ
trajectory_t trajectory_P(
    0.5 / gear_radius_P,  // 最大速度 (m/s) / ギアの円周(m) * 2pi = 最大速度 / ギアの半径 = 最大角速度 (rad/s)
    2.0 / gear_radius_P,  // 最大角加速度 (rad/s^2)
    0.0 / gear_radius_P,  // 開始位置 (rad)
    0.3 / gear_radius_P   // 目標位置 (rad) - 300mm移動
);

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, gear_ratio_P);  // motor_id=2

// PIDコントローラ（モータ1: 回転軸、モータ2: 直動軸）
// 位置PID制御器（位置[rad] → 目標速度[rad/s]）
const double R_POSITION_KP = 2.0;  // R軸位置PIDの比例ゲイン
const double R_VELOCITY_KP = 5.0;  // R軸速度I-Pの比例ゲイン
const double R_VELOCITY_KI = 0.5;  // R軸速度I-Pの積分ゲイン
// 27 * R_POSITION_KP * R_POSITION_KP * dynamics_R.get_inertia_mass();                           // R軸速度I-Pの積分ゲイン
const double P_POSITION_KP = 20.0;  // P軸位置PID
const double P_VELOCITY_KP = 20.0;  // P軸速度I-Pの比例ゲイン
const double P_VELOCITY_KI = 0.1;   // P軸速度I-Pの積分ゲイン

PositionPIDController position_pid_R(R_POSITION_KP, 0.0, 0.0, CONTROL_PERIOD_S);  // Kp, Ki, Kd
PositionPIDController position_pid_P(P_POSITION_KP, 0.0, 0.0, CONTROL_PERIOD_S);  // Kp, Ki, Kd

// 速度I-P制御器（速度[rad/s] → トルク[Nm]）
VelocityIPController velocity_ip_R(R_VELOCITY_KI, R_VELOCITY_KP, CONTROL_PERIOD_S);  // Ki, Kp
VelocityIPController velocity_ip_P(P_VELOCITY_KI, P_VELOCITY_KP, CONTROL_PERIOD_S);  // Ki, Kp

// 共有データ構造体
typedef struct
{
    int motor_speed;
    int sensor_value;

    // 制御目標値
    double target_position_R;  // R軸目標位置 [rad]
    double target_position_P;  // P軸目標位置 [rad]

    // 制御状態
    double current_position_R;  // R軸現在位置 [rad]
    double current_position_P;  // P軸現在位置 [rad]
    double current_velocity_R;  // R軸現在速度 [rad/s]
    double current_velocity_P;  // P軸現在速度 [rad/s]

    // 制御出力
    double target_velocity_R;  // R軸目標速度 [rad/s]（位置PIDの出力）
    double target_velocity_P;  // P軸目標速度 [rad/s]（位置PIDの出力）
    double target_torque_R;    // R軸目標トルク [Nm]（速度I-Pの出力）
    double target_torque_P;    // P軸目標トルク [Nm]（速度I-Pの出力）

    // デバッグ・制御タイミング情報
    double target_current_R;     // R軸目標電流 [A]
    double target_current_P;     // P軸目標電流 [A]
    int timing_violation_count;  // 制御周期違反回数
    led_mode_t led_status;       // LED状態
    int can_error_count;         // CAN送信エラー回数
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
    int encoder1_index = encoder_manager.add_encoder(7, false);  // CS pin 7 (R軸: 単回転)
    int encoder2_index = encoder_manager.add_encoder(6, true);   // CS pin 6 (P軸: マルチターン対応)

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

// PIDコントローラの初期化関数
bool init_pid_controllers() {
    printf("Initializing PID controllers...\n");
    printf("Control period: %.1f ms (%.0f Hz)\n", CONTROL_PERIOD_MS);

    // 方向補正設定の表示
    printf("Direction correction settings:\n");
    printf("  Encoder R direction: %+.1f\n", ENCODER_R_DIRECTION);
    printf("  Encoder P direction: %+.1f\n", ENCODER_P_DIRECTION);

    // 位置PID制御器の設定
    position_pid_R.setOutputLimits(-10.0, 10.0);  // 目標速度制限 ±10 rad/s
    position_pid_R.setIntegralLimits(-5.0, 5.0);  // 積分制限

    position_pid_P.setOutputLimits(-5.0, 5.0);    // 目標速度制限 ±5 rad/s
    position_pid_P.setIntegralLimits(-3.0, 3.0);  // 積分制限

    // 速度I-P制御器の設定
    velocity_ip_R.setOutputLimits(-16.0, 16.0);    // トルク制限 ±16 Nm（M3508最大トルク）
    velocity_ip_R.setIntegralLimits(-10.0, 10.0);  // 積分制限

    velocity_ip_P.setOutputLimits(-10.0, 10.0);  // トルク制限 ±10 Nm（M2006最大トルク）
    velocity_ip_P.setIntegralLimits(-8.0, 8.0);  // 積分制限

    printf("PID controllers initialized successfully!\n");
    return true;
}

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

// Core 1: 通信・制御担当
void core1_entry(void) {
    // CANの初期化（リトライ付き）
    while (!can.init(CAN_1000KBPS)) {
        // 初期化失敗時のLED点滅のみ（printf削除）
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(10);
    }
    // 初期化成功はCore0で表示済み
    double target_current[4] = {0.0, 0.0, 0.0, 0.0};  // モータ1~4の目標電流値(A)

    // 制御周期管理構造体の初期化
    control_timing_t control_timing;
    init_control_timing(&control_timing);

    while (true) {
        // 制御周期開始処理
        control_timing_start(&control_timing, OVERFLOW_CONTINUOUS);

        // --- エンコーダ読み取り処理 ---
        double motor_position_R = 0.0, motor_position_P = 0.0;
        bool enc1_ok = encoder_manager.read_encoder(0);  // エンコーダ0
        bool enc2_ok = encoder_manager.read_encoder(1);  // エンコーダ1

        if (enc1_ok) {
            motor_position_R = encoder_manager.get_encoder_angle_rad(0) * ENCODER_R_DIRECTION;
        }
        if (enc2_ok) {
            // P軸はマルチターン対応エンコーダのため連続角度を使用
            motor_position_P = encoder_manager.get_encoder_continuous_angle_rad(1) * ENCODER_P_DIRECTION;
        }

        // --- モータフィードバック受信 ---
        double motor_velocity_R = 0.0, motor_velocity_P = 0.0;
        if (motor1.receive_feedback()) {
            // motor_position_R = motor1.get_continuous_angle() * MOTOR_R_DIRECTION; //  位置は外付けエンコーダの方を使う
            motor_velocity_R = motor1.get_angular_velocity();
        }
        if (motor2.receive_feedback()) {
            // motor_position_P = motor2.get_continuous_angle() * MOTOR_P_DIRECTION;
            motor_velocity_P = motor2.get_angular_velocity();
        }

        // --- 共有データから目標値取得 ---
        double target_pos_R, target_pos_P;
        mutex_enter_blocking(&g_state_mutex);
        target_pos_R = g_robot_state.target_position_R;
        target_pos_P = g_robot_state.target_position_P;

        // 現在状態を更新
        g_robot_state.current_position_R = motor_position_R;
        g_robot_state.current_position_P = motor_position_P;
        g_robot_state.current_velocity_R = motor_velocity_R;
        g_robot_state.current_velocity_P = motor_velocity_P;
        mutex_exit(&g_state_mutex);

        // --- 制御計算 ---
        // 位置PID制御（位置偏差 → 目標速度）
        double target_vel_R = position_pid_R.computePosition(target_pos_R, motor_position_R);
        double target_vel_P = position_pid_P.computePosition(target_pos_P, motor_position_P);

        // 速度I-P制御（速度偏差 → 目標トルク）
        double target_torque_R = velocity_ip_R.computeVelocity(target_vel_R, motor_velocity_R);
        double target_torque_P = velocity_ip_P.computeVelocity(target_vel_P, motor_velocity_P);

        // // トルクから電流への変換
        // target_current[0] = target_torque_R / dynamics_R.get_torque_constant();  // Motor1 (R軸)
        target_current[1] = target_torque_P / dynamics_P.get_torque_constant();  // Motor2 (P軸)
        target_current[0] = 0.0;                                                 // Motor1 (R軸)
        // target_current[1] = 0.0;  // Motor2 (P軸)

        // --- 制御結果を共有データに保存 ---
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.target_velocity_R = target_vel_R;
        g_robot_state.target_velocity_P = target_vel_P;
        g_robot_state.target_torque_R = target_torque_R;
        g_robot_state.target_torque_P = target_torque_P;
        g_robot_state.target_current_R = target_current[0];
        g_robot_state.target_current_P = target_current[1];
        g_robot_state.timing_violation_count = control_timing.timing_violation_count;
        g_robot_state.led_status = control_timing.led_mode;
        mutex_exit(&g_state_mutex);

        // --- CAN送信処理 ---
        if (!send_all_motor_currents(&can, target_current)) {
            // CAN送信失敗時のみエラーカウンタを更新
            mutex_enter_blocking(&g_state_mutex);
            g_robot_state.can_error_count++;
            mutex_exit(&g_state_mutex);
        }

        // 制御周期終了処理
        control_timing_end(&control_timing, CONTROL_PERIOD_MS);
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

    // PIDコントローラの初期化
    if (!init_pid_controllers()) {
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

    // 制御初期値
    g_robot_state.target_position_R = 0.0;  // 初期目標位置
    g_robot_state.target_position_P = 0.0;
    g_robot_state.current_position_R = 0.0;
    g_robot_state.current_position_P = 0.0;
    g_robot_state.current_velocity_R = 0.0;
    g_robot_state.current_velocity_P = 0.0;
    g_robot_state.target_velocity_R = 0.0;
    g_robot_state.target_velocity_P = 0.0;
    g_robot_state.target_torque_R = 0.0;
    g_robot_state.target_torque_P = 0.0;

    // デバッグ情報の初期化
    g_robot_state.target_current_R = 0.0;
    g_robot_state.target_current_P = 0.0;
    g_robot_state.timing_violation_count = 0;
    g_robot_state.led_status = LED_OFF;
    g_robot_state.can_error_count = 0;

    // Core1で実行する関数を起動
    multicore_launch_core1(core1_entry);

    // Core1の初期化完了を待つ
    sleep_ms(1000);
    printf("MCP25625 Initialized successfully!\n");
    printf("Starting control loop at %.1f ms (%.0f Hz)\n", CONTROL_PERIOD_MS, 1000.0 / CONTROL_PERIOD_MS);

    absolute_time_t next_main_time = get_absolute_time();

    while (1) {
        next_main_time = delayed_by_us(next_main_time, 100000);  // 100ms周期

        // 目標位置の変更テスト（ゆっくりとした目標値変化）
        static double time_counter = 0.0;
        time_counter += 0.1;  // 1秒ずつ増加
        // 10秒ごとに0度(0rad)と30度(約0.5236rad)を切り替える
        double target_R = (static_cast<int>(time_counter / 10.0) % 2 == 0) ? 2.16 : 3.6;       // 0 or 30度
        double target_P_m = (static_cast<int>(time_counter / 10.0) % 2 == 0) ? 1.022 : 0.533;  // 0mmまたは500mm -0.97 : 0.5
        double target_P = target_P_m / gear_radius_P;                                          // m単位からrad単位に変換

        // 状態を取得してデバッグ出力（排他制御あり）
        mutex_enter_blocking(&g_state_mutex);
        g_robot_state.target_position_R = target_R;
        g_robot_state.target_position_P = target_P;

        // デバッグ用に現在状態も取得
        double current_pos_R = g_robot_state.current_position_R;
        double current_pos_P = g_robot_state.current_position_P;
        double current_vel_R = g_robot_state.current_velocity_R;
        double current_vel_P = g_robot_state.current_velocity_P;
        double target_vel_R = g_robot_state.target_velocity_R;
        double target_vel_P = g_robot_state.target_velocity_P;
        double target_torque_R = g_robot_state.target_torque_R;
        double target_torque_P = g_robot_state.target_torque_P;
        double target_cur_R = g_robot_state.target_current_R;
        double target_cur_P = g_robot_state.target_current_P;
        int timing_violations = g_robot_state.timing_violation_count;
        led_mode_t led_status = g_robot_state.led_status;
        int can_errors = g_robot_state.can_error_count;
        mutex_exit(&g_state_mutex);

        // rad単位からm単位への変換
        double current_pos_P_m = current_pos_P * gear_radius_P;
        double current_vel_P_m = current_vel_P * gear_radius_P;
        double target_vel_P_m = target_vel_P * gear_radius_P;

        // 1秒毎のステータス出力（Core0で実行 - 重いprintf処理）
        printf("\n=== System Status (t=%.1fs) ===\n", time_counter);
        printf("Target:  R=%.3f [rad] (%.1f°), P=%.3f [m] (%.1f mm)\n", target_R, target_R * 180.0 / M_PI, target_P_m, target_P_m * 1000.0);
        printf("Current: R=%.3f [rad] (%.1f°), P=%.3f [m] (%.1f mm)\n", current_pos_R, current_pos_R * 180.0 / M_PI, current_pos_P_m, current_pos_P_m * 1000.0);
        printf("Velocity: R=%.2f [rad/s], P=%.2f [m/s] (%.1f mm/s)\n", current_vel_R, current_vel_P_m, current_vel_P_m * 1000.0);

        // P軸マルチターン情報
        int16_t p_turn_count = encoder_manager.get_encoder_turn_count(1);
        double p_single_angle = encoder_manager.get_encoder_angle_deg(1);
        printf("P-axis multiturn: %d turns, single angle: %.1f°, continuous: %.3f m (%.1f mm)\n",
               p_turn_count, p_single_angle, current_pos_P_m, current_pos_P_m * 1000.0);

        // 制御詳細情報
        printf("Control: PosR=%.3f->%.3f VelR=%.2f->%.2f TorqR=%.2f CurR=%.2fA [LED:%s]\n",
               current_pos_R, target_R, current_vel_R, target_vel_R,
               target_torque_R, target_cur_R, get_led_status_string(led_status));
        printf("         PosP=%.3f->%.3f VelP=%.2f->%.2f TorqP=%.2f CurP=%.2fA [Violations:%d]\n",
               current_pos_P_m, target_P_m, current_vel_P_m, target_vel_P_m,
               target_torque_P, target_cur_P, timing_violations);

        // エラー情報
        if (can_errors > 0) {
            printf("WARNING: CAN transmission errors: %d\n", can_errors);
        }

        busy_wait_until(next_main_time);  // 1秒待機
    }

    return 0;
}
