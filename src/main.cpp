#include <cmath>

#include "amt223v.hpp"
#include "config.hpp"
#include "control_timing.hpp"
#include "debug_manager.hpp"
#include "disturbance_observer.hpp"
#include "mcp25625.hpp"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "pid_controller.hpp"
#include "robomaster_motor.hpp"
#include "trajectory.hpp"
#include "trajectory_sequence_manager.hpp"

// 制御周期定数
constexpr float CONTROL_PERIOD_MS = 0.5;                        // 制御周期 [ms]
constexpr float CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0;  // 制御周期 [s]

// Core間同期設定
constexpr int SYNC_EVERY_N_LOOPS = 200;  // 200ループごとにCore0に同期信号を送信
constexpr uint32_t SYNC_SIGNAL = 1;      // 同期信号の値

// 軌道完了判定の許容誤差
constexpr float TRAJECTORY_COMPLETION_TOLERANCE_R = 0.1;         // R軸完了判定許容誤差 [rad] (約0.6度)
constexpr float TRAJECTORY_COMPLETION_TOLERANCE_P = 0.1;         // P軸完了判定許容誤差 [rad] (約12.5μm相当)
constexpr float TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD = 0.1;  // 完了判定時の速度閾値 [rad/s]

// 軌道データ配列設定
constexpr int MAX_TRAJECTORY_POINTS = 3000;         // 最大軌道点数
constexpr uint32_t TRAJECTORY_DATA_SIGNAL = 2;      // 軌道データ送信信号
constexpr uint32_t TRAJECTORY_COMPLETE_SIGNAL = 3;  // 軌道完了信号

// 軌道データ点の構造体（制御用の詳細軌道）
typedef struct {
    float position_R;      // R軸目標位置 [rad]
    float velocity_R;      // R軸目標速度 [rad/s]
    float acceleration_R;  // R軸目標加速度 [rad/s^2]
    float position_P;      // P軸目標位置 [rad]
    float velocity_P;      // P軸目標速度 [rad/s]
    float acceleration_P;  // P軸目標加速度 [rad/s^2]
} trajectory_point_t;

// 軌道データ管理構造体
typedef struct {
    trajectory_point_t points[MAX_TRAJECTORY_POINTS];
    int point_count;
    int current_index;
    bool active;
    bool complete;
    float final_target_R;   // 最終目標位置 R軸 [rad]
    float final_target_P;   // 最終目標位置 P軸 [rad]
    bool position_reached;  // 位置到達フラグ
} trajectory_data_t;

// システム設定定数
constexpr int SHUTDOWN_PIN = 27;  // 明示的にLOWにしないとPicoが動かない

// モータとエンコーダの符号補正設定
constexpr float ENCODER_R_DIRECTION = 1.0;  // R軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で右回り、右回りでエンコーダ値が増加
constexpr float ENCODER_P_DIRECTION = 1.0;  // P軸エンコーダの増加方向補正 (+1.0 or -1.0) 正入力で根本方向、根本方向でエンコーダ値が減少

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
    .baudrate = 4'000'000,  // 1MHz
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
AMT223V_Manager encoder_manager(spi1,       // SPI0を使用
                                1'875'000,  // 規定値 2MHz 整数分数でベスト:1.875MHz
                                8,          // MISO pin
                                10,         // SCK pin
                                11);        // MOSI pin

// モータの出力軸から機構の出力軸までのギア比
constexpr float gear_ratio_R = 3.0;     // M3508出力軸からベース根本(3.0)
constexpr float gear_ratio_P = 1.0;     // M2006 P36出力軸からラックまで(ギアなし)
constexpr float gear_radius_P = 0.025;  // ギアの半径 (m) - M2006の出力軸からラックまでの距離が25mm

// R軸（ベース回転）の動力学パラメータ（定数で表現）
constexpr float R_EQ_INERTIA = 0.3279f;                   // 等価慣性モーメント (kg·m^2)
constexpr float R_EQ_DAMPING = 0.4084f;                   // 等価粘性摩擦係数 (N·m·s/rad)
constexpr float R_TORQUE_CONSTANT = 0.3f * gear_ratio_R;  // 等価トルク定数（M3508のトルク定数xギア比） (Nm/A)

// P軸（アーム直動）の動力学パラメータ（定数で表現）
constexpr float P_EQ_INERTIA = 0.00448f;                   // 等価慣性モーメント (kg·m^2)
constexpr float P_EQ_DAMPING = 0.00785f;                   // 粘性摩擦係数 (N·m·s/rad)
constexpr float P_TORQUE_CONSTANT = 0.18f * gear_ratio_P;  // 等価トルク定数（M2006のトルク定数xギア比） (Nm/A)

// 軌道生成と制御器で共通の制限定数
namespace TrajectoryLimits {
constexpr float R_MAX_VELOCITY = 482.0 / 60.0 * 2.0 * M_PI / gear_ratio_R;  // R軸最大速度制限 [rad/s] 無負荷回転数482rpm
constexpr float P_MAX_VELOCITY = 416.0 / 60.0 * 2.0 * M_PI / gear_ratio_P;  // P軸最大速度制限 [rad/s] Maximum speed at 1N•m: 416 rpm

// 動力学パラメータとトルク制限から計算した最大角加速度
constexpr float R_MAX_TORQUE = std::min(3.0f, 1.0f) * gear_ratio_R;  // R軸最大トルク制限 [Nm] (M3508最大連続トルク 3.0Nm と カップリング最大トルク 1.0Nm の最小値)
constexpr float P_MAX_TORQUE = 1.0f * gear_ratio_P;                  // P軸最大トルク制限 [Nm] (M2006最大連続トルク 1.0Nm)
constexpr float R_MAX_ACCELERATION = R_MAX_TORQUE / R_EQ_INERTIA;    // R軸最大角加速度 [rad/s^2] 最大トルク
constexpr float P_MAX_ACCELERATION = P_MAX_TORQUE / P_EQ_INERTIA;    // P軸最大角加速度 [rad/s^2] 最大トルク
}  // namespace TrajectoryLimits

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, gear_ratio_P);  // motor_id=2

// 外乱オブザーバのパラメータ
constexpr float R_CUTOFF_FREQ = 6.0f;                                             // R軸 外乱オブザーバのカットオフ周波数 [rad/s]
constexpr float sqrtf_R_POSITION_GAIN = 7.0f;                                     // R軸 外乱オブザーバの位置ゲインの平方根
constexpr float R_POSITION_GAIN = sqrtf_R_POSITION_GAIN * sqrtf_R_POSITION_GAIN;  // R軸 外乱オブザーバの位置ゲイン
constexpr float R_VELOCITY_GAIN = 2.0f * sqrtf_R_POSITION_GAIN;                   // R軸 外乱オブザーバの速度ゲイン
constexpr float P_CUTOFF_FREQ = 6.0f;                                             // P軸 外乱オブザーバのカットオフ周波数 [rad/s]
constexpr float sqrtf_P_POSITION_GAIN = 7.0f;                                     // P軸 外乱オブザーバの位置ゲインの平方根
constexpr float P_POSITION_GAIN = sqrtf_P_POSITION_GAIN * sqrtf_P_POSITION_GAIN;  // P軸 外乱オブザーバの位置ゲイン
constexpr float P_VELOCITY_GAIN = 2.0f * sqrtf_P_POSITION_GAIN;                   // P軸 外乱オブザーバの速度ゲイン

// 制御器の制限値設定
namespace ControlLimits {
// R軸（ベース回転）制御制限
namespace R_Axis {
constexpr float MAX_VELOCITY = 0.7 * TrajectoryLimits::R_MAX_VELOCITY;       // 位置PID出力の最大速度制限 [rad/s] - 軌道生成より小さく設定
constexpr float INTEGRAL_VELOCITY = 0.6 * TrajectoryLimits::R_MAX_VELOCITY;  // 位置PID積分制限 [rad/s] - 最大角速度の60%に設定
constexpr float MAX_TORQUE = TrajectoryLimits::R_MAX_TORQUE;                 // 速度I-P出力の最大トルク制限 [Nm] - 軌道生成と共通
constexpr float INTEGRAL_TORQUE = 0.8 * TrajectoryLimits::R_MAX_TORQUE;      // 速度I-P積分制限 [Nm] 最大トルクの80%に設定
}  // namespace R_Axis

// P軸（アーム直動）制御制限
namespace P_Axis {
constexpr float MAX_VELOCITY = 0.7 * TrajectoryLimits::P_MAX_VELOCITY;       // 位置PID出力の最大速度制限 [rad/s] - 軌道生成より小さく設定
constexpr float INTEGRAL_VELOCITY = 0.6 * TrajectoryLimits::P_MAX_VELOCITY;  // 位置PID積分制限 [rad/s]
constexpr float MAX_TORQUE = TrajectoryLimits::P_MAX_TORQUE;                 // 速度I-P出力の最大トルク制限 [Nm] - 軌道生成と共通
constexpr float INTEGRAL_TORQUE = 0.3 * TrajectoryLimits::P_MAX_TORQUE;      // 速度I-P積分制限 [Nm]
}  // namespace P_Axis
}  // namespace ControlLimits

float clampTorque(float torque, float max_torque) {
    if (torque > max_torque) {
        return max_torque;
    } else if (torque < -max_torque) {
        return -max_torque;
    }
    return torque;
}

// 共有データ構造体
typedef struct
{
    int motor_speed;
    int sensor_value;

    // 制御状態
    float current_position_R;  // R軸現在位置 [rad]
    float current_position_P;  // P軸現在位置 [rad]
    float current_velocity_R;  // R軸現在速度 [rad/s]
    float current_velocity_P;  // P軸現在速度 [rad/s]

    // 制御出力
    float target_velocity_R;  // R軸目標速度 [rad/s]（位置PIDの出力）
    float target_velocity_P;  // P軸目標速度 [rad/s]（位置PIDの出力）
    float target_torque_R;    // R軸目標トルク [Nm]（速度I-Pの出力）
    float target_torque_P;    // P軸目標トルク [Nm]（速度I-Pの出力）

    // エンコーダ詳細情報（Core1→Core0）
    int16_t encoder_p_turn_count;          // P軸エンコーダ回転回数
    float encoder_p_single_angle_deg;      // P軸エンコーダ単回転角度[deg]
    float encoder_p_continuous_angle_rad;  // P軸エンコーダ連続角度[rad]
    float encoder_r_angle_deg;             // R軸エンコーダ角度[deg]
    bool encoder_r_valid;                  // R軸エンコーダデータ有効性
    bool encoder_p_valid;                  // P軸エンコーダデータ有効性

    // デバッグ・制御タイミング情報
    float target_current_R;      // R軸目標電流 [A]
    float target_current_P;      // P軸目標電流 [A]
    int timing_violation_count;  // 制御周期違反回数
    led_mode_t led_status;       // LED状態
    int can_error_count;         // CAN送信エラー回数

    float current_time;  // 現在時刻 [s]
} robot_state_t;

// 共有状態とミューテックス
static robot_state_t g_robot_state;
static mutex_t g_state_mutex;

// 軌道データとミューテックス
static trajectory_data_t g_trajectory_data;
static mutex_t g_trajectory_mutex;

// グローバルデバッグマネージャ
static DebugManager* g_debug_manager = nullptr;

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
        g_debug_manager->info("CS%d pin %d initialized\n", i, config->pin_cs[i]);
    }

    // リセットピンがある場合の設定
    if (config->pin_rst >= 0) {
        gpio_init(config->pin_rst);
        gpio_set_dir(config->pin_rst, GPIO_OUT);
        gpio_put(config->pin_rst, 1);  // リセット解除
        g_debug_manager->info("Reset pin %d initialized\n", config->pin_rst);
    }

    return true;
}

// 複数のSPIデバイスを初期化する関数
bool init_all_spi_devices() {
    // CAN IC用SPI初期化
    if (!init_spi(&can_spi_config)) {
        g_debug_manager->error("Failed to initialize CAN SPI!\n");
        return false;
    }
    g_debug_manager->info("CAN SPI initialized successfully!\n");

    return true;
}

// エンコーダの初期化関数
bool init_encoders() {
    static int encoder1_index = -1;  // R軸エンコーダのインデックス
    static int encoder2_index = -1;  // P軸エンコーダのインデックス

    // エンコーダが未登録の場合のみ追加
    if (encoder_manager.get_current_encoder_count() == 0) {
        g_debug_manager->info("Adding encoders to manager...\n");
        // エンコーダを追加
        encoder1_index = encoder_manager.add_encoder(7, false);  // CS pin 7 (R軸: 単回転)
        encoder2_index = encoder_manager.add_encoder(6, true);   // CS pin 6 (P軸: マルチターン対応)

        if (encoder1_index < 0 || encoder2_index < 0) {
            g_debug_manager->error("Failed to add encoders!\n");
            return false;
        }
        g_debug_manager->info("Encoders added successfully: R=%d, P=%d\n", encoder1_index, encoder2_index);
    }

    // SPI初期化
    if (!encoder_manager.init_spi()) {
        g_debug_manager->error("Failed to initialize encoder SPI!\n");
        return false;
    }

    // 全エンコーダ初期化（安定化時間を含む）
    g_debug_manager->info("Starting encoder initialization with extended stabilization...\n");
    if (!encoder_manager.init_all_encoders()) {
        g_debug_manager->error("Failed to initialize encoders!\n");
        return false;
    }

    g_debug_manager->info("All encoders initialized successfully!\n");
    return true;
}

// PIDコントローラの初期化関数
bool init_pid_controllers() {
    g_debug_manager->info("Initializing PID controllers...\n");
    g_debug_manager->info("Control period: %.1f ms (%.0f Hz)\n", CONTROL_PERIOD_MS, 1000.0 / CONTROL_PERIOD_MS);

    // 方向補正設定の表示
    g_debug_manager->info("Direction correction settings:\n");
    g_debug_manager->info("  Encoder R direction: %+.1f\n", ENCODER_R_DIRECTION);
    g_debug_manager->info("  Encoder P direction: %+.1f\n", ENCODER_P_DIRECTION);

    g_debug_manager->info("PID controllers initialized successfully!\n");

    // 制限値設定の表示
    g_debug_manager->info("\n=== Control Limits Configuration ===\n");
    g_debug_manager->info("R-Axis Limits:\n");
    g_debug_manager->info("  Position PID Output: ±%.1f rad/s, Integral: ±%.1f rad/s\n",
                          ControlLimits::R_Axis::MAX_VELOCITY, ControlLimits::R_Axis::INTEGRAL_VELOCITY);
    g_debug_manager->info("  Velocity I-P Output: ±%.1f Nm, Integral: ±%.1f Nm\n",
                          ControlLimits::R_Axis::MAX_TORQUE, ControlLimits::R_Axis::INTEGRAL_TORQUE);

    g_debug_manager->info("P-Axis Limits:\n");
    g_debug_manager->info("  Position PID Output: ±%.1f rad/s, Integral: ±%.1f rad/s\n",
                          ControlLimits::P_Axis::MAX_VELOCITY, ControlLimits::P_Axis::INTEGRAL_VELOCITY);
    g_debug_manager->info("  Velocity I-P Output: ±%.1f Nm, Integral: ±%.1f Nm\n",
                          ControlLimits::P_Axis::MAX_TORQUE, ControlLimits::P_Axis::INTEGRAL_TORQUE);

    g_debug_manager->info("FeedForward Gains:\n");
    g_debug_manager->info("  R-Velocity FF: %.1f, P-Velocity FF: %.1f\n",
                          R_VELOCITY_GAIN,
                          P_VELOCITY_GAIN);

    return true;
}

// Core0用軌道計算関数
bool calculate_trajectory_core0(float current_pos_R, float current_pos_P, float target_pos_R, float target_pos_P) {
    // Core0専用の軌道生成インスタンス
    trajectory_t trajectory_R_core0(
        0.15 * TrajectoryLimits::R_MAX_VELOCITY,
        0.95 * TrajectoryLimits::R_MAX_ACCELERATION,
        current_pos_R, target_pos_R);
    trajectory_t trajectory_P_core0(
        0.15 * TrajectoryLimits::P_MAX_VELOCITY,
        0.9 * TrajectoryLimits::P_MAX_ACCELERATION,
        current_pos_P, target_pos_P);

    // 軌道パラメータを計算
    trajectory_R_core0.calculate_trapezoidal_params();
    trajectory_P_core0.calculate_trapezoidal_params();

    // より長い軌道時間を取得
    float total_time_R = trajectory_R_core0.get_total_time();
    float total_time_P = trajectory_P_core0.get_total_time();
    float max_time = std::max(total_time_R, total_time_P);

    // 軌道点数を計算（制御周期ベース）
    int point_count = static_cast<int>(max_time / CONTROL_PERIOD_S) + 1;
    if (point_count > MAX_TRAJECTORY_POINTS) {
        g_debug_manager->error("Trajectory too long: %d points (max: %d)", point_count, MAX_TRAJECTORY_POINTS);
        return false;
    }

    // 軌道データを計算して配列に格納
    mutex_enter_blocking(&g_trajectory_mutex);
    g_trajectory_data.point_count = point_count;
    g_trajectory_data.current_index = 0;
    g_trajectory_data.active = false;
    g_trajectory_data.complete = false;
    g_trajectory_data.final_target_R = target_pos_R;  // 最終目標位置を保存
    g_trajectory_data.final_target_P = target_pos_P;
    g_trajectory_data.position_reached = false;

    for (int i = 0; i < point_count; i++) {
        float time = i * CONTROL_PERIOD_S;

        // R軸の軌道状態を計算
        float pos_R, vel_R, accel_R;
        trajectory_R_core0.get_trapezoidal_state(time, &pos_R, &vel_R, &accel_R);

        // P軸の軌道状態を計算
        float pos_P, vel_P, accel_P;
        trajectory_P_core0.get_trapezoidal_state(time, &pos_P, &vel_P, &accel_P);

        // 配列に格納
        g_trajectory_data.points[i].position_R = pos_R;
        g_trajectory_data.points[i].velocity_R = vel_R;
        g_trajectory_data.points[i].acceleration_R = accel_R;
        g_trajectory_data.points[i].position_P = pos_P;
        g_trajectory_data.points[i].velocity_P = vel_P;
        g_trajectory_data.points[i].acceleration_P = accel_P;
    }
    mutex_exit(&g_trajectory_mutex);

    g_debug_manager->debug("Trajectory calculated: %d points, max_time=%.2fs", point_count, max_time);
    g_debug_manager->debug("  R: %.3f → %.3f rad, P: %.3f → %.3f rad",
                           current_pos_R, target_pos_R, current_pos_P, target_pos_P);

    return true;
}

void init_hand() {
    // ポンプの設定
    gpio_init(PUMP_PIN);
    gpio_set_dir(PUMP_PIN, GPIO_OUT);
    gpio_init(SOLENOID_PIN);
    gpio_set_dir(SOLENOID_PIN, GPIO_OUT);

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    g_debug_manager->info("Initializing Dynamixels (Daisy Chain on UART0)...\n");
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    sleep_ms(1000);
    write_statusReturnLevel(&UART0, DXL_ID1, 0x00);
    write_statusReturnLevel(&UART0, DXL_ID2, 0x00);
    sleep_ms(1000);
    write_dxl_led(&UART0, DXL_ID1, true);
    write_dxl_led(&UART0, DXL_ID2, true);
    sleep_ms(1000);
    write_dxl_led(&UART0, DXL_ID1, false);
    write_dxl_led(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, DXL_ID1, false);
    write_torqueEnable(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_operatingMode(&UART0, DXL_ID1, false);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART0, DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, DXL_ID1, true);
    write_torqueEnable(&UART0, DXL_ID2, true);
    sleep_ms(1000);
    gpio_put(SOLENOID_PIN, 0);  // ソレノイドを吸着状態にする
    gpio_put(PUMP_PIN, 1);
    g_debug_manager->info("hand initialized\n");
    sleep_ms(500);
    control_position(&UART0, DXL_ID1, START_HAND_ANGLE);
    sleep_ms(500);
    control_position(&UART0, DXL_ID2, START_UP_ANGLE);
}

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, bool* has_work, absolute_time_t* state_start_time) {
    uint32_t elapsed_ms = absolute_time_diff_us(*state_start_time, get_absolute_time()) / 1000;
    switch (*hand_state) {
        case HAND_IDLE:
            g_debug_manager->debug("hand requested\n");
            if (!*has_work) {
                *hand_state = HAND_LOWERING;
                *state_start_time = get_absolute_time();
                gpio_put(PUMP_PIN, 1);
                g_debug_manager->debug("Hand lowering...");
                control_position(&UART0, DXL_ID2, DOWN_ANGLE);
            } else {
                *hand_state = HAND_RELEASE;
                *state_start_time = get_absolute_time();
                gpio_put(SOLENOID_PIN, 1);
            }
            break;

        case HAND_LOWERING:
            if (elapsed_ms >= 100) {
                *hand_state = HAND_SUCTION_WAIT;
                *state_start_time = get_absolute_time();
                g_debug_manager->debug("Hand suction wait...\n");
            }
            break;

        case HAND_SUCTION_WAIT:
            if (elapsed_ms >= 100) {
                *hand_state = HAND_RAISING;
                *state_start_time = get_absolute_time();
                control_position(&UART0, DXL_ID2, UP_ANGLE);
                g_debug_manager->debug("Hand raising...\n");
            }
            break;

        case HAND_RAISING:
            if (elapsed_ms >= 100) {
                *has_work = true;
                control_position(&UART0, DXL_ID1, RELEASE_ANGLE);
                g_debug_manager->debug("Hand raised, work done.\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
            }
            break;

        case HAND_RELEASE:
            if (elapsed_ms >= 100) {
                *has_work = false;
                gpio_put(SOLENOID_PIN, 0);
                control_position(&UART0, DXL_ID1, GRAB_ANGLE);
                g_debug_manager->debug("Hand released\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
            }
            break;

        case HAND_WAITING:
            if (elapsed_ms >= 100) {
                *hand_state = HAND_IDLE;
            }
            break;
    }
}

// デバッグ用ユーティリティ関数: 軌道目標値を安全に取得する共通関数
void get_safe_trajectory_targets(float current_pos_R, float current_pos_P,
                                 float* traj_pos_R, float* traj_pos_P,
                                 float* traj_vel_R, float* traj_vel_P) {
    // mutex_enter_blocking(&g_trajectory_mutex); // 呼び出し元でミューテックス取得済み

    if (g_trajectory_data.active && g_trajectory_data.current_index < g_trajectory_data.point_count) {
        // 軌道実行中
        int idx = g_trajectory_data.current_index;
        *traj_pos_R = g_trajectory_data.points[idx].position_R;
        *traj_pos_P = g_trajectory_data.points[idx].position_P;
        *traj_vel_R = g_trajectory_data.points[idx].velocity_R;
        *traj_vel_P = g_trajectory_data.points[idx].velocity_P;
    } else if (g_trajectory_data.current_index >= g_trajectory_data.point_count &&
               g_trajectory_data.point_count > 0) {
        // 軌道データ終了後
        *traj_pos_R = g_trajectory_data.final_target_R;
        *traj_pos_P = g_trajectory_data.final_target_P;
        *traj_vel_R = 0.0;
        *traj_vel_P = 0.0;
    } else {
        // 軌道停止時
        *traj_pos_R = current_pos_R;
        *traj_pos_P = current_pos_P;
        *traj_vel_R = 0.0;
        *traj_vel_P = 0.0;
    }
}

// Core 1: 通信・制御担当
void core1_entry(void) {
    // CANの初期化（リトライ付き）
    while (!can.init(CAN_1000KBPS)) {
        // 初期化失敗時のLED点滅
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(10);
    }
    // 初期化成功はCore0で表示済み
    float target_current[4] = {0.0, 0.0, 0.0, 0.0};  // モータ1~4の目標電流値(A)

    // 制御周期管理構造体の初期化
    control_timing_t control_timing;
    init_control_timing(&control_timing);

    // 制御開始時刻を記録
    absolute_time_t control_start_time = get_absolute_time();

    float disturbance_torque_R = 0.0f;                                            // R軸の外乱トルク
    float control_torque_R = 0.0f;                                                // R軸の制御トルク
    float target_torque_R = 0.0f;                                                 // R軸の目標トルク
    float error_position_R = 0.0f;                                                // R軸の位置誤差
    float error_velocity_R = 0.0f;                                                // R軸の速度誤差
    float acceleration_feedforward_R = 0.0f;                                      // R軸の加速度フィードフォワード
    disturbance_observer_t dob_R(R_EQ_INERTIA, R_CUTOFF_FREQ, CONTROL_PERIOD_S);  // R軸の外乱オブザーバ

    float disturbance_torque_P = 0.0f;                                            // P軸の外乱トルク
    float control_torque_P = 0.0f;                                                // P軸の制御トルク
    float target_torque_P = 0.0f;                                                 // P軸の目標トルク
    float error_position_P = 0.0f;                                                // P軸の位置誤差
    float error_velocity_P = 0.0f;                                                // P軸の速度誤差
    float acceleration_feedforward_P = 0.0f;                                      // P軸の加速度フィードフォワード
    disturbance_observer_t dob_P(P_EQ_INERTIA, P_CUTOFF_FREQ, CONTROL_PERIOD_S);  // P軸の外乱オブザーバ

    // ループカウンタの初期化
    int loop_counter = 0;

    while (true) {
        // 制御周期開始処理
        control_timing_start(&control_timing, CONTROL_PERIOD_MS);

        // FIFOから軌道開始信号をチェック（ノンブロッキング）
        uint32_t trajectory_signal;
        if (multicore_fifo_pop_timeout_us(0, &trajectory_signal)) {
            if (trajectory_signal == TRAJECTORY_DATA_SIGNAL) {
                // 軌道データの実行を開始
                mutex_enter_blocking(&g_trajectory_mutex);
                g_trajectory_data.active = true;
                g_trajectory_data.current_index = 0;
                g_trajectory_data.complete = false;
                g_trajectory_data.position_reached = false;  // 位置到達フラグをリセット
                mutex_exit(&g_trajectory_mutex);
            }
        }

        // 経過時間を秒単位に変換
        float current_time_s = absolute_time_diff_us(control_start_time, control_timing.loop_start_time) / 1000000.0f;

        // --- エンコーダ読み取り処理 ---
        float motor_position_R = 0.0, motor_position_P = 0.0;
        float motor_velocity_R = 0.0, motor_velocity_P = 0.0;
        bool enc1_ok = encoder_manager.read_encoder(0);  // エンコーダ0 (R軸)
        bool enc2_ok = encoder_manager.read_encoder(1);  // エンコーダ1 (P軸)

        // エンコーダデータの取得
        float encoder_r_angle_deg = 0.0;
        int16_t encoder_p_turn_count = 0;
        float encoder_p_single_angle_deg = 0.0;
        float encoder_p_continuous_angle_rad = 0.0;

        if (enc1_ok) {
            motor_position_R = encoder_manager.get_encoder_angle_rad(0) * ENCODER_R_DIRECTION;
            motor_velocity_R = encoder_manager.get_encoder_angular_velocity_rad(0) * ENCODER_R_DIRECTION;
            encoder_r_angle_deg = encoder_manager.get_encoder_angle_deg(0);
        }

        if (enc2_ok) {
            // P軸はマルチターン対応エンコーダのため連続角度を使用
            motor_position_P = encoder_manager.get_encoder_continuous_angle_rad(1) * ENCODER_P_DIRECTION;
            motor_velocity_P = encoder_manager.get_encoder_angular_velocity_rad(1) * ENCODER_P_DIRECTION;

            // デバッグ用エンコーダ情報を取得
            encoder_p_turn_count = encoder_manager.get_encoder_turn_count(1);
            encoder_p_single_angle_deg = encoder_manager.get_encoder_angle_deg(1);
            encoder_p_continuous_angle_rad = encoder_manager.get_encoder_continuous_angle_rad(1);
        }

        // // --- モータフィードバック受信 ---
        // float motor_velocity_R = 0.0, motor_velocity_P = 0.0;
        // if (motor1.receive_feedback()) {
        //     // motor_position_R = motor1.get_continuous_angle() * MOTOR_R_DIRECTION; //  位置は外付けエンコーダの方を使う
        //     motor_velocity_R = motor1.get_angular_velocity();
        // }
        // if (motor2.receive_feedback()) {
        //     // motor_position_P = motor2.get_continuous_angle() * MOTOR_P_DIRECTION;
        //     motor_velocity_P = motor2.get_angular_velocity();
        // }

        // --- 共有データから目標値取得と状態更新 ---

        mutex_enter_blocking(&g_state_mutex);
        // 現在時刻を更新
        g_robot_state.current_time = current_time_s;

        // 現在状態を更新
        g_robot_state.current_position_R = motor_position_R;
        g_robot_state.current_position_P = motor_position_P;
        g_robot_state.current_velocity_R = motor_velocity_R;
        g_robot_state.current_velocity_P = motor_velocity_P;

        // エンコーダ詳細情報を更新
        g_robot_state.encoder_r_angle_deg = encoder_r_angle_deg;
        g_robot_state.encoder_p_turn_count = encoder_p_turn_count;
        g_robot_state.encoder_p_single_angle_deg = encoder_p_single_angle_deg;
        g_robot_state.encoder_p_continuous_angle_rad = encoder_p_continuous_angle_rad;
        g_robot_state.encoder_r_valid = enc1_ok;
        g_robot_state.encoder_p_valid = enc2_ok;

        mutex_exit(&g_state_mutex);

        // --- 軌道データベースの制御計算 ---
        float trajectory_target_pos_R = motor_position_R;  // デフォルトは現在位置
        float trajectory_target_pos_P = motor_position_P;
        float trajectory_target_vel_R = 0.0;
        float trajectory_target_vel_P = 0.0;
        float trajectory_target_accel_R = 0.0;
        float trajectory_target_accel_P = 0.0;

        // 軌道データから目標値を取得
        mutex_enter_blocking(&g_trajectory_mutex);
        bool trajectory_completed = false;
        if (g_trajectory_data.active) {
            if (g_trajectory_data.current_index < g_trajectory_data.point_count) {
                int idx = g_trajectory_data.current_index;
                trajectory_target_pos_R = g_trajectory_data.points[idx].position_R;
                trajectory_target_vel_R = g_trajectory_data.points[idx].velocity_R;
                trajectory_target_accel_R = g_trajectory_data.points[idx].acceleration_R;
                trajectory_target_pos_P = g_trajectory_data.points[idx].position_P;
                trajectory_target_vel_P = g_trajectory_data.points[idx].velocity_P;
                trajectory_target_accel_P = g_trajectory_data.points[idx].acceleration_P;

                // インデックスを進める
                g_trajectory_data.current_index++;

                // 軌道の時系列が完了した場合、位置ベースの完了判定に移行
            } else if (g_trajectory_data.current_index >= g_trajectory_data.point_count) {
                // 最終目標位置を設定し、位置到達判定モードに移行
                trajectory_target_pos_R = g_trajectory_data.final_target_R;
                trajectory_target_pos_P = g_trajectory_data.final_target_P;
                trajectory_target_vel_R = 0.0;
                trajectory_target_vel_P = 0.0;
                trajectory_target_accel_R = 0.0;
                trajectory_target_accel_P = 0.0;

                // 位置到達判定
                float position_error_R = std::abs(g_trajectory_data.final_target_R - motor_position_R);
                float position_error_P = std::abs(g_trajectory_data.final_target_P - motor_position_P);
                float velocity_magnitude_R = std::abs(motor_velocity_R);
                float velocity_magnitude_P = std::abs(motor_velocity_P);

                // 位置誤差と速度が両方とも許容範囲内の場合に完了とする
                if (position_error_R < TRAJECTORY_COMPLETION_TOLERANCE_R &&
                    position_error_P < TRAJECTORY_COMPLETION_TOLERANCE_P &&
                    velocity_magnitude_R < TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD &&
                    velocity_magnitude_P < TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD) {
                    g_trajectory_data.active = false;
                    g_trajectory_data.complete = true;
                    g_trajectory_data.position_reached = true;
                    trajectory_completed = true;
                }
            }
        } else if (!g_trajectory_data.active) {
            // 軌道停止時は現在位置を保持
            trajectory_target_pos_R = motor_position_R;
            trajectory_target_vel_R = 0.0;
            trajectory_target_accel_R = 0.0;
            trajectory_target_pos_P = motor_position_P;
            trajectory_target_vel_P = 0.0;
            trajectory_target_accel_P = 0.0;
        }
        mutex_exit(&g_trajectory_mutex);

        // 軌道完了信号の送信（mutex外で実行）
        if (trajectory_completed) {
            if (!multicore_fifo_push_timeout_us(TRAJECTORY_COMPLETE_SIGNAL, 0)) {
                // FIFO満杯の場合は次回再試行
            }
        }

        // --- 制御計算 ---
        // R軸の制御計算
        error_position_R = R_EQ_INERTIA * R_POSITION_GAIN * (trajectory_target_pos_R - motor_position_R);
        error_velocity_R = R_EQ_INERTIA * R_VELOCITY_GAIN * (trajectory_target_vel_R - motor_velocity_R);
        acceleration_feedforward_R = R_EQ_INERTIA * trajectory_target_accel_R;
        control_torque_R = error_position_R + error_velocity_R + disturbance_torque_R + acceleration_feedforward_R;  // 制御トルク計算
        target_torque_R = clampTorque(control_torque_R, ControlLimits::R_Axis::MAX_TORQUE);                          // 制御トルク制限
        control_torque_R = target_torque_R;                                                                          // 制御トルクを目標トルクに設定
        disturbance_torque_R = dob_R.update(control_torque_R, motor_velocity_R);                                     // 外乱トルクの更新

        // P軸の制御計算
        error_position_P = P_EQ_INERTIA * P_POSITION_GAIN * (trajectory_target_pos_P - motor_position_P);
        error_velocity_P = P_EQ_INERTIA * P_VELOCITY_GAIN * (trajectory_target_vel_P - motor_velocity_P);
        acceleration_feedforward_P = P_EQ_INERTIA * trajectory_target_accel_P;
        control_torque_P = error_position_P + error_velocity_P + disturbance_torque_P + acceleration_feedforward_P;
        target_torque_P = clampTorque(control_torque_P, ControlLimits::P_Axis::MAX_TORQUE);
        control_torque_P = target_torque_P;
        disturbance_torque_P = dob_P.update(control_torque_P, motor_velocity_P);  // 外乱トルクの更新

        // // トルクから電流への変換
        target_current[0] = target_torque_R / R_TORQUE_CONSTANT;  // Motor1 (R軸)
        target_current[1] = target_torque_P / P_TORQUE_CONSTANT;  // Motor2 (P軸)
        // target_current[0] = 0.0;                                  // Motor1 (R軸)
        // target_current[1] = 0.0;                                  // Motor2 (P軸)

        // --- 制御結果を共有データに保存 ---
        mutex_enter_blocking(&g_state_mutex);
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

        // ループカウンタを更新し、指定回数に達したらCore0に同期信号を送信
        loop_counter++;
        if (loop_counter >= SYNC_EVERY_N_LOOPS) {
            loop_counter = 0;  // カウンタをリセット

            // FIFOに同期信号を送信（ノンブロッキング）
            if (!multicore_fifo_push_timeout_us(SYNC_SIGNAL, 0)) {
                // FIFO満杯の場合は何もしない（次回再試行）
                g_debug_manager->error("Core1: Failed to push sync signal to Core0 FIFO");
            }
        }

        // 制御周期終了処理
        control_timing_end(&control_timing, CONTROL_PERIOD_MS);
    }
}

// システム初期化関数
bool initialize_system() {
    stdio_init_all();  // UARTなど初期化
    gpio_init(SHUTDOWN_PIN);
    gpio_set_dir(SHUTDOWN_PIN, GPIO_OUT);
    gpio_put(SHUTDOWN_PIN, 0);  // HIGHにしておくとPicoが動かないのでLOWに設定
    sleep_ms(2000);             // 少し待機して安定化

    // デバッグマネージャの初期化
    g_debug_manager = new DebugManager(DebugLevel::ERROR, 0.1f);

    // 全SPIデバイスの初期化
    while (!init_all_spi_devices()) {
        g_debug_manager->error("SPI initialization failed, retrying...");
        sleep_ms(1000);
    }

    // エンコーダの初期化
    while (!init_encoders()) {
        g_debug_manager->error("Encoder initialization failed, retrying...");
        sleep_ms(1000);
    }

    // PIDコントローラの初期化
    while (!init_pid_controllers()) {
        g_debug_manager->error("PID controller initialization failed, retrying...");
        sleep_ms(1000);
    }

    // ハンドの初期化
    init_hand();

    sleep_ms(2000);  // シリアル接続待ち

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // ミューテックス初期化
    mutex_init(&g_state_mutex);
    mutex_init(&g_trajectory_mutex);

    // 共有変数初期化
    g_robot_state.motor_speed = 0;
    g_robot_state.sensor_value = 0;

    // 軌道データの初期化
    g_trajectory_data.point_count = 0;
    g_trajectory_data.current_index = 0;
    g_trajectory_data.active = false;
    g_trajectory_data.complete = false;
    g_trajectory_data.final_target_R = 0.0;
    g_trajectory_data.final_target_P = 0.0;
    g_trajectory_data.position_reached = false;

    // 制御初期値
    g_robot_state.current_position_R = 0.0;
    g_robot_state.current_position_P = 0.0;
    g_robot_state.current_velocity_R = 0.0;
    g_robot_state.current_velocity_P = 0.0;
    g_robot_state.target_velocity_R = 0.0;
    g_robot_state.target_velocity_P = 0.0;
    g_robot_state.target_torque_R = 0.0;
    g_robot_state.target_torque_P = 0.0;

    // エンコーダ詳細情報の初期化
    g_robot_state.encoder_p_turn_count = 0;
    g_robot_state.encoder_p_single_angle_deg = 0.0;
    g_robot_state.encoder_p_continuous_angle_rad = 0.0;
    g_robot_state.encoder_r_angle_deg = 0.0;
    g_robot_state.encoder_r_valid = false;
    g_robot_state.encoder_p_valid = false;

    // 現在時間の初期化
    g_robot_state.current_time = 0.0;

    // デバッグ情報の初期化
    g_robot_state.target_current_R = 0.0;
    g_robot_state.target_current_P = 0.0;
    g_robot_state.timing_violation_count = 0;
    g_robot_state.led_status = LED_OFF;
    g_robot_state.can_error_count = 0;

    return true;
}

int main(void) {
    // 初期化処理をまとめて呼び出す
    if (!initialize_system()) {
        // 初期化失敗時の処理（必要ならエラー表示や停止）
        return -1;
    }

    // Core1で実行する関数を起動
    multicore_launch_core1(core1_entry);

    // Core1の初期化完了を待つ
    sleep_ms(1000);
    g_debug_manager->info("MCP25625 Initialized successfully!");
    g_debug_manager->info("Starting control loop at %.1f ms (%.0f Hz)", CONTROL_PERIOD_MS, 1000.0 / CONTROL_PERIOD_MS);
    g_debug_manager->info("Core sync enabled: Core0 processes every %d Core1 loops (%.1f ms)", SYNC_EVERY_N_LOOPS, CONTROL_PERIOD_MS * SYNC_EVERY_N_LOOPS);

    // Core0メインループのカウンタ
    int core0_loop_count = 0;

    // 軌道状態管理
    enum trajectory_state_t {
        TRAJECTORY_IDLE,
        TRAJECTORY_EXECUTING,
        TRAJECTORY_HANDLING
    };
    trajectory_state_t traj_state = TRAJECTORY_IDLE;
    // 軌道シーケンス管理
    const int WORK_NUM = 3;  // ワーク数
    trajectory_waypoint_t shooting_points[WORK_NUM] = {
        trajectory_waypoint_t(2.344f, 0.228f / gear_radius_P, 0.0f),
        trajectory_waypoint_t(2.344f, 0.228f / gear_radius_P, 0.0f),
        trajectory_waypoint_t(2.344f, 0.228f / gear_radius_P, 0.0f),
    };
    trajectory_waypoint_t work_points[WORK_NUM] = {
        trajectory_waypoint_t(3.783f, 0.1721f / gear_radius_P, 0.0f),
        trajectory_waypoint_t(4.024f, 0.3683f / gear_radius_P, 0.0f),
        trajectory_waypoint_t(4.43f, 0.5435f / gear_radius_P, 0.0f),
    };
    TrajectorySequenceManager* seq_manager = new TrajectorySequenceManager(g_debug_manager);
    trajectory_waypoint_t all_waypoints[2 * WORK_NUM];
    for (int i = 0; i < WORK_NUM; ++i) {
        all_waypoints[2 * i] = work_points[i];
        all_waypoints[2 * i + 1] = shooting_points[i];
    }
    seq_manager->setup_sequence(all_waypoints, 2 * WORK_NUM);

    // ハンド状態管理用ローカル変数
    hand_state_t hand_state = HAND_IDLE;
    bool has_work = false;
    absolute_time_t hand_timer = get_absolute_time();

    while (1) {
        // FIFOから同期信号を待機（ブロッキング）
        uint32_t signal = multicore_fifo_pop_blocking();

        // 同期信号を受信したら処理を実行
        if (signal == SYNC_SIGNAL) {
            core0_loop_count++;

            // 軌道状態機械による処理
            switch (traj_state) {
                case TRAJECTORY_IDLE: {
                    // シーケンスがアクティブなら次の軌道を設定
                    if (seq_manager->is_sequence_active()) {
                        float target_R, target_P;
                        if (seq_manager->get_next_waypoint(target_R, target_P)) {
                            float current_pos_R, current_pos_P;
                            mutex_enter_blocking(&g_state_mutex);
                            current_pos_R = g_robot_state.current_position_R;
                            current_pos_P = g_robot_state.current_position_P;
                            mutex_exit(&g_state_mutex);

                            if (calculate_trajectory_core0(current_pos_R, current_pos_P, target_R, target_P)) {
                                if (multicore_fifo_push_timeout_us(TRAJECTORY_DATA_SIGNAL, 0)) {
                                    traj_state = TRAJECTORY_EXECUTING;
                                    g_debug_manager->debug("Moving to waypoint: R=%.3f rad, P=%.1f mm",
                                                           target_R, target_P * gear_radius_P * 1000.0);
                                }
                            } else {
                                g_debug_manager->error("Failed to calculate trajectory to next waypoint");
                            }
                        } else {
                            g_debug_manager->info("All waypoints completed, sequence finished");
                            traj_state = TRAJECTORY_IDLE;
                        }
                    }
                    break;
                }

                case TRAJECTORY_EXECUTING:
                    // 軌道実行中は何もしない（完了信号待ち）
                    break;

                case TRAJECTORY_HANDLING:
                    // ハンド動作中
                    hand_tick(&hand_state, &has_work, &hand_timer);
                    if (hand_state == HAND_IDLE) {
                        // ハンド動作完了 → 次の軌道へ
                        if (seq_manager->is_sequence_active()) {
                            float target_R, target_P;
                            if (seq_manager->get_next_waypoint(target_R, target_P)) {
                                float current_pos_R, current_pos_P;
                                mutex_enter_blocking(&g_state_mutex);
                                current_pos_R = g_robot_state.current_position_R;
                                current_pos_P = g_robot_state.current_position_P;
                                mutex_exit(&g_state_mutex);

                                if (calculate_trajectory_core0(current_pos_R, current_pos_P, target_R, target_P)) {
                                    if (multicore_fifo_push_timeout_us(TRAJECTORY_DATA_SIGNAL, 0)) {
                                        traj_state = TRAJECTORY_EXECUTING;
                                        g_debug_manager->debug("Moving to waypoint: R=%.3f rad, P=%.1f mm",
                                                               target_R, target_P * gear_radius_P * 1000.0);
                                    }
                                } else {
                                    g_debug_manager->error("Failed to calculate trajectory to next waypoint");
                                }
                            } else {
                                g_debug_manager->info("All waypoints completed, sequence finished");
                                traj_state = TRAJECTORY_IDLE;
                            }
                        } else {
                            traj_state = TRAJECTORY_IDLE;
                        }
                    }
                    break;
            }
        } else if (signal == TRAJECTORY_COMPLETE_SIGNAL) {
            // 軌道完了信号を受信
            if (traj_state == TRAJECTORY_EXECUTING) {
                g_debug_manager->debug("Trajectory completed, starting hand operation");
                seq_manager->advance_to_next_waypoint();
                hand_state = HAND_IDLE;  // hand_tick()で自動的に開始
                traj_state = TRAJECTORY_HANDLING;
            }
        }

        // 通常の状態監視とデバッグ出力（同期信号受信時のみ）
        if (signal == SYNC_SIGNAL) {
            // 現在時刻を再取得
            float current_main_time = 0.0;
            mutex_enter_blocking(&g_state_mutex);
            current_main_time = g_robot_state.current_time;
            mutex_exit(&g_state_mutex);

            // 状態を取得してデバッグ出力（排他制御あり）
            mutex_enter_blocking(&g_state_mutex);
            // デバッグ用に現在状態も取得
            float current_pos_R = g_robot_state.current_position_R;
            float current_pos_P = g_robot_state.current_position_P;
            float current_vel_R = g_robot_state.current_velocity_R;
            float current_vel_P = g_robot_state.current_velocity_P;
            float target_vel_R = g_robot_state.target_velocity_R;
            float target_vel_P = g_robot_state.target_velocity_P;
            float target_torque_R = g_robot_state.target_torque_R;
            float target_torque_P = g_robot_state.target_torque_P;
            float target_cur_R = g_robot_state.target_current_R;
            float target_cur_P = g_robot_state.target_current_P;

            // 台形プロファイル制御情報
            float traj_target_pos_R, traj_target_pos_P;
            float traj_target_vel_R, traj_target_vel_P;
            get_safe_trajectory_targets(current_pos_R, current_pos_P,
                                        &traj_target_pos_R, &traj_target_pos_P,
                                        &traj_target_vel_R, &traj_target_vel_P);
            bool traj_active_R = g_trajectory_data.active;
            bool traj_active_P = g_trajectory_data.active;

            // 最終目標位置（Core0→Core1）
            float final_target_pos_R = g_trajectory_data.final_target_R;
            float final_target_pos_P = g_trajectory_data.final_target_P;

            int timing_violations = g_robot_state.timing_violation_count;
            led_mode_t led_status = g_robot_state.led_status;
            int can_errors = g_robot_state.can_error_count;

            // エンコーダ詳細情報を共有変数から取得
            int16_t p_turn_count = g_robot_state.encoder_p_turn_count;
            float p_single_angle = g_robot_state.encoder_p_single_angle_deg;
            float r_angle_deg = g_robot_state.encoder_r_angle_deg;
            bool encoder_r_valid = g_robot_state.encoder_r_valid;
            bool encoder_p_valid = g_robot_state.encoder_p_valid;
            mutex_exit(&g_state_mutex);

            // 軌道状態変化の検出
            g_debug_manager->check_trajectory_state_changes(traj_active_R, traj_active_P,
                                                            current_pos_R, current_pos_P,
                                                            final_target_pos_R, final_target_pos_P,
                                                            gear_radius_P);

            // 軌道制限値の表示（1回のみ）
            g_debug_manager->print_trajectory_limits(TrajectoryLimits::R_MAX_VELOCITY, TrajectoryLimits::R_MAX_ACCELERATION,
                                                     TrajectoryLimits::P_MAX_VELOCITY, TrajectoryLimits::P_MAX_ACCELERATION,
                                                     gear_radius_P);

            // 異常値検出
            g_debug_manager->check_abnormal_values(traj_target_pos_P, gear_radius_P);

            // 定期ステータス出力（同期信号に基づく周期で）
            if (g_debug_manager->should_output_status(current_main_time)) {
                // Core0同期回数情報
                g_debug_manager->debug("Core0 sync count: %d (every %d Core1 loops)", core0_loop_count, SYNC_EVERY_N_LOOPS);

                // 軌道完了状況の詳細情報を取得
                float trajectory_final_target_R = 0.0, trajectory_final_target_P = 0.0;
                bool trajectory_position_reached = false;
                int trajectory_current_index = 0, trajectory_point_count = 0;

                mutex_enter_blocking(&g_trajectory_mutex);
                trajectory_final_target_R = g_trajectory_data.final_target_R;
                trajectory_final_target_P = g_trajectory_data.final_target_P;
                trajectory_position_reached = g_trajectory_data.position_reached;
                trajectory_current_index = g_trajectory_data.current_index;
                trajectory_point_count = g_trajectory_data.point_count;
                mutex_exit(&g_trajectory_mutex);

                // 軌道進行状況の出力（debug_managerに委譲）
                g_debug_manager->print_trajectory_progress(current_pos_R, current_pos_P,
                                                           trajectory_final_target_R, trajectory_final_target_P,
                                                           trajectory_position_reached,
                                                           trajectory_current_index, trajectory_point_count,
                                                           gear_radius_P);  // 軌道デバッグ情報構造体の作成
                TrajectoryDebugInfo r_info = {
                    .final_target_pos = final_target_pos_R,
                    .trajectory_target_pos = traj_target_pos_R,
                    .trajectory_target_vel = traj_target_vel_R,
                    .current_pos = current_pos_R,
                    .current_vel = current_vel_R,
                    .final_target_vel = target_vel_R,
                    .trajectory_active = traj_active_R,
                    .gear_radius = 1.0f,  // R軸はrad単位なのでギア半径は使わない
                    .unit_name = "rad",
                    .axis_name = "R"};

                TrajectoryDebugInfo p_info = {
                    .final_target_pos = final_target_pos_P,
                    .trajectory_target_pos = traj_target_pos_P,
                    .trajectory_target_vel = traj_target_vel_P,
                    .current_pos = current_pos_P,
                    .current_vel = current_vel_P,
                    .final_target_vel = target_vel_P,
                    .trajectory_active = traj_active_P,
                    .gear_radius = gear_radius_P,
                    .unit_name = "rad",
                    .axis_name = "P"};

                // システム状態デバッグ情報構造体の作成
                SystemDebugInfo sys_info = {
                    .timing_violations = timing_violations,
                    .can_errors = can_errors,
                    .led_status = led_status,
                    .encoder_r_valid = encoder_r_valid,
                    .encoder_p_valid = encoder_p_valid,
                    .target_torque_R = target_torque_R,
                    .target_torque_P = target_torque_P,
                    .target_current_R = target_cur_R,
                    .target_current_P = target_cur_P,
                    .encoder_p_turn_count = p_turn_count,
                    .encoder_p_single_angle_deg = p_single_angle,
                    .encoder_r_angle_deg = r_angle_deg};

                // デバッグ情報出力
                g_debug_manager->print_trajectory_status(r_info, p_info);
                g_debug_manager->print_system_status(sys_info);
            }
        }
    }

    // gpio_put(SOLENOID_PIN1, 0);  // ソレノイドを非吸着状態にする
    gpio_put(PUMP_PIN, 0);  // ポンプを停止
    return 0;
}
