
#include "config.hpp"

namespace Mc = MicrocontrollerConfig;
namespace Mech = MechanismConfig;
namespace Traj = TrajectoryConfig;
namespace Ctrl = ControlConfig;
namespace Dxl = DynamixelConfig;

// CAN IC用SPI設定
const spi_config_t SPI1_CONFIG = {
    .spi_port = spi1,       // SPI1を使用
    .baudrate = 1'875'000,  // エンコーダ規定値 2MHz 整数分数でベスト:1.875MHz
    .pin_miso = 8,
    .pin_cs = {5, 7, 6},  // CSピン3つ（CAN、回転エンコーダ、直動エンコーダ）
    .num_cs_pins = 3,     // CSピン数
    .pin_sck = 10,
    .pin_mosi = 11,
    .pin_rst = -1  // リセットピンは使用しない
};

// MCP25625オブジェクトを作成（CAN SPI設定を使用）
mcp25625_t can(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_cs[0], SPI1_CONFIG.pin_rst);

// AMT223-V エンコーダマネージャを作成
AMT223V_Manager encoder_manager(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_miso, SPI1_CONFIG.pin_sck, SPI1_CONFIG.pin_mosi);

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, Mech::gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, Mech::gear_ratio_P);  // motor_id=2

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
    if (!init_spi(&SPI1_CONFIG)) {
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
        encoder1_index = encoder_manager.add_encoder(SPI1_CONFIG.pin_cs[1], Ctrl::R_VELOCITY_CUTOFF_FREQ, false);  // CS pin 7 (R軸: 単回転)
        encoder2_index = encoder_manager.add_encoder(SPI1_CONFIG.pin_cs[2], Ctrl::P_VELOCITY_CUTOFF_FREQ, true);   // CS pin 6 (P軸: マルチターン対応)

        if (encoder1_index < 0 || encoder2_index < 0) {
            g_debug_manager->error("Failed to add encoders!\n");
            return false;
        }
        g_debug_manager->info("Encoders added successfully: R=%d, P=%d\n", encoder1_index, encoder2_index);
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

// Core0用軌道計算関数
bool calculate_trajectory_core0(
    const float current_position[2],
    const float target_position[2],
    const float intermediate_position[2]) {
    // intermediate_positionがNAN配列の場合は中継点がない
    bool has_intermediate = false;
    if (!std::isnan(intermediate_position[0]) && !std::isnan(intermediate_position[1])) {
        has_intermediate = true;
    }

    ruckig::Ruckig<2> otg(Traj::TRAJECTORY_CONTROL_PERIOD);  // 2軸のRuckigオブジェクトを作成
    ruckig::InputParameter<2> input;
    ruckig::OutputParameter<2> output_intermediate;
    ruckig::InputParameter<2> input_intermediate;
    ruckig::OutputParameter<2> output;

    // 制限
    input.max_velocity = {
        Traj::RuckigConfig::R_MAX_VELOCITY,
        Traj::RuckigConfig::P_MAX_VELOCITY};
    input.min_velocity = {-input.max_velocity[0], -input.max_velocity[1]};

    // 進行方向判定
    bool is_forward_R = current_position[0] < target_position[0];
    bool is_forward_P = current_position[1] < target_position[1];
    // R軸加速度
    double accel_R = is_forward_R ? Traj::RuckigConfig::R_ACCEL : Traj::RuckigConfig::R_DECEL;
    double decel_R = is_forward_R ? Traj::RuckigConfig::R_DECEL : Traj::RuckigConfig::R_ACCEL;
    // P軸加速度
    double accel_P = is_forward_P ? Traj::RuckigConfig::P_ACCEL : Traj::RuckigConfig::P_DECEL;
    double decel_P = is_forward_P ? Traj::RuckigConfig::P_DECEL : Traj::RuckigConfig::P_ACCEL;
    input.max_acceleration = {accel_R, accel_P};
    input.min_acceleration = {-decel_R, -decel_P};
    input.max_jerk = {Traj::RuckigConfig::R_JERK, Traj::RuckigConfig::P_JERK};
    // 中継点の制限
    input_intermediate.max_velocity = input.max_velocity;
    input_intermediate.max_acceleration = input.max_acceleration;
    input_intermediate.max_jerk = input.max_jerk;
    input_intermediate.min_velocity = input.min_velocity;
    input_intermediate.min_acceleration = input.min_acceleration;

    // 出発点の設定
    input.current_position = {current_position[0], current_position[1]};
    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};

    // 目標位置と速度の設定
    if (has_intermediate) {  // 中継点が指定されている場合
        input.target_position = {intermediate_position[0], intermediate_position[1]};
        // 中継点のR軸は目標位置に向かう方向の速度に設定
        double R_intermediate_vel = (target_position[0] - intermediate_position[0] > 0.0) ? input.max_velocity[0] : input.min_velocity.value()[0];
        input.target_velocity = {R_intermediate_vel, 0.0};  // R軸は中継点が合っても動く方向は変わらない
        input.target_acceleration = {0.0, 0.0};

        // 中継点から（存在すれば）
        input_intermediate.current_position = input.target_position;
        input_intermediate.current_velocity = input.target_velocity;
        input_intermediate.current_acceleration = input.target_acceleration;

        input_intermediate.target_position = {target_position[0], target_position[1]};
        input_intermediate.target_velocity = {0.0, 0.0};
        input_intermediate.target_acceleration = {0.0, 0.0};
    } else {  // 中継点がない場合は直接目標位置へ
        input.target_position = {target_position[0], target_position[1]};
        input.target_velocity = {0.0, 0.0};
        input.target_acceleration = {0.0, 0.0};
    }

    trajectory_point_t trajectory_points[Traj::MAX_TRAJECTORY_POINTS];
    uint16_t point_count = 0;
    bool overflow = false;
    if (has_intermediate) {
        while (otg.update(input, output_intermediate) == ruckig::Result::Working) {
            if (point_count >= Traj::MAX_TRAJECTORY_POINTS) {
                overflow = true;
                break;
            }
            trajectory_point_t pt;
            pt.position_R = output_intermediate.new_position[0];
            pt.velocity_R = output_intermediate.new_velocity[0];
            pt.acceleration_R = output_intermediate.new_acceleration[0];
            pt.position_P = output_intermediate.new_position[1];
            pt.velocity_P = output_intermediate.new_velocity[1];
            pt.acceleration_P = output_intermediate.new_acceleration[1];
            trajectory_points[point_count++] = pt;
            output_intermediate.pass_to_input(input);
        }
        while (!overflow && otg.update(input_intermediate, output) == ruckig::Result::Working) {
            if (point_count >= Traj::MAX_TRAJECTORY_POINTS) {
                overflow = true;
                break;
            }
            trajectory_point_t pt;
            pt.position_R = output.new_position[0];
            pt.velocity_R = output.new_velocity[0];
            pt.acceleration_R = output.new_acceleration[0];
            pt.position_P = output.new_position[1];
            pt.velocity_P = output.new_velocity[1];
            pt.acceleration_P = output.new_acceleration[1];
            trajectory_points[point_count++] = pt;
            output.pass_to_input(input_intermediate);
        }
    } else {
        while (otg.update(input, output) == ruckig::Result::Working) {
            if (point_count >= Traj::MAX_TRAJECTORY_POINTS) {
                overflow = true;
                break;
            }
            trajectory_point_t pt;
            pt.position_R = output.new_position[0];
            pt.velocity_R = output.new_velocity[0];
            pt.acceleration_R = output.new_acceleration[0];
            pt.position_P = output.new_position[1];
            pt.velocity_P = output.new_velocity[1];
            pt.acceleration_P = output.new_acceleration[1];
            trajectory_points[point_count++] = pt;
            output.pass_to_input(input);
        }
    }

    if (!overflow) {
        // 軌道データを計算して配列に格納
        mutex_enter_blocking(&g_trajectory_mutex);
        g_trajectory_data.point_count = point_count;
        g_trajectory_data.current_index = 0;
        g_trajectory_data.active = false;
        g_trajectory_data.complete = false;
        g_trajectory_data.final_target_R = target_position[0];  // 最終目標位置を保存
        g_trajectory_data.final_target_P = target_position[1];
        g_trajectory_data.position_reached = false;
        memcpy(g_trajectory_data.points, trajectory_points, sizeof(trajectory_point_t) * point_count);
        mutex_exit(&g_trajectory_mutex);
    } else {
        g_debug_manager->error("Trajectory overflow: %d points exceeded maximum of %d\n", point_count, Traj::MAX_TRAJECTORY_POINTS);
        return false;
    }

    g_debug_manager->debug("Trajectory calculated: %d points, max_time=%.2fs", point_count, point_count * Mc::CONTROL_PERIOD_S);
    g_debug_manager->debug("  R: %.3f → %.3f rad, P: %.3f → %.3f rad",
                           current_position[0], target_position[0], current_position[1], target_position[1]);

    return true;
}

void init_hand() {
    // ポンプの設定
    gpio_init(Dxl::PUMP_PIN);
    gpio_set_dir(Dxl::PUMP_PIN, GPIO_OUT);
    gpio_init(Dxl::SOLENOID_PIN);
    gpio_set_dir(Dxl::SOLENOID_PIN, GPIO_OUT);

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    g_debug_manager->info("Initializing Dynamixels (Daisy Chain on UART0)...\n");
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    sleep_ms(100);
    write_statusReturnLevel(&UART0, Dxl::DXL_ID1, 0x00);
    write_statusReturnLevel(&UART0, Dxl::DXL_ID2, 0x00);
    sleep_ms(100);
    write_dxl_led(&UART0, Dxl::DXL_ID1, true);
    write_dxl_led(&UART0, Dxl::DXL_ID2, true);
    sleep_ms(1000);
    write_dxl_led(&UART0, Dxl::DXL_ID1, false);
    write_dxl_led(&UART0, Dxl::DXL_ID2, false);
    sleep_ms(100);
    write_torqueEnable(&UART0, Dxl::DXL_ID1, false);
    write_torqueEnable(&UART0, Dxl::DXL_ID2, false);
    sleep_ms(100);
    write_dxl_current_limit(&UART0, Dxl::DXL_ID1, 1000);  // ID=1, 電流制限=100mA
    write_dxl_current_limit(&UART0, Dxl::DXL_ID2, 1000);  // ID=2, 電流制限=100mA
    sleep_ms(100);
    write_operatingMode(&UART0, Dxl::DXL_ID1, false);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART0, Dxl::DXL_ID2, false);
    sleep_ms(1000);
    write_torqueEnable(&UART0, Dxl::DXL_ID1, true);
    write_torqueEnable(&UART0, Dxl::DXL_ID2, true);
    sleep_ms(500);
    control_position(&UART0, Dxl::DXL_ID1, 132.10f);
    sleep_ms(500);
    control_position_multiturn(&UART0, Dxl::DXL_ID2, Dxl::START_UP_ANGLE);
    sleep_ms(1000);
    gpio_put(Dxl::SOLENOID_PIN, 0);  // ソレノイドを吸着状態にする
    gpio_put(Dxl::PUMP_PIN, 1);
    g_debug_manager->info("hand initialized\n");
}

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, bool* has_work, absolute_time_t* state_start_time, float hand_angle) {
    uint32_t elapsed_ms = absolute_time_diff_us(*state_start_time, get_absolute_time()) / 1000;
    switch (*hand_state) {
        case HAND_IDLE:
            g_debug_manager->debug("hand requested\n");
            if (!*has_work) {
                *hand_state = HAND_LOWERING;
                *state_start_time = get_absolute_time();
                gpio_put(Dxl::PUMP_PIN, 1);
                g_debug_manager->debug("Hand lowering...");
                control_position_multiturn(&UART0, Dxl::DXL_ID2, Dxl::DOWN_ANGLE);
            } else {
                *hand_state = HAND_RELEASE;
                *state_start_time = get_absolute_time();
                gpio_put(Dxl::SOLENOID_PIN, 1);
            }
            break;

        case HAND_LOWERING:
            if (elapsed_ms >= 300) {
                *hand_state = HAND_SUCTION_WAIT;
                *state_start_time = get_absolute_time();
                g_debug_manager->debug("Hand suction wait...\n");
            }
            break;

        case HAND_SUCTION_WAIT:
            if (elapsed_ms >= 100) {
                *hand_state = HAND_RAISING;
                *state_start_time = get_absolute_time();
                control_position_multiturn(&UART0, Dxl::DXL_ID2, Dxl::UP_ANGLE);
                g_debug_manager->debug("Hand raising...\n");
            }
            break;

        case HAND_RAISING:
            if (elapsed_ms >= 200) {
                *has_work = true;
                control_position(&UART0, Dxl::DXL_ID1, hand_angle);
                g_debug_manager->debug("Hand raised, work done.\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
            }
            break;

        case HAND_RELEASE:
            if (elapsed_ms >= 150) {
                *has_work = false;
                gpio_put(Dxl::SOLENOID_PIN, 0);
                control_position(&UART0, Dxl::DXL_ID1, hand_angle);
                g_debug_manager->debug("Hand released\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
            }
            break;

        case HAND_WAITING:
            if (elapsed_ms >= 150) {
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
        *traj_vel_R = 0.0f;
        *traj_vel_P = 0.0f;
    } else {
        // 軌道停止時
        *traj_pos_R = current_pos_R;
        *traj_pos_P = current_pos_P;
        *traj_vel_R = 0.0f;
        *traj_vel_P = 0.0f;
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

    float disturbance_torque_R = 0.0f;                                                                        // R軸の外乱トルク
    float control_torque_R = 0.0f;                                                                            // R軸の制御トルク
    float target_torque_R = 0.0f;                                                                             // R軸の目標トルク
    float error_position_R = 0.0f;                                                                            // R軸の位置誤差
    float error_velocity_R = 0.0f;                                                                            // R軸の速度誤差
    float acceleration_feedforward_R = 0.0f;                                                                  // R軸の加速度フィードフォワード
    disturbance_observer_t dob_R(Mech::R_EQ_INERTIA, Ctrl::R_VELOCITY_CUTOFF_FREQ, Ctrl::R_DOB_CUTOFF_FREQ);  // R軸の外乱オブザーバ

    float disturbance_torque_P = 0.0f;                                                                        // P軸の外乱トルク
    float control_torque_P = 0.0f;                                                                            // P軸の制御トルク
    float target_torque_P = 0.0f;                                                                             // P軸の目標トルク
    float error_position_P = 0.0f;                                                                            // P軸の位置誤差
    float error_velocity_P = 0.0f;                                                                            // P軸の速度誤差
    float acceleration_feedforward_P = 0.0f;                                                                  // P軸の加速度フィードフォワード
    disturbance_observer_t dob_P(Mech::P_EQ_INERTIA, Ctrl::P_VELOCITY_CUTOFF_FREQ, Ctrl::P_DOB_CUTOFF_FREQ);  // P軸の外乱オブザーバ

    // ループカウンタの初期化
    int loop_counter = 0;

    while (true) {
        // 制御周期開始処理
        control_timing_start(&control_timing, Mc::CONTROL_PERIOD_MS);

        // FIFOから軌道開始信号をチェック（ノンブロッキング）
        uint32_t trajectory_signal;
        if (multicore_fifo_pop_timeout_us(0, &trajectory_signal)) {
            if (trajectory_signal == Mc::TRAJECTORY_DATA_SIGNAL) {
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
        float motor_position_R = 0.0f, motor_position_P = 0.0f;
        float motor_velocity_R = 0.0f, motor_velocity_P = 0.0f;
        bool enc1_ok = encoder_manager.read_encoder(0);  // エンコーダ0 (R軸)
        bool enc2_ok = encoder_manager.read_encoder(1);  // エンコーダ1 (P軸)

        // エンコーダデータの取得
        float encoder_r_angle_deg = 0.0f;
        int16_t encoder_p_turn_count = 0;
        float encoder_p_single_angle_deg = 0.0f;
        float encoder_p_continuous_angle_rad = 0.0f;

        if (enc1_ok) {
            motor_position_R = encoder_manager.get_encoder_angle_rad(0) * Mech::ENCODER_R_DIRECTION;
            motor_velocity_R = encoder_manager.get_encoder_angular_velocity_rad(0) * Mech::ENCODER_R_DIRECTION;
            encoder_r_angle_deg = encoder_manager.get_encoder_angle_deg(0);
        }

        if (enc2_ok) {
            // P軸はマルチターン対応エンコーダのため連続角度を使用
            motor_position_P = encoder_manager.get_encoder_continuous_angle_rad(1) * Mech::ENCODER_P_DIRECTION;
            motor_velocity_P = encoder_manager.get_encoder_angular_velocity_rad(1) * Mech::ENCODER_P_DIRECTION;

            // デバッグ用エンコーダ情報を取得
            encoder_p_turn_count = encoder_manager.get_encoder_turn_count(1);
            encoder_p_single_angle_deg = encoder_manager.get_encoder_angle_deg(1);
            encoder_p_continuous_angle_rad = encoder_manager.get_encoder_continuous_angle_rad(1);
        }

        // // --- モータフィードバック受信 ---
        // float motor_velocity_R = 0.0, motor_velocity_P = 0.0f;
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
        float trajectory_target_vel_R = 0.0f;
        float trajectory_target_vel_P = 0.0f;
        float trajectory_target_accel_R = 0.0f;
        float trajectory_target_accel_P = 0.0f;

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

                // Core1が10回回るごとに軌道インデックスを1だけ進める
                static float last_trajectory_update_time = current_time_s;  // 最後の軌道更新時刻
                if (current_time_s - last_trajectory_update_time >= Traj::TRAJECTORY_CONTROL_PERIOD) {
                    g_trajectory_data.current_index++;
                    last_trajectory_update_time = current_time_s;
                }

                // 軌道の時系列が完了した場合、位置ベースの完了判定に移行
            } else if (g_trajectory_data.current_index >= g_trajectory_data.point_count) {
                // 最終目標位置を設定し、位置到達判定モードに移行
                trajectory_target_pos_R = g_trajectory_data.final_target_R;
                trajectory_target_pos_P = g_trajectory_data.final_target_P;
                trajectory_target_vel_R = 0.0f;
                trajectory_target_vel_P = 0.0f;
                trajectory_target_accel_R = 0.0f;
                trajectory_target_accel_P = 0.0f;

                // 位置到達判定
                float position_error_R = std::abs(g_trajectory_data.final_target_R - motor_position_R);
                float position_error_P = std::abs(g_trajectory_data.final_target_P - motor_position_P);
                float velocity_magnitude_R = std::abs(motor_velocity_R);
                float velocity_magnitude_P = std::abs(motor_velocity_P);

                // 位置誤差と速度が両方とも許容範囲内の場合に完了とする
                if (position_error_R < Traj::TRAJECTORY_COMPLETION_TOLERANCE_R &&
                    position_error_P < Traj::TRAJECTORY_COMPLETION_TOLERANCE_P &&
                    velocity_magnitude_R < Traj::TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD &&
                    velocity_magnitude_P < Traj::TRAJECTORY_COMPLETION_VELOCITY_THRESHOLD) {
                    g_trajectory_data.active = false;
                    g_trajectory_data.complete = true;
                    g_trajectory_data.position_reached = true;
                    trajectory_completed = true;
                }
            }
        } else if (!g_trajectory_data.active) {
            // 軌道停止時は現在位置を保持
            trajectory_target_pos_R = motor_position_R;
            trajectory_target_vel_R = 0.0f;
            trajectory_target_accel_R = 0.0f;
            trajectory_target_pos_P = motor_position_P;
            trajectory_target_vel_P = 0.0f;
            trajectory_target_accel_P = 0.0f;
        }
        mutex_exit(&g_trajectory_mutex);

        // 軌道完了信号の送信（mutex外で実行）
        if (trajectory_completed) {
            if (!multicore_fifo_push_timeout_us(Mc::TRAJECTORY_COMPLETE_SIGNAL, 0)) {
                // FIFO満杯の場合は次回再試行
                g_debug_manager->error("Core1: Failed to push trajectory complete signal to Core0 FIFO");
            }
        }

        // --- 制御計算 ---
        // R軸の制御計算
        error_position_R = Mech::R_EQ_INERTIA * Ctrl::R_POSITION_GAIN * (trajectory_target_pos_R - motor_position_R);
        error_velocity_R = Mech::R_EQ_INERTIA * Ctrl::R_VELOCITY_GAIN * (trajectory_target_vel_R - motor_velocity_R);
        acceleration_feedforward_R = Mech::R_EQ_INERTIA * trajectory_target_accel_R;
        control_torque_R = error_position_R + error_velocity_R + disturbance_torque_R + acceleration_feedforward_R;  // 制御トルク計算
        target_torque_R = clampTorque(control_torque_R, Mech::R_MAX_TORQUE);                                         // 制御トルク制限
        control_torque_R = target_torque_R;                                                                          // 制御トルクを目標トルクに設定
        disturbance_torque_R = dob_R.update(control_torque_R, motor_velocity_R);                                     // 外乱トルクの更新

        // P軸の制御計算
        error_position_P = Mech::P_EQ_INERTIA * Ctrl::P_POSITION_GAIN * (trajectory_target_pos_P - motor_position_P);
        error_velocity_P = Mech::P_EQ_INERTIA * Ctrl::P_VELOCITY_GAIN * (trajectory_target_vel_P - motor_velocity_P);
        acceleration_feedforward_P = Mech::P_EQ_INERTIA * trajectory_target_accel_P;
        control_torque_P = error_position_P + error_velocity_P + disturbance_torque_P + acceleration_feedforward_P;
        target_torque_P = clampTorque(control_torque_P, Mech::P_MAX_TORQUE);
        control_torque_P = target_torque_P;
        disturbance_torque_P = dob_P.update(control_torque_P, motor_velocity_P);  // 外乱トルクの更新

        // // トルクから電流への変換
        target_current[0] = target_torque_R / Mech::R_TORQUE_CONSTANT;  // Motor1 (R軸)
        target_current[1] = target_torque_P / Mech::P_TORQUE_CONSTANT;  // Motor2 (P軸)
        // target_current[0] = 0.0f;                                  // Motor1 (R軸)
        // target_current[1] = 0.0f;                                  // Motor2 (P軸)

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
        if (loop_counter >= Mc::SYNC_EVERY_N_LOOPS) {
            loop_counter = 0;  // カウンタをリセット

            // FIFOに同期信号を送信（ノンブロッキング）
            if (!multicore_fifo_push_timeout_us(Mc::SYNC_SIGNAL, 0)) {
                // FIFO満杯の場合は何もしない（次回再試行）
                printf("Core1: Failed to push sync signal to Core0 FIFO\n");
            }
        }

        // 制御周期終了処理
        control_timing_end(&control_timing, Mc::CONTROL_PERIOD_MS);
    }
}

// システム初期化関数
bool initialize_system() {
    stdio_init_all();  // UARTなど初期化
    gpio_init(Mc::SHUTDOWN_PIN);
    gpio_set_dir(Mc::SHUTDOWN_PIN, GPIO_OUT);
    gpio_put(Mc::SHUTDOWN_PIN, 0);  // HIGHにしておくとPicoが動かないのでLOWに設定
    sleep_ms(2000);                 // 少し待機して安定化

    // デバッグマネージャの初期化
    g_debug_manager = new DebugManager(DebugLevel::INFO, 0.1f);

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
    g_trajectory_data.final_target_R = 0.0f;
    g_trajectory_data.final_target_P = 0.0f;
    g_trajectory_data.position_reached = false;

    // 制御初期値
    g_robot_state.current_position_R = 0.0f;
    g_robot_state.current_position_P = 0.0f;
    g_robot_state.current_velocity_R = 0.0f;
    g_robot_state.current_velocity_P = 0.0f;
    g_robot_state.target_velocity_R = 0.0f;
    g_robot_state.target_velocity_P = 0.0f;
    g_robot_state.target_torque_R = 0.0f;
    g_robot_state.target_torque_P = 0.0f;

    // エンコーダ詳細情報の初期化
    g_robot_state.encoder_p_turn_count = 0;
    g_robot_state.encoder_p_single_angle_deg = 0.0f;
    g_robot_state.encoder_p_continuous_angle_rad = 0.0f;
    g_robot_state.encoder_r_angle_deg = 0.0f;
    g_robot_state.encoder_r_valid = false;
    g_robot_state.encoder_p_valid = false;

    // 現在時間の初期化
    g_robot_state.current_time = 0.0f;

    // デバッグ情報の初期化
    g_robot_state.target_current_R = 0.0f;
    g_robot_state.target_current_P = 0.0f;
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
    g_debug_manager->info("Starting control loop at %.1f ms (%.0f Hz)", Mc::CONTROL_PERIOD_MS, 1000.0f / Mc::CONTROL_PERIOD_MS);
    g_debug_manager->info("Core sync enabled: Core0 processes every %d Core1 loops (%.1f ms)", Mc::SYNC_EVERY_N_LOOPS, Mc::CONTROL_PERIOD_MS * Mc::SYNC_EVERY_N_LOOPS);

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
    constexpr int WAYPOINT_NUM = 80;  // 軌道点数
    constexpr float INTERMEDIATE_POSITION_R = 4.102f;
    constexpr float INTERMEDIATE_POSITION_P = -0.1322f / Mech::gear_radius_P;

    static trajectory_waypoint_t all_waypoints[WAYPOINT_NUM] = {
        // 一番奥側ロボットから見て左から右へ
        // 1行目
        trajectory_waypoint_t(3.407f, -0.4584f / Mech::gear_radius_P, 132.10f),  // 1-1
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.507f, -0.3872f / Mech::gear_radius_P, 126.91f),  // 1-2
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.711f, -0.3034f / Mech::gear_radius_P, 115.58f),  // 1-3
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.858f, -0.2662f / Mech::gear_radius_P, 104.94f),  // 1-4
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.121f, -0.2349f / Mech::gear_radius_P, 92.29f),  // 1-5
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.295f, -0.2348f / Mech::gear_radius_P, 79.80f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 1-6
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.544f, -0.2640f / Mech::gear_radius_P, 67.32f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 1-7
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.687f, -0.3081f / Mech::gear_radius_P, 57.92f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 1-8
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.859f, -0.3910f / Mech::gear_radius_P, 46.41f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 1-9
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.960f, -0.4566f / Mech::gear_radius_P, 43.42f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 1-10
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),
        // 2行目
        trajectory_waypoint_t(3.315f, -0.3916f / Mech::gear_radius_P, 137.98f),  // 2-1
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.419f, -0.3179f / Mech::gear_radius_P, 130.43f),  // 2-2
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.628f, -0.2194f / Mech::gear_radius_P, 117.86f),  // 2-3
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.797f, -0.1739f / Mech::gear_radius_P, 107.93f),  // 2-4
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.102f, -0.1322f / Mech::gear_radius_P, 91.67f),  // 2-5
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.327f, -0.1343f / Mech::gear_radius_P, 79.10f),  // 2-6
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.625f, -0.1682f / Mech::gear_radius_P, 61.52f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 2-7
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.785f, -0.2205f / Mech::gear_radius_P, 52.73f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 2-8
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(4.972f, -0.3138f / Mech::gear_radius_P, 39.64f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 2-9
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),

        trajectory_waypoint_t(5.069f, -0.3886f / Mech::gear_radius_P, 35.24f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),  // 2-10
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f, INTERMEDIATE_POSITION_R, INTERMEDIATE_POSITION_P),
        // 3行目
        trajectory_waypoint_t(3.203f, -0.3313f / Mech::gear_radius_P, 142.82f),  // 3-1
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.303f, -0.2523f / Mech::gear_radius_P, 137.55f),  // 3-2
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.527f, -0.1428f / Mech::gear_radius_P, 124.19f),  // 3-3
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.713f, -0.0834f / Mech::gear_radius_P, 113.29f),  // 3-4
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.105f, -0.0379f / Mech::gear_radius_P, 90.0f),  // 3-5
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.378f, -0.0357f / Mech::gear_radius_P, 75.59f),  // 3-6
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.735f, -0.0829f / Mech::gear_radius_P, 55.28f),  // 3-7
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.908f, -0.1413f / Mech::gear_radius_P, 45.70f),  // 3-8
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(5.096f, -0.2512f / Mech::gear_radius_P, 33.75f),  // 3-9
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(5.181f, -0.3315f / Mech::gear_radius_P, 28.74f),  // 3-10
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        // 4行目（ロボットに一番近い行）
        trajectory_waypoint_t(3.075f, -0.2835f / Mech::gear_radius_P, 149.77f),  // 4-1
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.165f, -0.1925f / Mech::gear_radius_P, 146.07f),  // 4-2
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.377f, -0.0687f / Mech::gear_radius_P, 132.36f),  // 4-3
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.590f, -0.0029f / Mech::gear_radius_P, 120.94f),  // 4-4
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(3.766f, -0.0365f / Mech::gear_radius_P, 90.0f),  // 4-5 // ダミーデータ
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.132f, -0.0365f / Mech::gear_radius_P, 75.0f),  // 4-6 // ダミーデータ
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(4.904f, -0.0024f / Mech::gear_radius_P, 45.26f),  // 4-7
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(5.090f, -0.0716f / Mech::gear_radius_P, 33.84f),  // 4-8
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(5.252f, -0.1933f / Mech::gear_radius_P, 25.75f),  // 4-9
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),

        trajectory_waypoint_t(5.327f, -0.2841f / Mech::gear_radius_P, 18.81f),  // 4-10
        trajectory_waypoint_t(2.380f, -0.5645f / Mech::gear_radius_P, 90.0f),
    };
    TrajectorySequenceManager* seq_manager = new TrajectorySequenceManager(g_debug_manager);
    seq_manager->setup_sequence(all_waypoints, WAYPOINT_NUM);

    // ハンド状態管理用ローカル変数
    hand_state_t hand_state = HAND_IDLE;
    bool has_work = false;
    absolute_time_t hand_timer = get_absolute_time();

    while (1) {
        // FIFOから同期信号を待機（ブロッキング）
        uint32_t signal = multicore_fifo_pop_blocking();

        // 同期信号を受信したら処理を実行
        if (signal == Mc::SYNC_SIGNAL) {
            core0_loop_count++;

            // 軌道状態機械による処理
            auto try_start_next_trajectory = [&]() {
                if (seq_manager->is_sequence_active()) {
                    float target_position[2];
                    float intermediate_position[2];
                    if (seq_manager->get_next_waypoint(target_position, intermediate_position)) {
                        float current_position[2];
                        mutex_enter_blocking(&g_state_mutex);
                        current_position[0] = g_robot_state.current_position_R;
                        current_position[1] = g_robot_state.current_position_P;
                        mutex_exit(&g_state_mutex);

                        if (calculate_trajectory_core0(current_position, target_position, intermediate_position)) {
                            if (multicore_fifo_push_timeout_us(Mc::TRAJECTORY_DATA_SIGNAL, 0)) {
                                traj_state = TRAJECTORY_EXECUTING;
                                g_debug_manager->debug("Moving to waypoint: R=%.3f rad, P=%.1f mm",
                                                       target_position[0], target_position[1] * Mech::gear_radius_P * 1000.0);
                                return;
                            }
                        } else {
                            g_debug_manager->error("Failed to calculate trajectory to next waypoint");
                        }
                    } else {
                        g_debug_manager->info("All waypoints completed, sequence finished");
                        traj_state = TRAJECTORY_IDLE;
                    }
                } else {
                    traj_state = TRAJECTORY_IDLE;  // シーケンスがアクティブでない場合はアイドル状態に戻す
                }
            };

            switch (traj_state) {
                case TRAJECTORY_IDLE:
                    try_start_next_trajectory();
                    break;
                case TRAJECTORY_EXECUTING:
                    // 軌道実行中は何もしない（完了信号待ち）
                    break;
                case TRAJECTORY_HANDLING:
                    // ハンド動作中
                    int seq_index = seq_manager->get_current_waypoint_index();
                    hand_tick(&hand_state, &has_work, &hand_timer, all_waypoints[seq_index].end_effector_angle);
                    if (hand_state == HAND_IDLE) {
                        try_start_next_trajectory();
                    }
                    break;
            }
        } else if (signal == Mc::TRAJECTORY_COMPLETE_SIGNAL) {
            // 軌道完了信号を受信
            if (traj_state == TRAJECTORY_EXECUTING) {
                g_debug_manager->debug("Trajectory completed, starting hand operation");
                seq_manager->advance_to_next_waypoint();
                hand_state = HAND_IDLE;  // hand_tick()で自動的に開始
                traj_state = TRAJECTORY_HANDLING;
            }
        }

        // 通常の状態監視とデバッグ出力（同期信号受信時のみ）
        if (signal == Mc::SYNC_SIGNAL) {
            // 現在時刻を再取得
            float current_main_time = 0.0f;
            mutex_enter_blocking(&g_state_mutex);
            current_main_time = g_robot_state.current_time;

            // 状態を取得してデバッグ出力（排他制御あり）
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

            mutex_enter_blocking(&g_trajectory_mutex);
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
            mutex_exit(&g_trajectory_mutex);

            // 軌道状態変化の検出
            g_debug_manager->check_trajectory_state_changes(traj_active_R, traj_active_P,
                                                            current_pos_R, current_pos_P,
                                                            final_target_pos_R, final_target_pos_P,
                                                            Mech::gear_radius_P);

            // 異常値検出
            g_debug_manager->check_abnormal_values(traj_target_pos_P, Mech::gear_radius_P);

            // 定期ステータス出力（同期信号に基づく周期で）
            if (g_debug_manager->should_output_status(current_main_time)) {
                // Core0同期回数情報
                g_debug_manager->debug("Core0 sync count: %d (every %d Core1 loops)", core0_loop_count, Mc::SYNC_EVERY_N_LOOPS);

                // 軌道完了状況の詳細情報を取得
                float trajectory_final_target_R = 0.0f, trajectory_final_target_P = 0.0f;
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
                                                           Mech::gear_radius_P);  // 軌道デバッグ情報構造体の作成
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
                    .gear_radius = Mech::gear_radius_P,
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
    gpio_put(Dxl::PUMP_PIN, 0);  // ポンプを停止
    return 0;
}
