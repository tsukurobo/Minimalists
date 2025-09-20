#include "config.hpp"
#include "dynamixel.hpp"
#include "servo.hpp"
#include "trajectory.hpp"
#include "trajectory_sequence_manager.hpp"

namespace Mc = MicrocontrollerConfig;
namespace Mech = MechanismConfig;
namespace Traj = TrajectoryConfig;
namespace Ctrl = ControlConfig;
namespace Hand = HandConfig;
namespace Dist = DisturbanceConfig;

// CAN IC用SPI設定
const spi_config_t SPI1_CONFIG = {
    .spi_port = spi1,       // SPI1を使用
    .baudrate = 1'875'000,  // エンコーダ規定値 2MHz 整数分数でベスト:1.875MHz
    .pin_miso = SPI1::MISO_PIN,
    .pin_cs = {SPI1::MCP25625::CS_PIN, SPI1::Encoder::R_PIN, SPI1::Encoder::P_PIN},  // CSピン3つ（CAN、回転エンコーダ、直動エンコーダ）
    .num_cs_pins = 3,                                                                // CSピン数
    .pin_sck = SPI1::SCK_PIN,
    .pin_mosi = SPI1::MOSI_PIN,
    .pin_rst = -1  // MCP25625用リセットピン
};

// 妨害展開レベル (0: 初期状態, 1: 1段階目, 2: 2段階目)
volatile int g_disturbance_level = 0;

// 最速アーム状態
QuickArm_state_t g_quickarm_state = HAND_STANDBY;
bool g_moving_quick_arm = false;  // 妨害展開用高速アーム動作中フラグ

// MCP25625オブジェクトを作成（CAN SPI設定を使用）
mcp25625_t can(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_cs[0], SPI1_CONFIG.pin_rst);

// AMT223-V エンコーダマネージャを作成
AMT223V_Manager encoder_manager(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_miso, SPI1_CONFIG.pin_sck, SPI1_CONFIG.pin_mosi);

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, Mech::gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, Mech::gear_ratio_P);  // motor_id=2

// シューティングエリアのサーボ
Servo shooting_servo(ShootingConfig::SERVO_PIN);
bool is_moving_shooting_servo = false;
absolute_time_t g_last_shoot_servo_time = {0};
void move_shooting_servo() {
    is_moving_shooting_servo = true;
    g_last_shoot_servo_time = get_absolute_time();
}

float clampTorque(float torque, float max_torque) {
    if (torque > max_torque) {
        return max_torque;
    } else if (torque < -max_torque) {
        return -max_torque;
    }
    return torque;
}

float clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    } else if (value > max_value) {
        return max_value;
    }
    return value;
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
    const float intermediate_pos1[2],
    const float intermediate_pos2[2],
    const float intermediate_pos3[2]) {
    // 中継点の有無を判定
    bool has_inter1 = !std::isnan(intermediate_pos1[0]) && !std::isnan(intermediate_pos1[1]);
    bool has_inter2 = !std::isnan(intermediate_pos2[0]) && !std::isnan(intermediate_pos2[1]);
    bool has_inter3 = !std::isnan(intermediate_pos3[0]) && !std::isnan(intermediate_pos3[1]);

    // 軌道計算用入力構造体、まずはR軸のみ軌道を計算
    trajectory_t trajectory_R_core0(Traj::R_MAX_VELOCITY, Traj::R_ACCEL, Traj::R_DECEL, Traj::R_S_CURVE_RATIO, current_position[0], target_position[0]);

    // R軸のみ軌道を計算
    trajectory_R_core0.calculate_s_curve_trajectory_params();
    float total_time_R = trajectory_R_core0.get_total_time();  // R軸の総移動時間[s]
    int point_count = static_cast<int>(total_time_R / Traj::TRAJECTORY_CONTROL_PERIOD) + 1;
    if (point_count > Traj::MAX_TRAJECTORY_POINTS) {
        g_debug_manager->error("Calculated R trajectory points %d exceed maximum limit!\n", point_count);
        return false;
    }

    // 軌道点の配列を確保
    static trajectory_point_t trajectory_points[Traj::MAX_TRAJECTORY_POINTS];

    // 進行方向判定
    bool is_forward_R = current_position[0] < target_position[0];
    bool is_forward_P = has_inter1 ? (current_position[1] < intermediate_pos1[1]) : (current_position[1] < target_position[1]);

    struct SectionInfo {
        const float* start_pos;
        const float* end_pos;
    };
    SectionInfo sections[4];  // 最大4区間（中継点3つ＋目標点）
    int section_count = 0;

    // 区間リストを構築
    if (has_inter1) sections[section_count++] = {current_position, intermediate_pos1};
    if (has_inter2) sections[section_count++] = {intermediate_pos1, intermediate_pos2};
    if (has_inter3) sections[section_count++] = {intermediate_pos2, intermediate_pos3};
    const float* final_section_start_pos;
    if (has_inter3) {
        final_section_start_pos = intermediate_pos3;
    } else if (has_inter2) {
        final_section_start_pos = intermediate_pos2;
    } else if (has_inter1) {
        final_section_start_pos = intermediate_pos1;
    } else {
        final_section_start_pos = current_position;
    }
    sections[section_count++] = {final_section_start_pos, target_position};

    int current_section = 0;
    float section_start_time = 0.0f;

    // P軸軌道オブジェクト
    trajectory_t trajectory_P_core0(Traj::P_MAX_VELOCITY, Traj::P_ACCEL, Traj::P_DECEL, Traj::P_S_CURVE_RATIO,
                                    sections[0].start_pos[1], sections[0].end_pos[1]);
    trajectory_P_core0.calculate_s_curve_trajectory_params();

    for (int i = 0; i < point_count; i++) {
        float time = i * Traj::TRAJECTORY_CONTROL_PERIOD;
        float pos_R, vel_R, acc_R;
        float pos_P, vel_P, acc_P;

        trajectory_R_core0.get_s_curve_state(time, pos_R, vel_R, acc_R);

        // P軸の現在区間で軌道計算
        trajectory_P_core0.get_s_curve_state(time - section_start_time, pos_P, vel_P, acc_P);

        // 区間終端判定（R軸・P軸両方）
        const float* end_pos = sections[current_section].end_pos;
        bool r_reached = (is_forward_R && pos_R >= end_pos[0]) || (!is_forward_R && pos_R <= end_pos[0]);
        bool p_reached = (is_forward_P && pos_P >= end_pos[1]) || (!is_forward_P && pos_P <= end_pos[1]);

        if (r_reached && p_reached && current_section + 1 < section_count) {
            // 次区間へ
            section_start_time = time;
            current_section++;
            is_forward_P = sections[current_section].start_pos[1] < sections[current_section].end_pos[1];
            trajectory_P_core0.set_start_pos(sections[current_section].start_pos[1]);
            trajectory_P_core0.set_end_pos(sections[current_section].end_pos[1]);
            trajectory_P_core0.calculate_s_curve_trajectory_params();

            // 最後の区間に入る場合
            if (current_section + 1 == section_count) {
                float remaining_time = total_time_R - time;
                float new_total_time_P = trajectory_P_core0.get_total_time();
                if (new_total_time_P > remaining_time) {
                    point_count = i + static_cast<int>(new_total_time_P / Traj::TRAJECTORY_CONTROL_PERIOD) + 1;
                    if (point_count > Traj::MAX_TRAJECTORY_POINTS) {
                        g_debug_manager->error("Adjusted trajectory points %d exceed maximum limit!\n", point_count);
                        return false;
                    }
                }
            }
        }

        trajectory_points[i] = {pos_R, vel_R, acc_R, pos_P, vel_P, acc_P};
    }

    // // 軌道点を全て表示
    // printf("Point, Rpos, Rvel, Racc, Ppos, Pvel, Pacc\n");
    // for (int i = 0; i < point_count; i++) {
    //     printf("  %d, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
    //            i,
    //            trajectory_points[i].position_R,
    //            trajectory_points[i].velocity_R,
    //            trajectory_points[i].acceleration_R,
    //            trajectory_points[i].position_P,
    //            trajectory_points[i].velocity_P,
    //            trajectory_points[i].acceleration_P);
    // }

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

    g_debug_manager->debug("Trajectory calculated: %d points, max_time=%.2fs", point_count, point_count * Mc::CONTROL_PERIOD_S);
    g_debug_manager->debug("  R: %.3f → %.3f rad, P: %.3f → %.3f rad",
                           current_position[0], target_position[0], current_position[1], target_position[1]);

    return true;
}

void init_hand_dist() {
    // ポンプの設定
    gpio_init(Hand::PUMP_PIN);
    gpio_set_dir(Hand::PUMP_PIN, GPIO_OUT);
    gpio_init(Hand::SOLENOID_PIN);
    gpio_set_dir(Hand::SOLENOID_PIN, GPIO_OUT);
    gpio_init(QuickArmConfig::PUMP_PIN);
    gpio_set_dir(QuickArmConfig::PUMP_PIN, GPIO_OUT);
    gpio_init(QuickArmConfig::SOLENOID_PIN);
    gpio_set_dir(QuickArmConfig::SOLENOID_PIN, GPIO_OUT);

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    g_debug_manager->info("Initializing Dynamixels (Daisy Chain on UART1)...\n");
    init_crc();
    configure_uart(&UART0, BAUD_RATE);
    configure_uart(&UART1, BAUD_RATE);
    sleep_ms(100);
    write_statusReturnLevel(&UART0, QuickArmConfig::DXL_ID_ROTATE, 0x00);
    write_statusReturnLevel(&UART0, QuickArmConfig::DXL_ID_LIFT, 0x00);
    write_statusReturnLevel(&UART1, Hand::DXL_ID_HAND, 0x00);
    write_statusReturnLevel(&UART1, Hand::DXL_ID_LIFT, 0x00);
    write_statusReturnLevel(&UART1, Dist::DXL_ID_LEFT, 0x00);
    write_statusReturnLevel(&UART1, Dist::DXL_ID_RIGHT, 0x00);
    sleep_ms(100);
    write_dxl_led(&UART0, QuickArmConfig::DXL_ID_ROTATE, true);
    write_dxl_led(&UART0, QuickArmConfig::DXL_ID_LIFT, true);
    write_dxl_led(&UART1, Hand::DXL_ID_HAND, true);
    write_dxl_led(&UART1, Hand::DXL_ID_LIFT, true);
    write_dxl_led(&UART1, Dist::DXL_ID_LEFT, true);
    write_dxl_led(&UART1, Dist::DXL_ID_RIGHT, true);
    sleep_ms(100);
    write_dxl_led(&UART0, QuickArmConfig::DXL_ID_ROTATE, false);
    write_dxl_led(&UART0, QuickArmConfig::DXL_ID_LIFT, false);
    write_dxl_led(&UART1, Hand::DXL_ID_HAND, false);
    write_dxl_led(&UART1, Hand::DXL_ID_LIFT, false);
    write_dxl_led(&UART1, Dist::DXL_ID_LEFT, false);
    write_dxl_led(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(100);
    write_torqueEnable(&UART0, QuickArmConfig::DXL_ID_ROTATE, false);
    write_torqueEnable(&UART0, QuickArmConfig::DXL_ID_LIFT, false);
    write_torqueEnable(&UART1, Hand::DXL_ID_HAND, false);
    write_torqueEnable(&UART1, Hand::DXL_ID_LIFT, false);
    write_torqueEnable(&UART1, Dist::DXL_ID_LEFT, false);
    write_torqueEnable(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(100);
    write_position_Dgain(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::ROTATE_POSITION_D_GAIN);
    write_position_Dgain(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::LIFT_POSITION_D_GAIN);
    sleep_ms(100);
    write_position_Pgain(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::ROTATE_POSITION_P_GAIN);
    write_position_Pgain(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::LIFT_POSITION_P_GAIN);
    sleep_ms(100);
    // write_position_Igain(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::ROTATE_POSITION_I_GAIN);
    write_position_Igain(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::LIFT_POSITION_I_GAIN);
    sleep_ms(100);
    write_dxl_current_limit(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::ROTATE_CURRENT_LIMIT);
    write_dxl_current_limit(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::LIFT_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Hand::DXL_ID_HAND, Hand::HAND_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Hand::DXL_ID_LIFT, Hand::LIFT_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Dist::DXL_ID_LEFT, Dist::DISTURBANCE_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Dist::DXL_ID_RIGHT, Dist::DISTURBANCE_CURRENT_LIMIT);
    sleep_ms(100);
    write_operatingMode(&UART0, QuickArmConfig::DXL_ID_ROTATE, false);
    write_operatingMode(&UART0, QuickArmConfig::DXL_ID_LIFT, false);
    write_operatingMode(&UART1, Hand::DXL_ID_HAND, false);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART1, Hand::DXL_ID_LIFT, false);
    write_operatingMode(&UART1, Dist::DXL_ID_LEFT, false);
    write_operatingMode(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(500);
    write_torqueEnable(&UART0, QuickArmConfig::DXL_ID_ROTATE, true);
    write_torqueEnable(&UART0, QuickArmConfig::DXL_ID_LIFT, true);
    write_torqueEnable(&UART1, Hand::DXL_ID_HAND, true);
    write_torqueEnable(&UART1, Hand::DXL_ID_LIFT, true);
    write_torqueEnable(&UART1, Dist::DXL_ID_LEFT, true);
    write_torqueEnable(&UART1, Dist::DXL_ID_RIGHT, true);
    sleep_ms(500);
    control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::START_HAND_ANGLE);
    control_position(&UART1, Hand::DXL_ID_HAND, Hand::HandAngle::START);
    sleep_ms(500);
    control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::START_UP_ANGLE);
    control_position_multiturn(&UART1, Hand::DXL_ID_LIFT, Hand::LiftAngle::SHOOT_UP);
    sleep_ms(500);
    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_PRE);
    sleep_ms(500);
    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_PRE);
    sleep_ms(100);
    gpio_put(QuickArmConfig::SOLENOID_PIN, 0);  // ソレノイドを吸着状態にする
    gpio_put(QuickArmConfig::PUMP_PIN, 1);
    gpio_put(Hand::SOLENOID_PIN, 0);  // ソレノイドを吸着状態にする
    gpio_put(Hand::PUMP_PIN, 1);
    g_debug_manager->info("hand initialized\n");
}

// 　最速アーム実行
void exe_QuickArm(QuickArm_state_t* quick_arm_state) {
    static absolute_time_t state_start_time = get_absolute_time();
    uint32_t elapsed_ms = absolute_time_diff_us(state_start_time, get_absolute_time()) / 1000;
    switch (*quick_arm_state) {
        case HAND_STANDBY:
            g_debug_manager->debug("hand requested\n");
            *quick_arm_state = CATCHING_POSITION;
            state_start_time = get_absolute_time();
            gpio_put(QuickArmConfig::PUMP_PIN, 1);
            control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::CATCH_ANGLE);
            g_debug_manager->debug("Hand catching position %d\n");

            break;

        case CATCHING_POSITION:
            if (elapsed_ms >= 800) {  // 500
                *quick_arm_state = HAND_DROPPING;
                state_start_time = get_absolute_time();
                g_debug_manager->debug("Hand dropping...\n");
                control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::LOWER_ANGLE);
            }
            break;

        case HAND_DROPPING:
            if (elapsed_ms >= 800) {
                *quick_arm_state = CATCHING_WAIT;
                state_start_time = get_absolute_time();
            }
            break;

        case CATCHING_WAIT:
            if (elapsed_ms >= 300) {
                *quick_arm_state = HAND_LIFTING;
                state_start_time = get_absolute_time();
                control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_LIFT, QuickArmConfig::UPPER_ANGLE);
                g_debug_manager->debug("Hand raising...\n");
            }
            break;

        case HAND_LIFTING:
            if (elapsed_ms >= 800) {
                control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::SHOOTING_ANGLE);
                g_debug_manager->debug("Hand raised, work done.\n");
                state_start_time = get_absolute_time();
                *quick_arm_state = SHOOTING_POSITION;
            }
            break;

        case SHOOTING_POSITION:
            if (elapsed_ms >= 900) {
                *quick_arm_state = HAND_FOLD;
                state_start_time = get_absolute_time();
                gpio_put(QuickArmConfig::PUMP_PIN, 0);      // ポンプ停止
                gpio_put(QuickArmConfig::SOLENOID_PIN, 1);  // ソレノイドを非吸着状態にする
                g_debug_manager->debug("Hand in shooting position, pump off.\n");
            }
            break;

        case HAND_FOLD:
            if (elapsed_ms >= 100) {
                state_start_time = get_absolute_time();
                control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::INTER_POINT);
                g_debug_manager->debug("Hand folding...\n");
                *quick_arm_state = HAND_FINISH;
            }
            break;

        case HAND_FINISH:
            if (elapsed_ms >= 500) {
                control_position_multiturn(&UART0, QuickArmConfig::DXL_ID_ROTATE, QuickArmConfig::FOLD_ANGLE);
                g_moving_quick_arm = false;
                *quick_arm_state = HAND_END;
                g_debug_manager->debug("Hand folded, work done.\n");
            }
            break;

        case HAND_END:
            break;  // 何もしない
    }
}

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, bool* has_work, absolute_time_t* state_start_time, float hand_angle, int32_t lift_angle) {
    uint32_t elapsed_ms = absolute_time_diff_us(*state_start_time, get_absolute_time()) / 1000;
    switch (*hand_state) {
        case HAND_IDLE:
            g_debug_manager->debug("hand requested\n");
            if (!*has_work) {
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
                *hand_state = HAND_LOWERING;
                *state_start_time = get_absolute_time();
                gpio_put(Hand::PUMP_PIN, 1);
                g_debug_manager->debug("Hand lowering...");
                control_position_multiturn(&UART1, Hand::DXL_ID_LIFT, Hand::LiftAngle::FRONT_CATCH);
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
            } else {
                *hand_state = HAND_RELEASE;
                *state_start_time = get_absolute_time();
                gpio_put(Hand::SOLENOID_PIN, 1);
                gpio_put(Hand::PUMP_PIN, 0);
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
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
                *hand_state = HAND_RAISING;
                *state_start_time = get_absolute_time();
                control_position_multiturn(&UART1, Hand::DXL_ID_LIFT, lift_angle);
                g_debug_manager->debug("Hand raising...\n");
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
            }
            break;

        case HAND_RAISING:
            if (elapsed_ms >= 200) {
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
                *has_work = true;
                control_position(&UART1, Hand::DXL_ID_HAND, hand_angle);
                g_debug_manager->debug("Hand raised, work done.\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
            }
            break;

        case HAND_RELEASE:
            if (elapsed_ms >= 150) {
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
                *has_work = false;
                gpio_put(Hand::SOLENOID_PIN, 0);
                gpio_put(Hand::PUMP_PIN, 1);
                control_position(&UART1, Hand::DXL_ID_HAND, hand_angle);
                g_debug_manager->debug("Hand released\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
            }
            break;

        case HAND_WAITING:
            if (elapsed_ms >= 150) {
                *hand_state = HAND_IDLE;
            }
            break;
    }
}

// 妨害展開・最速アームの割り込みハンドラ
void handle_disturbance_trigger() {
    if (g_moving_quick_arm && (g_quickarm_state != HAND_END)) {
        return;  // 高速アーム動作中は無視
    }

    static uint32_t last_button_press_time = 0;
    uint32_t now = time_us_32();
    // 200ms以内の連続した割り込みはチャタリングとみなし無視する
    if (now - last_button_press_time < 200 * 1000) {
        return;
    }
    last_button_press_time = now;

    // 妨害レベルを変化させる
    g_disturbance_level = (g_disturbance_level) % 2 + 1;  // 0→1→2→1→2...
}

// ボタンを押したときのコールバック関数
void button_cb(uint gpio, uint32_t events) {
    if (gpio != BUTTON_PIN || !(events & GPIO_IRQ_EDGE_FALL)) {
        return;  // 関係ないピンやイベントは無視
    }
    gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
    handle_disturbance_trigger();
    gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
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
        float current_time_s = static_cast<float>(absolute_time_diff_us(control_start_time, control_timing.loop_start_time)) / 1000000.0f;

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

        // R軸の慣性を更新
        float updated_inertia_R = Mech::R_EQ_INERTIA + Mech::P_MASS * (motor_position_P * Mech::gear_radius_P - Mech::P_CENTER_OF_MASS) * (motor_position_P * Mech::gear_radius_P - Mech::P_CENTER_OF_MASS);
        updated_inertia_R = clamp(updated_inertia_R, Mech::R_INERTIA_MIN, Mech::R_INERTIA_MAX);  // 慣性の下限・上限を設定
        dob_R.set_inertia(updated_inertia_R);

        // --- 制御計算 ---
        // R軸の制御計算
        error_position_R = updated_inertia_R * Ctrl::R_POSITION_GAIN * (trajectory_target_pos_R - motor_position_R);
        error_velocity_R = updated_inertia_R * Ctrl::R_VELOCITY_GAIN * (trajectory_target_vel_R - motor_velocity_R);
        acceleration_feedforward_R = updated_inertia_R * trajectory_target_accel_R;
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
        // target_current[0] = 0.0f;  // Motor1 (R軸)
        // target_current[1] = 0.0f;  // Motor2 (P軸)

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

        // // LEDを500ms間隔でONにする（Core1が動作していることの確認用）
        // static absolute_time_t last_led_toggle_time = {0};
        // if (absolute_time_diff_us(last_led_toggle_time, get_absolute_time()) >= 500000) {
        //     last_led_toggle_time = get_absolute_time();
        //     gpio_put(LED::ALIVE_PIN, 1);
        // }

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

    sleep_ms(2000);  // 少し待機して安定化

    gpio_init(Mc::PWR_ON_DETECT_PIN);
    gpio_set_dir(Mc::PWR_ON_DETECT_PIN, GPIO_IN);
    gpio_init(SPI1::Encoder::ON_PIN);
    gpio_set_dir(SPI1::Encoder::ON_PIN, GPIO_OUT);
    gpio_put(SPI1::Encoder::ON_PIN, 1);  // ON状態に設定

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

    // ハンド・妨害機構の初期化
    init_hand_dist();

    // ボタンのGPIOと割り込み設定
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // 内部プルアップを有効化
    // 立ち下がりエッジ（ボタンが押されたとき）で割り込み発生
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, button_cb);

    sleep_ms(2000);  // シリアル接続待ち

    // LEDのGPIO初期化
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(LED::ALIVE_PIN);
    gpio_set_dir(LED::ALIVE_PIN, GPIO_OUT);
    gpio_init(LED::ERROR_PIN);
    gpio_set_dir(LED::ERROR_PIN, GPIO_OUT);

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

#if LEFT_SHOOTING_AREA == 1
    static const trajectory_waypoint_t all_waypoints[WAYPOINT_NUM] = {
        // trajectory_waypoint_t(3.480f, (-0.475f+ 0.55ff) / Mech::gear_radius_P, 224.703f, 5749, Traj::PassThroughMode::DIRECT),                                       // ID: 1-1
        // trajectory_waypoint_t(2.647f, (-0.313f+ 0.55ff) / Mech::gear_radius_P, 347.604f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-1
        // trajectory_waypoint_t(3.578f, (-0.417f+ 0.55ff) / Mech::gear_radius_P, 217.407f, 5768, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 1-2
        // trajectory_waypoint_t(2.653f, (-0.228f+ 0.55ff) / Mech::gear_radius_P, 343.209f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),       // ID: l-1
        // trajectory_waypoint_t(3.766f, (-0.338f+ 0.55ff) / Mech::gear_radius_P, 208.527f, 5759, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 1-3
        // trajectory_waypoint_t(2.590f, (-0.316f+ 0.55ff) / Mech::gear_radius_P, 357.714f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-2
        // trajectory_waypoint_t(3.919f, (-0.293f+ 0.55ff) / Mech::gear_radius_P, 199.033f, 5884, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 1-4
        // trajectory_waypoint_t(2.584f, (-0.233f+ 0.55ff) / Mech::gear_radius_P, 356.220f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),       // ID: l-2
        // trajectory_waypoint_t(4.169f, (-0.255f+ 0.55ff) / Mech::gear_radius_P, 184.088f, 5800, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 1-5
        // trajectory_waypoint_t(2.525f, (-0.316f+ 0.55ff) / Mech::gear_radius_P, 362.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-3
        // trajectory_waypoint_t(4.359f, (-0.261f+ 0.55ff) / Mech::gear_radius_P, 173.451f, 5765, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 1-6
        // trajectory_waypoint_t(2.510f, (-0.233f+ 0.55ff) / Mech::gear_radius_P, 365.187f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),       // ID: l-3
        // trajectory_waypoint_t(4.615f, (-0.290f+ 0.55ff) / Mech::gear_radius_P, 157.626f, 5765, Traj::PassThroughMode::INTERMEDIATE_12),                              // ID: 1-7
        // trajectory_waypoint_t(2.470f, (-0.324f+ 0.55ff) / Mech::gear_radius_P, 366.242f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-4
        // trajectory_waypoint_t(4.763f, (-0.334f+ 0.55ff) / Mech::gear_radius_P, 150.769f, 5696, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 1-8
        // trajectory_waypoint_t(2.451f, (-0.244f+ 0.55ff) / Mech::gear_radius_P, 367.033f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),       // ID: l-4
        // trajectory_waypoint_t(4.949f, (-0.422f+ 0.55ff) / Mech::gear_radius_P, 140.220f, 5528, Traj::PassThroughMode::INTERMEDIATE_12),                              // ID: 1-9
        // trajectory_waypoint_t(2.409f, (-0.331f+ 0.55ff) / Mech::gear_radius_P, 373.011f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-5
        // trajectory_waypoint_t(5.047f, (-0.487f+ 0.55ff) / Mech::gear_radius_P, 134.154f, 5466, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 1-10
        // trajectory_waypoint_t(2.378f, (-0.254f+ 0.55ff) / Mech::gear_radius_P, 375.121f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),       // ID: l-5
        // trajectory_waypoint_t(3.373f, (-0.416f+ 0.55ff) / Mech::gear_radius_P, 230.857f, 5745, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-1
        // trajectory_waypoint_t(2.344f, (-0.346f+ 0.55ff) / Mech::gear_radius_P, 371.780f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-6
        // trajectory_waypoint_t(3.473f, (-0.343f+ 0.55ff) / Mech::gear_radius_P, 224.791f, 5769, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-2
        // trajectory_waypoint_t(2.288f, (-0.270f+ 0.55ff) / Mech::gear_radius_P, 371.780f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),       // ID: l-6
        // trajectory_waypoint_t(3.669f, (-0.245f+ 0.55ff) / Mech::gear_radius_P, 213.187f, 5830, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-3
        // trajectory_waypoint_t(2.288f, (-0.362f+ 0.55ff) / Mech::gear_radius_P, 379.165f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-7
        // trajectory_waypoint_t(3.838f, (-0.198f+ 0.55ff) / Mech::gear_radius_P, 203.077f, 5837, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-4
        // trajectory_waypoint_t(2.229f, (-0.288f+ 0.55ff) / Mech::gear_radius_P, 381.802f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),       // ID: l-7
        // trajectory_waypoint_t(4.143f, (-0.160f+ 0.55ff) / Mech::gear_radius_P, 186.549f, 5872, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-5
        // trajectory_waypoint_t(2.238f, (-0.386f+ 0.55ff) / Mech::gear_radius_P, 384.703f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-8
        // trajectory_waypoint_t(4.365f, (-0.158f+ 0.55ff) / Mech::gear_radius_P, 173.363f, 5870, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 2-6
        // trajectory_waypoint_t(2.170f, (-0.302f+ 0.55ff) / Mech::gear_radius_P, 390.066f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),       // ID: l-8
        // trajectory_waypoint_t(4.673f, (-0.195f+ 0.55ff) / Mech::gear_radius_P, 155.341f, 5827, Traj::PassThroughMode::INTERMEDIATE_12),                              // ID: 2-7
        // trajectory_waypoint_t(2.185f, (-0.408f+ 0.55ff) / Mech::gear_radius_P, 393.758f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-9
        // trajectory_waypoint_t(4.842f, (-0.247f+ 0.55ff) / Mech::gear_radius_P, 145.846f, 5758, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 2-8
        // trajectory_waypoint_t(2.112f, (-0.325f+ 0.55ff) / Mech::gear_radius_P, 393.670f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),       // ID: l-9
        // trajectory_waypoint_t(5.043f, (-0.340f+ 0.55ff) / Mech::gear_radius_P, 134.505f, 5671, Traj::PassThroughMode::INTERMEDIATE_12),                              // ID: 2-9
        // trajectory_waypoint_t(2.134f, (-0.418f+ 0.55ff) / Mech::gear_radius_P, 401.670f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-10
        // trajectory_waypoint_t(5.145f, (-0.414f+ 0.55ff) / Mech::gear_radius_P, 128.352f, 5632, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 2-10
        // trajectory_waypoint_t(2.066f, (-0.353f+ 0.55ff) / Mech::gear_radius_P, 405.363f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),       // ID: l-10
        // trajectory_waypoint_t(3.255f, (-0.359f+ 0.55ff) / Mech::gear_radius_P, 238.857f, 5700, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-1
        // trajectory_waypoint_t(2.072f, (-0.439f+ 0.55ff) / Mech::gear_radius_P, 397.187f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-11
        // trajectory_waypoint_t(3.344f, (-0.273f+ 0.55ff) / Mech::gear_radius_P, 228.659f, 5757, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-2
        // trajectory_waypoint_t(2.006f, (-0.385f+ 0.55ff) / Mech::gear_radius_P, 397.538f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),       // ID: l-11
        // trajectory_waypoint_t(3.546f, (-0.165f+ 0.55ff) / Mech::gear_radius_P, 221.099f, 5834, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-3
        // trajectory_waypoint_t(2.027f, (-0.459f+ 0.55ff) / Mech::gear_radius_P, 402.901f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-12
        // trajectory_waypoint_t(3.729f, (-0.108f+ 0.55ff) / Mech::gear_radius_P, 210.549f, 5817, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-4
        // trajectory_waypoint_t(1.972f, (-0.418f+ 0.55ff) / Mech::gear_radius_P, 402.110f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),       // ID: l-12
        // trajectory_waypoint_t(4.110f, (-0.055f+ 0.55ff) / Mech::gear_radius_P, 187.165f, 5808, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-5
        // trajectory_waypoint_t(1.985f, (-0.485f+ 0.55ff) / Mech::gear_radius_P, 410.198f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-13
        // trajectory_waypoint_t(4.398f, (-0.055f+ 0.55ff) / Mech::gear_radius_P, 172.044f, 5802, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-6
        // trajectory_waypoint_t(1.921f, (-0.436f+ 0.55ff) / Mech::gear_radius_P, 410.989f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),       // ID: l-13
        // trajectory_waypoint_t(4.773f, (-0.107f+ 0.55ff) / Mech::gear_radius_P, 149.451f, 5767, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-7
        // trajectory_waypoint_t(1.950f, (-0.507f+ 0.55ff) / Mech::gear_radius_P, 415.121f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: h-14
        // trajectory_waypoint_t(4.967f, (-0.167f+ 0.55ff) / Mech::gear_radius_P, 139.341f, 5717, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 3-8
        // trajectory_waypoint_t(1.895f, (-0.468f+ 0.55ff) / Mech::gear_radius_P, 412.396f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),       // ID: l-14
        // trajectory_waypoint_t(5.167f, (-0.271f+ 0.55ff) / Mech::gear_radius_P, 127.473f, 5610, Traj::PassThroughMode::INTERMEDIATE_12),                              // ID: 3-9
        // trajectory_waypoint_t(1.915f, (-0.533f+ 0.55ff) / Mech::gear_radius_P, 422.857f, (Hand::LiftAngle::SHOOT_UP - 100), Traj::PassThroughMode::INTERMEDIATE_1),  // ID: h-15
        // trajectory_waypoint_t(5.266f, (-0.357f+ 0.55ff) / Mech::gear_radius_P, 122.813f, 5508, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 3-10
        // trajectory_waypoint_t(1.857f, (-0.495f+ 0.55ff) / Mech::gear_radius_P, 423.560f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),       // ID: l-15
        // trajectory_waypoint_t(3.113f, (-0.312f+ 0.55ff) / Mech::gear_radius_P, 244.747f, 5658, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-1
        // trajectory_waypoint_t(2.614f, (-0.141f+ 0.55ff) / Mech::gear_radius_P, 363.077f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-1
        // trajectory_waypoint_t(3.190f, (-0.217f+ 0.55ff) / Mech::gear_radius_P, 241.055f, 5693, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-2
        // trajectory_waypoint_t(2.436f, (-0.150f+ 0.55ff) / Mech::gear_radius_P, 373.363f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-2
        // trajectory_waypoint_t(3.379f, (-0.096f+ 0.55ff) / Mech::gear_radius_P, 229.626f, 5746, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-3
        // trajectory_waypoint_t(2.318f, (-0.171f+ 0.55ff) / Mech::gear_radius_P, 381.890f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-3
        // trajectory_waypoint_t(3.574f, (-0.025f+ 0.55ff) / Mech::gear_radius_P, 218.813f, 5750, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-4
        // trajectory_waypoint_t(2.206f, (-0.199f+ 0.55ff) / Mech::gear_radius_P, 387.429f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-4
        // trajectory_waypoint_t(4.055f, (0.043f+ 0.55ff) / Mech::gear_radius_P, 190.593f, 5753, Traj::PassThroughMode::INTERMEDIATE_1),                                // ID: 4-5
        // trajectory_waypoint_t(2.105f, (-0.226f+ 0.55ff) / Mech::gear_radius_P, 394.198f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-5
        // trajectory_waypoint_t(4.447f, (0.044f+ 0.55ff) / Mech::gear_radius_P, 168.615f, 5727, Traj::PassThroughMode::INTERMEDIATE_1),                                // ID: 4-6
        // trajectory_waypoint_t(2.024f, (-0.263f+ 0.55ff) / Mech::gear_radius_P, 398.945f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-6
        // trajectory_waypoint_t(4.928f, (-0.025f+ 0.55ff) / Mech::gear_radius_P, 141.626f, 5721, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-7
        // trajectory_waypoint_t(1.939f, (-0.286f+ 0.55ff) / Mech::gear_radius_P, 402.813f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-7
        // trajectory_waypoint_t(5.134f, (-0.092f+ 0.55ff) / Mech::gear_radius_P, 130.022f, 5600, Traj::PassThroughMode::INTERMEDIATE_1),                               // ID: 4-8
        // trajectory_waypoint_t(1.873f, (-0.329f+ 0.55ff) / Mech::gear_radius_P, 404.132f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-8
        // trajectory_waypoint_t(5.319f, (-0.220f+ 0.55ff) / Mech::gear_radius_P, 117.187f, 5572, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 4-9
        // trajectory_waypoint_t(1.821f, (-0.336f+ 0.55ff) / Mech::gear_radius_P, 410.022f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID: o-9
        // trajectory_waypoint_t(5.406f, (-0.312f+ 0.55ff) / Mech::gear_radius_P, 113.495f, 5453, Traj::PassThroughMode::INTERMEDIATE_2),                               // ID: 4-10
        // trajectory_waypoint_t(1.763f, (-0.372f+ 0.55ff) / Mech::gear_radius_P, 414.505f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),          // ID(: o-10

        trajectory_waypoint_t(3.480f, (-0.475f + 0.55f) / Mech::gear_radius_P, 224.703f, 5749, Traj::PassThroughMode::DIRECT),                                  // ID: 1-1
        trajectory_waypoint_t(2.647f, (-0.313f + 0.55f) / Mech::gear_radius_P, 347.604f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-1
        trajectory_waypoint_t(3.578f, (-0.417f + 0.55f) / Mech::gear_radius_P, 217.407f, 5768, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-2
        trajectory_waypoint_t(2.653f, (-0.228f + 0.55f) / Mech::gear_radius_P, 343.209f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-1
        trajectory_waypoint_t(3.766f, (-0.338f + 0.55f) / Mech::gear_radius_P, 208.527f, 5759, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-3
        trajectory_waypoint_t(2.590f, (-0.316f + 0.55f) / Mech::gear_radius_P, 357.714f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-2
        trajectory_waypoint_t(3.919f, (-0.293f + 0.55f) / Mech::gear_radius_P, 199.033f, 5884, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-4
        trajectory_waypoint_t(2.584f, (-0.233f + 0.55f) / Mech::gear_radius_P, 356.220f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-2
        trajectory_waypoint_t(4.177f, (-0.261f + 0.55f) / Mech::gear_radius_P, 184.088f, 5755, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-5
        trajectory_waypoint_t(2.525f, (-0.316f + 0.55f) / Mech::gear_radius_P, 362.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-3
        trajectory_waypoint_t(4.359f, (-0.261f + 0.55f) / Mech::gear_radius_P, 173.451f, 5765, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-6
        trajectory_waypoint_t(2.510f, (-0.233f + 0.55f) / Mech::gear_radius_P, 365.187f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-3
        trajectory_waypoint_t(4.928f, (-0.025f + 0.55f) / Mech::gear_radius_P, 141.626f, 5721, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-7
        trajectory_waypoint_t(2.470f, (-0.324f + 0.55f) / Mech::gear_radius_P, 366.242f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-4
        trajectory_waypoint_t(5.134f, (-0.092f + 0.55f) / Mech::gear_radius_P, 130.022f, 5600, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-8
        trajectory_waypoint_t(2.451f, (-0.244f + 0.55f) / Mech::gear_radius_P, 367.033f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-4
        trajectory_waypoint_t(5.319f, (-0.220f + 0.55f) / Mech::gear_radius_P, 117.187f, 5572, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 4-9
        trajectory_waypoint_t(2.409f, (-0.331f + 0.55f) / Mech::gear_radius_P, 373.011f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-5
        trajectory_waypoint_t(5.406f, (-0.312f + 0.55f) / Mech::gear_radius_P, 113.495f, 5453, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 4-10
        trajectory_waypoint_t(2.378f, (-0.254f + 0.55f) / Mech::gear_radius_P, 375.121f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-5
        trajectory_waypoint_t(3.373f, (-0.416f + 0.55f) / Mech::gear_radius_P, 230.857f, 5745, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-1
        trajectory_waypoint_t(2.344f, (-0.346f + 0.55f) / Mech::gear_radius_P, 371.780f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-6
        trajectory_waypoint_t(3.473f, (-0.343f + 0.55f) / Mech::gear_radius_P, 224.791f, 5769, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-2
        trajectory_waypoint_t(2.288f, (-0.270f + 0.55f) / Mech::gear_radius_P, 371.780f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-6
        trajectory_waypoint_t(3.669f, (-0.245f + 0.55f) / Mech::gear_radius_P, 213.187f, 5830, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-3
        trajectory_waypoint_t(2.288f, (-0.362f + 0.55f) / Mech::gear_radius_P, 379.165f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-7
        trajectory_waypoint_t(3.838f, (-0.198f + 0.55f) / Mech::gear_radius_P, 203.077f, 5837, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-4
        trajectory_waypoint_t(2.229f, (-0.288f + 0.55f) / Mech::gear_radius_P, 381.802f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-7
        trajectory_waypoint_t(4.143f, (-0.160f + 0.55f) / Mech::gear_radius_P, 186.549f, 5872, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-5
        trajectory_waypoint_t(2.238f, (-0.386f + 0.55f) / Mech::gear_radius_P, 384.703f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-8
        trajectory_waypoint_t(4.365f, (-0.158f + 0.55f) / Mech::gear_radius_P, 173.363f, 5870, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-6
        trajectory_waypoint_t(2.170f, (-0.302f + 0.55f) / Mech::gear_radius_P, 390.066f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-8
        trajectory_waypoint_t(4.673f, (-0.195f + 0.55f) / Mech::gear_radius_P, 155.341f, 5827, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 2-7
        trajectory_waypoint_t(2.185f, (-0.408f + 0.55f) / Mech::gear_radius_P, 393.758f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-9
        trajectory_waypoint_t(4.842f, (-0.247f + 0.55f) / Mech::gear_radius_P, 145.846f, 5758, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 2-8
        trajectory_waypoint_t(2.112f, (-0.325f + 0.55f) / Mech::gear_radius_P, 393.670f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-9
        trajectory_waypoint_t(5.043f, (-0.340f + 0.55f) / Mech::gear_radius_P, 134.505f, 5671, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 2-9
        trajectory_waypoint_t(2.134f, (-0.418f + 0.55f) / Mech::gear_radius_P, 401.670f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-10
        trajectory_waypoint_t(5.145f, (-0.414f + 0.55f) / Mech::gear_radius_P, 128.352f, 5632, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 2-10
        trajectory_waypoint_t(2.066f, (-0.353f + 0.55f) / Mech::gear_radius_P, 405.363f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-10
        trajectory_waypoint_t(3.255f, (-0.359f + 0.55f) / Mech::gear_radius_P, 238.857f, 5700, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-1
        trajectory_waypoint_t(2.072f, (-0.439f + 0.55f) / Mech::gear_radius_P, 397.187f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-11
        trajectory_waypoint_t(3.344f, (-0.273f + 0.55f) / Mech::gear_radius_P, 228.659f, 5757, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-2
        trajectory_waypoint_t(2.006f, (-0.385f + 0.55f) / Mech::gear_radius_P, 397.538f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-11
        trajectory_waypoint_t(3.546f, (-0.165f + 0.55f) / Mech::gear_radius_P, 221.099f, 5834, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-3
        trajectory_waypoint_t(2.027f, (-0.459f + 0.55f) / Mech::gear_radius_P, 402.901f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-12
        trajectory_waypoint_t(3.729f, (-0.108f + 0.55f) / Mech::gear_radius_P, 210.549f, 5817, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-4
        trajectory_waypoint_t(1.972f, (-0.418f + 0.55f) / Mech::gear_radius_P, 402.110f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-12
        trajectory_waypoint_t(4.110f, (-0.055f + 0.55f) / Mech::gear_radius_P, 187.165f, 5808, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-5
        trajectory_waypoint_t(1.985f, (-0.485f + 0.55f) / Mech::gear_radius_P, 410.198f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-13
        trajectory_waypoint_t(4.398f, (-0.055f + 0.55f) / Mech::gear_radius_P, 172.044f, 5802, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-6
        trajectory_waypoint_t(1.921f, (-0.436f + 0.55f) / Mech::gear_radius_P, 410.989f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-13
        trajectory_waypoint_t(4.773f, (-0.107f + 0.55f) / Mech::gear_radius_P, 149.451f, 5767, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-7
        trajectory_waypoint_t(1.950f, (-0.507f + 0.55f) / Mech::gear_radius_P, 415.121f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-14
        trajectory_waypoint_t(4.967f, (-0.167f + 0.55f) / Mech::gear_radius_P, 139.341f, 5717, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-8
        trajectory_waypoint_t(1.895f, (-0.468f + 0.55f) / Mech::gear_radius_P, 412.396f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-14
        trajectory_waypoint_t(5.167f, (-0.271f + 0.55f) / Mech::gear_radius_P, 127.473f, 5610, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 3-9
        trajectory_waypoint_t(1.915f, (-0.533f + 0.55f) / Mech::gear_radius_P, 422.857f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-15
        trajectory_waypoint_t(5.266f, (-0.357f + 0.55f) / Mech::gear_radius_P, 122.813f, 5508, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 3-10
        trajectory_waypoint_t(1.857f, (-0.495f + 0.55f) / Mech::gear_radius_P, 423.560f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-15
        trajectory_waypoint_t(3.113f, (-0.312f + 0.55f) / Mech::gear_radius_P, 244.747f, 5658, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-1
        trajectory_waypoint_t(2.614f, (-0.141f + 0.55f) / Mech::gear_radius_P, 363.077f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-1
        trajectory_waypoint_t(3.190f, (-0.217f + 0.55f) / Mech::gear_radius_P, 241.055f, 5693, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-2
        trajectory_waypoint_t(2.436f, (-0.150f + 0.55f) / Mech::gear_radius_P, 373.363f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-2
        trajectory_waypoint_t(3.379f, (-0.096f + 0.55f) / Mech::gear_radius_P, 229.626f, 5746, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-3
        trajectory_waypoint_t(2.318f, (-0.171f + 0.55f) / Mech::gear_radius_P, 381.890f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-3
        trajectory_waypoint_t(3.574f, (-0.025f + 0.55f) / Mech::gear_radius_P, 218.813f, 5750, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-4
        trajectory_waypoint_t(2.206f, (-0.199f + 0.55f) / Mech::gear_radius_P, 387.429f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-4
        trajectory_waypoint_t(4.055f, (0.043f + 0.55f) / Mech::gear_radius_P, 190.593f, 5753, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-5
        trajectory_waypoint_t(2.105f, (-0.226f + 0.55f) / Mech::gear_radius_P, 394.198f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-5
        trajectory_waypoint_t(4.447f, (0.044f + 0.55f) / Mech::gear_radius_P, 168.615f, 5727, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-6
        trajectory_waypoint_t(2.024f, (-0.263f + 0.55f) / Mech::gear_radius_P, 398.945f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-6
        trajectory_waypoint_t(4.615f, (-0.290f + 0.55f) / Mech::gear_radius_P, 157.626f, 5765, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-7
        trajectory_waypoint_t(1.939f, (-0.286f + 0.55f) / Mech::gear_radius_P, 402.813f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-7
        trajectory_waypoint_t(4.763f, (-0.334f + 0.55f) / Mech::gear_radius_P, 150.769f, 5696, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-8
        trajectory_waypoint_t(1.873f, (-0.329f + 0.55f) / Mech::gear_radius_P, 404.132f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-8
        trajectory_waypoint_t(4.949f, (-0.422f + 0.55f) / Mech::gear_radius_P, 140.220f, 5528, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-9
        trajectory_waypoint_t(1.821f, (-0.336f + 0.55f) / Mech::gear_radius_P, 410.022f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-9
        trajectory_waypoint_t(5.047f, (-0.487f + 0.55f) / Mech::gear_radius_P, 134.154f, 5466, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-10
        trajectory_waypoint_t(1.763f, (-0.372f + 0.55f) / Mech::gear_radius_P, 414.505f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: o-10

    };
#else
    static const trajectory_waypoint_t all_waypoints[WAYPOINT_NUM] = {
        trajectory_waypoint_t(4.026f, -0.343f / Mech::gear_radius_P, 128.879f, 5648, Traj::PassThroughMode::DIRECT),                                  // ID: 2-10
        trajectory_waypoint_t(4.766f, -0.241f / Mech::gear_radius_P, 371.692f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-1
        trajectory_waypoint_t(3.929f, -0.271f / Mech::gear_radius_P, 134.505f, 5625, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 2-9
        trajectory_waypoint_t(4.738f, -0.150f / Mech::gear_radius_P, 374.242f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-1
        trajectory_waypoint_t(3.726f, -0.173f / Mech::gear_radius_P, 146.462f, 5788, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 2-8
        trajectory_waypoint_t(4.826f, -0.241f / Mech::gear_radius_P, 363.516f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-2
        trajectory_waypoint_t(3.560f, -0.129f / Mech::gear_radius_P, 155.077f, 5807, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 2-7
        trajectory_waypoint_t(4.819f, -0.150f / Mech::gear_radius_P, 364.044f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-2
        trajectory_waypoint_t(3.247f, -0.085f / Mech::gear_radius_P, 172.484f, 5862, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-6
        trajectory_waypoint_t(4.889f, -0.241f / Mech::gear_radius_P, 356.484f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-3
        trajectory_waypoint_t(3.028f, -0.085f / Mech::gear_radius_P, 187.341f, 5868, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-5
        trajectory_waypoint_t(4.892f, -0.161f / Mech::gear_radius_P, 352.440f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-3
        trajectory_waypoint_t(2.722f, -0.120f / Mech::gear_radius_P, 203.429f, 5875, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-4
        trajectory_waypoint_t(4.948f, -0.249f / Mech::gear_radius_P, 346.022f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-4
        trajectory_waypoint_t(2.552f, -0.167f / Mech::gear_radius_P, 214.154f, 5833, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-3
        trajectory_waypoint_t(4.958f, -0.173f / Mech::gear_radius_P, 345.407f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-4
        trajectory_waypoint_t(2.356f, -0.268f / Mech::gear_radius_P, 223.736f, 5780, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-2
        trajectory_waypoint_t(5.007f, -0.249f / Mech::gear_radius_P, 341.363f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-5
        trajectory_waypoint_t(2.255f, -0.335f / Mech::gear_radius_P, 229.451f, 5786, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 2-1
        trajectory_waypoint_t(5.037f, -0.178f / Mech::gear_radius_P, 342.681f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),  // ID: l-5
        trajectory_waypoint_t(4.142f, -0.287f / Mech::gear_radius_P, 120.791f, 5512, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 3-10
        trajectory_waypoint_t(5.076f, -0.266f / Mech::gear_radius_P, 346.989f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-6
        trajectory_waypoint_t(4.050f, -0.200f / Mech::gear_radius_P, 125.978f, 5627, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 3-9
        trajectory_waypoint_t(5.113f, -0.200f / Mech::gear_radius_P, 343.648f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-6
        trajectory_waypoint_t(3.848f, -0.092f / Mech::gear_radius_P, 139.253f, 5754, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-8
        trajectory_waypoint_t(5.128f, -0.282f / Mech::gear_radius_P, 338.198f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-7
        trajectory_waypoint_t(3.659f, -0.028f / Mech::gear_radius_P, 149.363f, 5767, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-7
        trajectory_waypoint_t(5.180f, -0.217f / Mech::gear_radius_P, 333.978f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-7
        trajectory_waypoint_t(3.279f, 0.013f / Mech::gear_radius_P, 171.341f, 5807, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 3-6
        trajectory_waypoint_t(5.184f, -0.296f / Mech::gear_radius_P, 331.516f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-8
        trajectory_waypoint_t(2.998f, 0.013f / Mech::gear_radius_P, 188.484f, 5779, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 3-5
        trajectory_waypoint_t(5.234f, -0.245f / Mech::gear_radius_P, 329.495f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-8
        trajectory_waypoint_t(2.616f, -0.027f / Mech::gear_radius_P, 209.319f, 5807, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-4
        trajectory_waypoint_t(5.230f, -0.316f / Mech::gear_radius_P, 320.527f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-9
        trajectory_waypoint_t(2.431f, -0.091f / Mech::gear_radius_P, 221.363f, 5797, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-3
        trajectory_waypoint_t(5.287f, -0.262f / Mech::gear_radius_P, 324.132f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-9
        trajectory_waypoint_t(2.227f, -0.202f / Mech::gear_radius_P, 233.319f, 5769, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-2
        trajectory_waypoint_t(5.285f, -0.337f / Mech::gear_radius_P, 313.055f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-10
        trajectory_waypoint_t(2.135f, -0.285f / Mech::gear_radius_P, 236.220f, 5741, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 3-1
        trajectory_waypoint_t(5.344f, -0.284f / Mech::gear_radius_P, 318.418f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),  // ID: l-10
        trajectory_waypoint_t(4.285f, -0.242f / Mech::gear_radius_P, 113.231f, 5499, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 4-10
        trajectory_waypoint_t(5.336f, -0.368f / Mech::gear_radius_P, 320.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-11
        trajectory_waypoint_t(4.205f, -0.146f / Mech::gear_radius_P, 119.209f, 5524, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 4-9
        trajectory_waypoint_t(5.403f, -0.314f / Mech::gear_radius_P, 319.824f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-11
        trajectory_waypoint_t(4.009f, -0.017f / Mech::gear_radius_P, 129.231f, 5680, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-8
        trajectory_waypoint_t(5.380f, -0.397f / Mech::gear_radius_P, 313.670f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-12
        trajectory_waypoint_t(3.812f, 0.052f / Mech::gear_radius_P, 139.956f, 5736, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-7
        trajectory_waypoint_t(5.441f, -0.336f / Mech::gear_radius_P, 314.110f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-12
        trajectory_waypoint_t(3.329f, 0.118f / Mech::gear_radius_P, 168.176f, 5762, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-6
        trajectory_waypoint_t(5.417f, -0.414f / Mech::gear_radius_P, 307.516f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-13
        trajectory_waypoint_t(2.935f, 0.118f / Mech::gear_radius_P, 192.703f, 5779, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-5
        trajectory_waypoint_t(5.487f, -0.364f / Mech::gear_radius_P, 306.022f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-13
        trajectory_waypoint_t(2.456f, 0.047f / Mech::gear_radius_P, 218.813f, 5773, Traj::PassThroughMode::INTERMEDIATE_1),                           // ID: 4-4
        trajectory_waypoint_t(5.452f, -0.434f / Mech::gear_radius_P, 300.308f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-14
        trajectory_waypoint_t(2.261f, -0.018f / Mech::gear_radius_P, 229.099f, 5771, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-3
        trajectory_waypoint_t(5.522f, -0.394f / Mech::gear_radius_P, 298.110f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-14
        trajectory_waypoint_t(2.074f, -0.139f / Mech::gear_radius_P, 240.264f, 5755, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-2
        trajectory_waypoint_t(5.485f, -0.459f / Mech::gear_radius_P, 293.714f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),     // ID: h-15
        trajectory_waypoint_t(2.001f, -0.235f / Mech::gear_radius_P, 246.945f, 5654, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 4-1
        trajectory_waypoint_t(5.562f, -0.424f / Mech::gear_radius_P, 292.835f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),  // ID: l-15
        trajectory_waypoint_t(3.927f, -0.416f / Mech::gear_radius_P, 133.890f, 5632, Traj::PassThroughMode::INTERMEDIATE_12),                         // ID: 1-10
        trajectory_waypoint_t(4.842f, -0.079f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-1
        trajectory_waypoint_t(3.824f, -0.339f / Mech::gear_radius_P, 138.549f, 5748, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-9
        trajectory_waypoint_t(4.897f, -0.080f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-2
        trajectory_waypoint_t(3.639f, -0.261f / Mech::gear_radius_P, 150.066f, 5769, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-8
        trajectory_waypoint_t(4.945f, -0.081f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-3
        trajectory_waypoint_t(3.491f, -0.214f / Mech::gear_radius_P, 160.000f, 5818, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-7
        trajectory_waypoint_t(4.964f, -0.094f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-4
        trajectory_waypoint_t(3.232f, -0.185f / Mech::gear_radius_P, 174.066f, 5863, Traj::PassThroughMode::INTERMEDIATE_2),                          // ID: 1-6
        trajectory_waypoint_t(5.032f, -0.104f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-5
        trajectory_waypoint_t(3.051f, -0.185f / Mech::gear_radius_P, 185.582f, 5901, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-5
        trajectory_waypoint_t(5.092f, -0.113f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-6
        trajectory_waypoint_t(2.794f, -0.222f / Mech::gear_radius_P, 200.000f, 5885, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-4
        trajectory_waypoint_t(5.148f, -0.124f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-7
        trajectory_waypoint_t(2.643f, -0.255f / Mech::gear_radius_P, 208.703f, 5869, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-3
        trajectory_waypoint_t(5.217f, -0.139f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-8
        trajectory_waypoint_t(2.456f, -0.337f / Mech::gear_radius_P, 219.868f, 5780, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-2
        trajectory_waypoint_t(5.289f, -0.161f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-9
        trajectory_waypoint_t(2.360f, -0.408f / Mech::gear_radius_P, 226.198f, 5740, Traj::PassThroughMode::INTERMEDIATE_1),                          // ID: 1-1
        trajectory_waypoint_t(5.334f, -0.196f / Mech::gear_radius_P, 0.000f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),       // ID: o-10
    };
#endif
    TrajectorySequenceManager* seq_manager = new TrajectorySequenceManager(g_debug_manager);
    seq_manager->setup_sequence(all_waypoints, WAYPOINT_NUM);

    // ハンド状態管理用ローカル変数
    hand_state_t hand_state = HAND_IDLE;
    bool has_work = false;
    absolute_time_t hand_timer = get_absolute_time();

    int prev_disturbance_level = -1;  // 前回の妨害レベルを保持

    while (1) {
        // 妨害の展開
        if (prev_disturbance_level != g_disturbance_level) {
            switch (g_disturbance_level) {
                case 0:  // 初期状態
                    sleep_us(1);
                    continue;  // 何もしない
                case 1:        // 1段階目
                    g_debug_manager->info("Disturbance deploying to 1st stage.");
                    g_moving_quick_arm = true;  // 高速アーム動作中フラグを立てる
                    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_1ST);
                    sleep_ms(100);
                    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_1ST);
                    sleep_ms(1);
                    break;
                case 2:  // 2段階目
                    g_debug_manager->info("Disturbance deploying to 2nd stage.");
                    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_2ND);
                    sleep_ms(100);
                    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_2ND);
                    sleep_ms(1);
                    break;
            }
            prev_disturbance_level = g_disturbance_level;  // 状態を更新
        }

        // 高速アーム処理
        if (g_moving_quick_arm && (g_quickarm_state != QuickArm_state_t::HAND_END)) {
            exe_QuickArm(&g_quickarm_state);
            sleep_ms(100);
            continue;
        }

        // シューティングエリアのサーボを動かす指示が来てから1秒後に1秒間動かして戻す
        if (is_moving_shooting_servo) {
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(g_last_shoot_servo_time, now) >= 1'000'000 &&
                absolute_time_diff_us(g_last_shoot_servo_time, now) < 2'000'000) {
                g_debug_manager->debug("Set shooting servo angle to correction angle.");
                printf("Set shooting servo angle to correction angle.\n");
                shooting_servo.set_angle(ShootingConfig::CORRECTION_ANGLE);
            } else if (absolute_time_diff_us(g_last_shoot_servo_time, now) >= 2'000'000) {
                g_debug_manager->debug("Set shooting servo angle to idle angle.");
                printf("Set shooting servo angle to idle angle.\n");
                shooting_servo.set_angle(ShootingConfig::IDLE_ANGLE);
                is_moving_shooting_servo = false;
            }
        }

        // FIFOから同期信号を待機（ブロッキング）
        uint32_t signal = multicore_fifo_pop_blocking();

        // 同期信号を受信したら処理を実行
        if (signal == Mc::SYNC_SIGNAL) {
            core0_loop_count++;

            // 軌道状態機械による処理
            auto try_start_next_trajectory = [&]() {
                if (seq_manager->is_sequence_active()) {
                    float target_position[2];
                    TrajectoryConfig::PassThroughMode pass_through_mode;
                    if (seq_manager->get_next_waypoint(target_position, pass_through_mode)) {
                        float current_position[2];
                        mutex_enter_blocking(&g_state_mutex);
                        current_position[0] = g_robot_state.current_position_R;
                        current_position[1] = g_robot_state.current_position_P;
                        mutex_exit(&g_state_mutex);

                        float intermediate_pos1[2] = {NAN, NAN};
                        float intermediate_pos2[2] = {NAN, NAN};
                        float intermediate_pos3[2] = {NAN, NAN};
                        if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_1) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_2) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_12) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_21) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_1U1) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U11) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_1U2) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U21) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_1U3) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U31) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U12) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U22) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U32) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_12U1) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U121) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_12U2) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U221) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_2[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_12U3) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::INTERMEDIATE_U321) {
                            intermediate_pos1[0] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[0];
                            intermediate_pos1[1] = TrajectoryConfig::INTERMEDIATE_POS_UNDER_3[1];
                            intermediate_pos2[0] = TrajectoryConfig::INTERMEDIATE_POS_2[0];
                            intermediate_pos2[1] = TrajectoryConfig::INTERMEDIATE_POS_2[1];
                            intermediate_pos3[0] = TrajectoryConfig::INTERMEDIATE_POS_1[0];
                            intermediate_pos3[1] = TrajectoryConfig::INTERMEDIATE_POS_1[1];
                        } else if (pass_through_mode == TrajectoryConfig::PassThroughMode::DIRECT) {
                            // DIRECTの場合は中間点なし
                        } else {
                            g_debug_manager->error("Unknown pass-through mode");
                        }

                        if (calculate_trajectory_core0(current_position, target_position, intermediate_pos1, intermediate_pos2, intermediate_pos3)) {
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
                    hand_tick(&hand_state, &has_work, &hand_timer, all_waypoints[seq_index].end_effector_angle, all_waypoints[seq_index].end_effector_height);
                    if (hand_state == HAND_IDLE) {
                        try_start_next_trajectory();
                        move_shooting_servo();  // シューティングエリア用サーボを動かす
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

        // // LEDを700ms周期で消灯（Core1が停止していることの確認用）
        // static absolute_time_t last_led_off_time = {0};
        // if (absolute_time_diff_us(last_led_off_time, get_absolute_time()) >= 700'000) {
        //     last_led_off_time = get_absolute_time();
        //     gpio_put(LED::ALIVE_PIN, 0);
        // }
    }

    // gpio_put(SOLENOID_PIN1, 0);  // ソレノイドを非吸着状態にする
    gpio_put(Hand::PUMP_PIN, 0);  // ポンプを停止
    return 0;
}
