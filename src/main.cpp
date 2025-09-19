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
    .spi_port = spi1,                    // SPI1を使用
    .baudrate = SPI1::BAUDRATE_DEFAULT,  // 初期値 2MHz（後で変更する）
    .pin_miso = SPI1::MISO_PIN,
    .pin_cs = {SPI1::MCP25625::CS_PIN, SPI1::Encoder::R_PIN, SPI1::Encoder::P_PIN},  // CSピン3つ（CAN、回転エンコーダ、直動エンコーダ）
    .num_cs_pins = 3,                                                                // CSピン数
    .pin_sck = SPI1::SCK_PIN,
    .pin_mosi = SPI1::MOSI_PIN,
    .pin_rst = -1  // MCP25625用リセットピン
};

// 妨害展開レベル (0: 初期状態, 1: 1段階目, 2: 2段階目)
volatile int g_disturbance_level = 0;

// MCP25625オブジェクトを作成（CAN SPI設定を使用）
mcp25625_t can(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_cs[0], SPI1_CONFIG.pin_rst);

// AMT223-V エンコーダマネージャを作成
AMT223V_Manager encoder_manager(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_miso, SPI1_CONFIG.pin_sck, SPI1_CONFIG.pin_mosi);

// RoboMasterモータオブジェクト
robomaster_motor_t motor1(&can, 1, Mech::gear_ratio_R);  // motor_id=1
robomaster_motor_t motor2(&can, 2, Mech::gear_ratio_P);  // motor_id=2

Servo shooting_servo(ShootingConfig::SERVO_PIN);

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

    sleep_ms(1000);  // GPIO初期化後の安定化待ち

    // Dynamixelの設定
    g_debug_manager->info("Initializing Dynamixels (Daisy Chain on UART1)...\n");
    init_crc();
    configure_uart(&UART1, BAUD_RATE);
    sleep_ms(100);
    write_statusReturnLevel(&UART1, Hand::DXL_ID1, 0x00);
    write_statusReturnLevel(&UART1, Hand::DXL_ID2, 0x00);
    write_statusReturnLevel(&UART1, Dist::DXL_ID_LEFT, 0x00);
    write_statusReturnLevel(&UART1, Dist::DXL_ID_RIGHT, 0x00);
    sleep_ms(100);
    write_dxl_led(&UART1, Hand::DXL_ID1, true);
    write_dxl_led(&UART1, Hand::DXL_ID2, true);
    write_dxl_led(&UART1, Dist::DXL_ID_LEFT, true);
    write_dxl_led(&UART1, Dist::DXL_ID_RIGHT, true);
    sleep_ms(1000);
    write_dxl_led(&UART1, Hand::DXL_ID1, false);
    write_dxl_led(&UART1, Hand::DXL_ID2, false);
    write_dxl_led(&UART1, Dist::DXL_ID_LEFT, false);
    write_dxl_led(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(100);
    write_torqueEnable(&UART1, Hand::DXL_ID1, false);
    write_torqueEnable(&UART1, Hand::DXL_ID2, false);
    write_torqueEnable(&UART1, Dist::DXL_ID_LEFT, false);
    write_torqueEnable(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(100);
    write_dxl_current_limit(&UART1, Hand::DXL_ID1, Hand::HAND_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Hand::DXL_ID2, Hand::LIFT_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Dist::DXL_ID_LEFT, Dist::DISTURBANCE_CURRENT_LIMIT);
    write_dxl_current_limit(&UART1, Dist::DXL_ID_RIGHT, Dist::DISTURBANCE_CURRENT_LIMIT);
    sleep_ms(100);
    write_operatingMode(&UART1, Hand::DXL_ID1, false);  // false : 位置制御, true : 拡張位置制御(マルチターン)
    write_operatingMode(&UART1, Hand::DXL_ID2, false);
    write_operatingMode(&UART1, Dist::DXL_ID_LEFT, false);
    write_operatingMode(&UART1, Dist::DXL_ID_RIGHT, false);
    sleep_ms(1000);
    write_torqueEnable(&UART1, Hand::DXL_ID1, true);
    write_torqueEnable(&UART1, Hand::DXL_ID2, true);
    write_torqueEnable(&UART1, Dist::DXL_ID_LEFT, true);
    write_torqueEnable(&UART1, Dist::DXL_ID_RIGHT, true);
    sleep_ms(500);
    control_position(&UART1, Hand::DXL_ID1, Hand::HandAngle::START);
    sleep_ms(500);
    control_position_multiturn(&UART1, Hand::DXL_ID2, Hand::LiftAngle::SHOOT_UP);
    sleep_ms(500);
    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_PRE);
    sleep_ms(500);
    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_PRE);
    sleep_ms(1000);
    gpio_put(Hand::SOLENOID_PIN, 0);  // ソレノイドを吸着状態にする
    gpio_put(Hand::PUMP_PIN, 1);
    g_debug_manager->info("hand initialized\n");
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
                control_position_multiturn(&UART1, Hand::DXL_ID2, Hand::LiftAngle::FRONT_CATCH);
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
                control_position_multiturn(&UART1, Hand::DXL_ID2, lift_angle);
                g_debug_manager->debug("Hand raising...\n");
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
            }
            break;

        case HAND_RAISING:
            if (elapsed_ms >= 200) {
                gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, false);
                *has_work = true;
                control_position(&UART1, Hand::DXL_ID1, hand_angle);
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
                control_position(&UART1, Hand::DXL_ID1, hand_angle);
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

    static const trajectory_waypoint_t all_waypoints[WAYPOINT_NUM] = {
        // 一番奥側ロボットから見て左から右へ
        trajectory_waypoint_t(3.489f, -0.527f / Mech::gear_radius_P, 224.703f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::DIRECT),  // 5272
        trajectory_waypoint_t(2.659f, -0.385f / Mech::gear_radius_P, 347.165f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.577f, -0.461f / Mech::gear_radius_P, 218.198f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.670f, -0.293f / Mech::gear_radius_P, 347.780f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.756f, -0.385f / Mech::gear_radius_P, 208.000f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.609f, -0.385f / Mech::gear_radius_P, 358.154f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.902f, -0.345f / Mech::gear_radius_P, 198.418f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.610f, -0.293f / Mech::gear_radius_P, 353.143f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.176f, -0.307f / Mech::gear_radius_P, 186.374f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.549f, -0.385f / Mech::gear_radius_P, 361.319f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.360f, -0.309f / Mech::gear_radius_P, 173.890f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),  // ここから角柱にぶつかる
        trajectory_waypoint_t(2.546f, -0.293f / Mech::gear_radius_P, 364.308f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.615f, -0.346f / Mech::gear_radius_P, 158.681f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.462f, -0.395f / Mech::gear_radius_P, 7.341f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.750f, -0.386f / Mech::gear_radius_P, 150.330f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.473f, -0.303f / Mech::gear_radius_P, 373.714f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),

        trajectory_waypoint_t(4.942f, -0.471f / Mech::gear_radius_P, 137.758f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U12),
        trajectory_waypoint_t(2.409f, -0.402f / Mech::gear_radius_P, 14.374f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.069f, -0.527f / Mech::gear_radius_P, 133.011f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.399f, -0.318f / Mech::gear_radius_P, 381.626f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U1),

        // 2行目
        trajectory_waypoint_t(3.378f, -0.460f / Mech::gear_radius_P, 228.923f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U11),
        trajectory_waypoint_t(2.336f, -0.415f / Mech::gear_radius_P, 7.429f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.477f, -0.398f / Mech::gear_radius_P, 222.506f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.309f, -0.327f / Mech::gear_radius_P, 373.099f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),

        trajectory_waypoint_t(3.663f, -0.297f / Mech::gear_radius_P, 212.659f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U21),
        trajectory_waypoint_t(2.285f, -0.436f / Mech::gear_radius_P, 15.692f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.850f, -0.250f / Mech::gear_radius_P, 202.637f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.249f, -0.339f / Mech::gear_radius_P, 380.571f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),

        trajectory_waypoint_t(4.157f, -0.210f / Mech::gear_radius_P, 184.791f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U21),
        trajectory_waypoint_t(2.231f, -0.447f / Mech::gear_radius_P, 24.308f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.385f, -0.210f / Mech::gear_radius_P, 173.187f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.198f, -0.372f / Mech::gear_radius_P, 390.154f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),

        trajectory_waypoint_t(4.697f, -0.251f / Mech::gear_radius_P, 153.758f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U22),  // ここから角柱にぶつかる
        trajectory_waypoint_t(2.175f, -0.461f / Mech::gear_radius_P, 35.560f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.842f, -0.299f / Mech::gear_radius_P, 146.681f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.140f, -0.390f / Mech::gear_radius_P, 396.484f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),

        trajectory_waypoint_t(5.038f, -0.395f / Mech::gear_radius_P, 134.857f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U22),
        trajectory_waypoint_t(2.136f, -0.482f / Mech::gear_radius_P, 38.374f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.168f, -0.467f / Mech::gear_radius_P, 128.527f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.089f, -0.414f / Mech::gear_radius_P, 403.429f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U2),

        // 3行目
        trajectory_waypoint_t(3.293f, -0.397f / Mech::gear_radius_P, 236.747f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U21),
        trajectory_waypoint_t(2.084f, -0.516f / Mech::gear_radius_P, 31.604f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.341f, -0.323f / Mech::gear_radius_P, 227.077f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.026f, -0.437f / Mech::gear_radius_P, 395.165f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),

        trajectory_waypoint_t(3.551f, -0.213f / Mech::gear_radius_P, 219.692f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U31),
        trajectory_waypoint_t(2.041f, -0.535f / Mech::gear_radius_P, 39.780f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.729f, -0.157f / Mech::gear_radius_P, 209.846f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(1.980f, -0.465f / Mech::gear_radius_P, 405.626f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),

        trajectory_waypoint_t(4.090f, -0.109f / Mech::gear_radius_P, 183.736f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U31),
        trajectory_waypoint_t(2.011f, -0.561f / Mech::gear_radius_P, 44.440f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.388f, -0.109f / Mech::gear_radius_P, 169.495f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(1.945f, -0.495f / Mech::gear_radius_P, 409.495f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),

        trajectory_waypoint_t(4.791f, -0.167f / Mech::gear_radius_P, 149.099f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U31),
        trajectory_waypoint_t(1.965f, -0.575f / Mech::gear_radius_P, 54.286f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.969f, -0.225f / Mech::gear_radius_P, 139.429f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(1.908f, -0.520f / Mech::gear_radius_P, 415.209f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),

        trajectory_waypoint_t(5.165f, -0.334f / Mech::gear_radius_P, 123.956f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U32),  // ここから角柱にぶつかる
        trajectory_waypoint_t(1.939f, -0.601f / Mech::gear_radius_P, 58.242f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.270f, -0.417f / Mech::gear_radius_P, 120.967f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(1.874f, -0.552f / Mech::gear_radius_P, 418.813f, Hand::LiftAngle::SHOOT_LOW, Traj::PassThroughMode::INTERMEDIATE_1U3),

        // 4行目（ロボットに一番近い行）
        trajectory_waypoint_t(3.124f, -0.359f / Mech::gear_radius_P, 242.813f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_U31),
        trajectory_waypoint_t(1.903f, -0.432f / Mech::gear_radius_P, 178.549f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.201f, -0.265f / Mech::gear_radius_P, 237.187f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(1.953f, -0.394f / Mech::gear_radius_P, 178.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1U3),

        trajectory_waypoint_t(3.389f, -0.140f / Mech::gear_radius_P, 228.571f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.000f, -0.368f / Mech::gear_radius_P, 178.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(3.570f, -0.074f / Mech::gear_radius_P, 216.440f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.029f, -0.348f / Mech::gear_radius_P, 178.725f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.093f, -0.004f / Mech::gear_radius_P, 190.242f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),  //        5619
        trajectory_waypoint_t(2.077f, -0.328f / Mech::gear_radius_P, 178.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.467f, -0.012f / Mech::gear_radius_P, 165.538f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.136f, -0.308f / Mech::gear_radius_P, 178.374f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(4.939f, -0.079f / Mech::gear_radius_P, 138.374f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.180f, -0.289f / Mech::gear_radius_P, 178.549f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.138f, -0.151f / Mech::gear_radius_P, 126.506f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_1),
        trajectory_waypoint_t(2.250f, -0.267f / Mech::gear_radius_P, 178.637f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.327f, -0.278f / Mech::gear_radius_P, 116.220f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),  // ここから角柱にぶつかる
        trajectory_waypoint_t(2.382f, -0.239f / Mech::gear_radius_P, 178.549f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),

        trajectory_waypoint_t(5.410f, -0.372f / Mech::gear_radius_P, 113.407f, Hand::LiftAngle::FRONT_CATCH, Traj::PassThroughMode::INTERMEDIATE_2),
        trajectory_waypoint_t(2.495f, -0.222f / Mech::gear_radius_P, 178.286f, Hand::LiftAngle::SHOOT_UP, Traj::PassThroughMode::INTERMEDIATE_1),
    };
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
                    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_1ST);
                    sleep_ms(100);
                    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_1ST);
                    sleep_ms(100);
                    break;
                case 2:  // 2段階目
                    g_debug_manager->info("Disturbance deploying to 2nd stage.");
                    control_position_multiturn(&UART1, Dist::DXL_ID_LEFT, Dist::LEFT_DEPLOY_2ND);
                    sleep_ms(100);
                    control_position_multiturn(&UART1, Dist::DXL_ID_RIGHT, Dist::RIGHT_DEPLOY_2ND);
                    sleep_ms(100);
                    break;
            }
            prev_disturbance_level = g_disturbance_level;  // 状態を更新
        }

        // シューティングエリアのサーボを3秒に1回動かす
        static absolute_time_t last_shoot_servo_time = get_absolute_time();
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_shoot_servo_time, now) >= 3'000'000 &&
            absolute_time_diff_us(last_shoot_servo_time, now) < 6'000'000) {
            g_debug_manager->debug("Set shooting servo angle to correction angle.");
            shooting_servo.set_angle(ShootingConfig::CORRECTION_ANGLE);
        } else if (absolute_time_diff_us(last_shoot_servo_time, now) >= 6'000'000) {
            g_debug_manager->debug("Set shooting servo angle to idle angle.");
            shooting_servo.set_angle(ShootingConfig::IDLE_ANGLE);
            last_shoot_servo_time = now;
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
