#include "config.hpp"
#include "dynamixel.hpp"
#include "trajectory.hpp"
#include "trajectory_sequence_manager.hpp"

namespace Mc = MicrocontrollerConfig;
namespace Mech = MechanismConfig;
namespace Traj = TrajectoryConfig;
namespace Ctrl = ControlConfig;
namespace Hand = HandConfig;
namespace Dist = DisturbanceConfig;

bool irq_status = false;  // 割り込みステータスフラグ

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

// AMT223-V エンコーダマネージャを作成
AMT223V_Manager encoder_manager(SPI1_CONFIG.spi_port, SPI1_CONFIG.pin_miso, SPI1_CONFIG.pin_sck, SPI1_CONFIG.pin_mosi);

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

// 　ハンドの動作実行
void hand_tick(hand_state_t* hand_state, bool* has_work, absolute_time_t* state_start_time, float hand_angle, int32_t lift_angle) {
    uint32_t elapsed_ms = absolute_time_diff_us(*state_start_time, get_absolute_time()) / 1000;
    switch (*hand_state) {
        case HAND_IDLE:
            g_debug_manager->debug("hand requested\n");
            if (!*has_work) {
                *hand_state = HAND_LOWERING;
                *state_start_time = get_absolute_time();
                gpio_put(Hand::PUMP_PIN, 1);
                g_debug_manager->debug("Hand lowering...");
                control_position_multiturn(&UART1, Hand::DXL_ID2, Hand::LiftAngle::CATCH);
            } else {
                *hand_state = HAND_RELEASE;
                *state_start_time = get_absolute_time();
                gpio_put(Hand::SOLENOID_PIN, 1);
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
                control_position_multiturn(&UART1, Hand::DXL_ID2, lift_angle);
                g_debug_manager->debug("Hand raising...\n");
            }
            break;

        case HAND_RAISING:
            if (elapsed_ms >= 200) {
                *has_work = true;
                control_position(&UART1, Hand::DXL_ID1, hand_angle);
                g_debug_manager->debug("Hand raised, work done.\n");
                *hand_state = HAND_WAITING;  // HAND_IDLE前に1秒待機
                *state_start_time = get_absolute_time();
            }
            break;

        case HAND_RELEASE:
            if (elapsed_ms >= 150) {
                *has_work = false;
                gpio_put(Hand::SOLENOID_PIN, 0);
                control_position(&UART1, Hand::DXL_ID1, hand_angle);
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

// 妨害展開・最速アームの割り込みハンドラ
void handle_disturbance_trigger() {
    static uint32_t last_button_press_time = 0;
    uint32_t now = time_us_32();
    // 200ms以内の連続した割り込みはチャタリングとみなし無視する
    if (now - last_button_press_time < 200 * 1000) {
        return;
    }
    last_button_press_time = now;

    irq_status = true;  // メインループで処理するためのフラグをセット
}

// ボタンを押したときのコールバック関数
void button_cb(uint gpio, uint32_t events) {
    handle_disturbance_trigger();
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
    g_debug_manager = new DebugManager(DebugLevel::OFF, 0.1f);

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
    // init_hand_dist();

    // ボタンのGPIOと割り込み設定
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // 内部プルアップを有効化
    // 立ち下がりエッジ（ボタンが押されたとき）で割り込み発生
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, button_cb);

    sleep_ms(2000);  // シリアル接続待ち
    return true;
}

int main(void) {
    // 初期化処理をまとめて呼び出す
    if (!initialize_system()) {
        // 初期化失敗時の処理（必要ならエラー表示や停止）
        return -1;
    }

    init_crc();
    configure_uart(&UART1, BAUD_RATE);

    while (true) {
        if (irq_status) {
            irq_status = false;
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
            }

            if (enc2_ok) {
                motor_position_P = encoder_manager.get_encoder_continuous_angle_rad(1) * Mech::ENCODER_P_DIRECTION;
            }

            // 手先の角度を取得
            uint32_t hand_angle = 0;
            read_position(&UART1, Hand::DXL_ID1, &hand_angle);  // ハンド角度
            float hand_angle_deg = static_cast<float>(hand_angle) / 4095.0f * 360.0f;

            int32_t lift_angle = 0;
            read_position_multiturn(&UART1, Hand::DXL_ID2, &lift_angle);  // 握り上げ角度

            // 現在の軌道点を表示
            printf(
                "trajectory_waypoint_t(%.3f, %.3f / Mech::gear_radius_P, %.3f, %08d),\n",
                motor_position_R, motor_position_P * Mech::gear_radius_P, hand_angle_deg, lift_angle);
            // Core1の初期化完了を待つ
            sleep_ms(1000);
        }
        tight_loop_contents();
    }
    return 0;
}