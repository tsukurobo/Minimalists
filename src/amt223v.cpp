#include "amt223v.hpp"

#include <stdio.h>

#include <cmath>
#include <cstdint>

#include "hardware/gpio.h"

namespace {
constexpr float PI_F = 3.14159265358979323846f;
}

// AMT223D-V マルチターンコマンド定義
const uint8_t AMT223V::CMD_READ_MULTITURN[4] = {0x00, 0xA0, 0x00, 0x00};

// ゼロ位置セットコマンド定義（単回転エンコーダのみ）
const uint8_t AMT223V::CMD_SET_ZERO[2] = {0x00, 0x70};

AMT223V::AMT223V(spi_inst_t* spi_instance, float velocity_cutoff_freq, int chip_select_pin, bool multiturn_support)
    : spi_port(spi_instance),
      cs_pin(chip_select_pin),
      raw_angle(0),
      angle_rad(0.0f),
      angle_deg(0.0f),
      is_multiturn(multiturn_support),
      turn_count(0),
      initial_turn_count(0),
      continuous_angle_rad(0.0f),
      continuous_angle_deg(0.0f),
      previous_angle_rad(0.0f),
      angular_velocity_rad(0.0f),
      angular_velocity_deg(0.0f),
      previous_time_us(0),
      velocity_initialized(false),
      velocity_filter(velocity_cutoff_freq) {
}

bool AMT223V::init() {
    // CSピンの初期化（既にmain.cppで初期化済みの場合はスキップ可能）
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);  // 初期状態はHIGH（非選択）

    // 初期読み取りテスト
    if (!read_angle()) {
        printf("AMT223V initialization failed - initial read failed\n");
        return false;
    }

    // マルチターンエンコーダの場合は初期回転回数を保存
    if (is_multiturn) {
        initial_turn_count = turn_count;
        printf("Initial turn count saved: %d\n", initial_turn_count);
    }

    printf("AMT223V initialized successfully on CS pin %d (multiturn: %s)\n",
           cs_pin, is_multiturn ? "yes" : "no");
    printf("Initial angle: %d counts, %.2f deg, %.4f rad\n",
           raw_angle, angle_deg, angle_rad);

    // // 単回転エンコーダの場合はゼロ位置セットを実行
    // if (!is_multiturn) {
    //     printf("Setting zero position for single-turn encoder on CS pin %d...\n", cs_pin);

    //     // エンコーダが安定するまで少し待つ
    //     sleep_ms(100);

    //     if (set_zero_position()) {
    //         printf("Zero position set successfully!\n");

    //         // ゼロ位置セット後に再度読み取り
    //         sleep_ms(50);  // リセット後の安定化時間
    //         if (read_angle()) {
    //             printf("Post-zero angle: %d counts, %.2f deg, %.4f rad\n",
    //                    raw_angle, angle_deg, angle_rad);
    //         }
    //     } else {
    //         printf("Warning: Failed to set zero position\n");
    //     }
    // }

    return true;
}

bool AMT223V::read_angle() {
    if (is_multiturn) {
        // マルチターン対応の場合は4バイト通信
        uint8_t tx_data[4] = {0x00, 0xA0, 0x00, 0x00};
        uint8_t rx_data[4] = {0};

        // SPI通信実行
        select();
        spi_transfer(tx_data, rx_data, 4);
        deselect();

        // 最初の2バイトから角度を抽出（14ビット）
        uint16_t received_angle = (static_cast<uint16_t>(rx_data[0]) << 8) | static_cast<uint16_t>(rx_data[1]);

        // パリティチェック実行
        if (!verify_parity(received_angle)) {
            printf("Warning: Parity check failed for angle data\n");
            return false;
        }

        raw_angle = received_angle & ANGLE_MASK;

        // 次の2バイトから回転回数を抽出（14ビット符号付き）
        uint16_t received_turn = (static_cast<uint16_t>(rx_data[2]) << 8) | static_cast<uint16_t>(rx_data[3]);
        received_turn = TURN_MASK & received_turn;  // 14ビットでマスク

        // // 受信したデータとマスク後のデータを表示
        // printf("Received hex: 0x%02X 0x%02X 0x%02X 0x%02X\n",
        //        rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
        // printf("Received hex (masked): 0x%04X\n", received_turn);

        // 14ビット符号付き数値に変換（生の値）
        int16_t raw_turn_count;
        if (received_turn & 0x2000) {                            // MSBが1の場合は負数
            raw_turn_count = (int16_t)(received_turn | 0xC000);  // 符号拡張
        } else {
            raw_turn_count = (int16_t)received_turn;
        }

        // printf("raw_turn_count: %d   ", raw_turn_count);
        // printf("initial_turn_count: %d\n", initial_turn_count);

        // 初期化時の回転回数からの差分を計算
        turn_count = raw_turn_count - initial_turn_count;

        // 角度変換
        angle_rad = (float)raw_angle * 2.0f * PI_F / COUNTS_PER_REV;
        angle_deg = (float)raw_angle * 360.0f / COUNTS_PER_REV;

        // 連続角度計算
        continuous_angle_rad = (float)turn_count * 2.0f * PI_F + angle_rad;
        continuous_angle_deg = (float)turn_count * 360.0f + angle_deg;

    } else {
        // 単回転モード
        uint8_t tx_data[2] = {CMD_READ_ANGLE, 0x00};  // コマンド + ダミーバイト
        uint8_t rx_data[2] = {0};

        // SPI通信実行
        select();
        spi_transfer(tx_data, rx_data, 2);
        deselect();

        // 受信データから角度を抽出（16ビット応答）
        uint16_t received_data = (rx_data[0] << 8) | rx_data[1];

        // パリティチェック実行
        if (!verify_parity(received_data)) {
            printf("Warning: Parity check failed for single-turn data\n");
            return false;
        }

        raw_angle = received_data & ANGLE_MASK;

        // 角度変換
        angle_rad = (float)raw_angle * 2.0f * PI_F / COUNTS_PER_REV;
        angle_deg = (float)raw_angle * 360.0f / COUNTS_PER_REV;

        // 単回転では連続角度は通常角度と同じ
        continuous_angle_rad = angle_rad;
        continuous_angle_deg = angle_deg;
        turn_count = 0;
    }

    // データの妥当性チェック（14ビット範囲内かどうか）
    if (raw_angle >= COUNTS_PER_REV) {
        printf("Warning: Invalid angle data received: %d\n", raw_angle);
        return false;
    }

    // 角速度計算
    uint64_t current_time_us = time_us_64();
    float current_angle = is_multiturn ? continuous_angle_rad : angle_rad;
    calculate_angular_velocity(current_angle, current_time_us);

    return true;
}

bool AMT223V::set_zero_position() {
    // マルチターンエンコーダではゼロ位置セットは使用できません
    if (is_multiturn) {
        printf("Error: Set zero position is not supported for multiturn encoders\n");
        return false;
    }

    // ゼロ位置セットコマンドを送信
    uint8_t tx_data[2] = {CMD_SET_ZERO[0], CMD_SET_ZERO[1]};  // {0x00, 0x70}
    uint8_t rx_data[2] = {0};

    printf("Sending set zero command: 0x%02X 0x%02X\n", tx_data[0], tx_data[1]);

    // SPI通信実行
    select();
    spi_transfer(tx_data, rx_data, 2);
    deselect();

    // 受信したデータを確認（現在位置が返される）
    uint16_t received_data = (rx_data[0] << 8) | rx_data[1];
    uint16_t position_before_reset = received_data & ANGLE_MASK;

    printf("Position before reset: %d counts (%.2f deg)\n",
           position_before_reset,
           (float)position_before_reset * 360.0f / COUNTS_PER_REV);

    // エンコーダがリセット処理を完了するまで待機
    // データシートによると、リセット処理には時間がかかる場合があります
    sleep_ms(100);

    printf("Zero position set command completed\n");
    return true;
}

bool AMT223V::reset_turn_count() {
    if (!is_multiturn) {
        printf("Error: Reset turn count is only supported for multiturn encoders\n");
        return false;
    }

    // 現在の生の回転回数を読み取り
    if (!read_angle()) {
        printf("Error: Failed to read current angle for turn count reset\n");
        return false;
    }

    // 現在の生の回転回数を新しい基準点として設定
    // この時点でturn_countは差分値なので、initial_turn_count + turn_countが生の値
    initial_turn_count = initial_turn_count + turn_count;
    turn_count = 0;  // 差分をリセット

    printf("Turn count reset completed. New reference: %d\n", initial_turn_count);
    return true;
}

void AMT223V::select() {
    gpio_put(cs_pin, 0);  // CSをLOWにして選択
    sleep_us(4);          // セットアップ時間 規定値 2.5 us
}

void AMT223V::deselect() {
    sleep_us(4);          // time before CS can be released 規定値 3 us　
    gpio_put(cs_pin, 1);  // CSをHIGHにして非選択
    sleep_us(1);          // time after CS is released
}

void AMT223V::spi_transfer(const uint8_t* tx_data, uint8_t* rx_buffer, size_t length) {
    // SPI通信実行
    for (size_t i = 0; i < length; i++) {
        if (i == 2) {
            sleep_us(4);
        }
        spi_write_read_blocking(spi_port, &tx_data[i], &rx_buffer[i], 1);
        sleep_us(4);  // time between bytes 規定値 2.5 us
    }
}

bool AMT223V::verify_parity(uint16_t response) const {
    // チェックビットを抽出
    uint8_t k1 = (response >> 15) & 0x01;  // MSB (ビット15)
    uint8_t k0 = (response >> 14) & 0x01;  // ビット14

    // 14ビット位置データを抽出
    uint16_t position = response & 0x3FFF;

    // 奇数ビットの XOR 計算: H5^H3^H1^L7^L5^L3^L1
    uint8_t odd_parity = 0;
    odd_parity ^= (position >> 13) & 0x01;  // H5 (bit 13)
    odd_parity ^= (position >> 11) & 0x01;  // H3 (bit 11)
    odd_parity ^= (position >> 9) & 0x01;   // H1 (bit 9)
    odd_parity ^= (position >> 7) & 0x01;   // L7 (bit 7)
    odd_parity ^= (position >> 5) & 0x01;   // L5 (bit 5)
    odd_parity ^= (position >> 3) & 0x01;   // L3 (bit 3)
    odd_parity ^= (position >> 1) & 0x01;   // L1 (bit 1)

    // 偶数ビットの XOR 計算: H4^H2^H0^L6^L4^L2^L0
    uint8_t even_parity = 0;
    even_parity ^= (position >> 12) & 0x01;  // H4 (bit 12)
    even_parity ^= (position >> 10) & 0x01;  // H2 (bit 10)
    even_parity ^= (position >> 8) & 0x01;   // H0 (bit 8)
    even_parity ^= (position >> 6) & 0x01;   // L6 (bit 6)
    even_parity ^= (position >> 4) & 0x01;   // L4 (bit 4)
    even_parity ^= (position >> 2) & 0x01;   // L2 (bit 2)
    even_parity ^= (position >> 0) & 0x01;   // L0 (bit 0)

    // チェックビット計算 (奇数パリティ)
    uint8_t k1_calc = !odd_parity;   // K1 = !(奇数ビットXOR)
    uint8_t k0_calc = !even_parity;  // K0 = !(偶数ビットXOR)

    // パリティチェック結果
    bool parity_ok = (k1 == k1_calc) && (k0 == k0_calc);

    // デバッグ情報（初期化時のみ表示）
    static int parity_debug_count = 0;
    if (parity_debug_count < 3 || !parity_ok) {
        printf("Parity check: 0x%04X, pos=0x%04X, K1=%d(calc:%d), K0=%d(calc:%d) -> %s\n",
               response, position, k1, k1_calc, k0, k0_calc,
               parity_ok ? "OK" : "ERROR");
        if (parity_ok) parity_debug_count++;
    }

    return parity_ok;
}

void AMT223V::calculate_angular_velocity(float current_angle_rad, uint64_t current_time_us) {
    // 角度差を計算
    float delta_angle_rad = current_angle_rad - previous_angle_rad;
    // マルチターンでない場合は360度の境界を考慮
    if (!is_multiturn) {
        // 角度差が180度を超える場合は逆方向の最短経路を計算
        if (delta_angle_rad > PI_F) {
            delta_angle_rad -= 2.0f * PI_F;
        } else if (delta_angle_rad < -PI_F) {
            delta_angle_rad += 2.0f * PI_F;
        }
    }

    velocity_filter.update(current_angle_rad);                   // フィルタを更新
    angular_velocity_rad = velocity_filter.get_dot_value();      // 疑似微分済み角速度を取得
    angular_velocity_deg = angular_velocity_rad * 180.0f / PI_F;  // ラジアンから度に変換

    // 異常値フィルタリング（物理的に不可能な角速度をチェック）
    const float MAX_ANGULAR_VELOCITY = 50.0f;  // 最大角速度 [rad/s] (約477rpm)
    if (fabs(angular_velocity_rad) > MAX_ANGULAR_VELOCITY) {
        // 異常値の場合は前回値を保持
        previous_angle_rad = current_angle_rad;
        previous_time_us = current_time_us;
        return;
    }

    previous_angle_rad = current_angle_rad;  // 異常検出・境界判定用に保存
}

// ===== AMT223V_Manager クラス実装 =====

AMT223V_Manager::AMT223V_Manager(spi_inst_t* spi_instance, int miso, int sck, int mosi)
    : spi_port(spi_instance), pin_miso(miso), pin_sck(sck), pin_mosi(mosi), num_encoders(0) {
    // エンコーダポインタを初期化
    for (auto& encoder : encoders) {
        encoder = nullptr;
    }
    // CSピンを初期化
    cs_pins.fill(-1);
}

int AMT223V_Manager::add_encoder(int cs_pin, float velocity_cutoff_freq, bool multiturn_support) {
    if (num_encoders >= MAX_ENCODERS) {
        printf("Error: Maximum %d encoders supported, current count: %d\n", MAX_ENCODERS, num_encoders);
        return -1;
    }

    cs_pins[num_encoders] = cs_pin;
    encoders[num_encoders] = new AMT223V(spi_port, velocity_cutoff_freq, cs_pin, multiturn_support);
    num_encoders++;

    printf("Added encoder %d with CS pin %d (multiturn: %s)\n",
           num_encoders - 1, cs_pin, multiturn_support ? "yes" : "no");
    printf("Current encoder count: %d/%d\n", num_encoders, MAX_ENCODERS);
    return num_encoders - 1;
}

bool AMT223V_Manager::init_all_encoders() {
    printf("Initializing %d/%d AMT223-V encoders...\n", num_encoders, MAX_ENCODERS);

    for (int i = 0; i < num_encoders; i++) {
        if (!encoders[i]->init()) {
            printf("Failed to initialize Encoder %d!\n", i);
            return false;
        }
    }

    printf("All %d encoders initialized successfully! (Max capacity: %d)\n",
           num_encoders, MAX_ENCODERS);
    return true;
}

bool AMT223V_Manager::read_encoder(int encoder_index) {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return false;
    }

    return encoders[encoder_index]->read_angle();
}

int AMT223V_Manager::read_all_encoders() {
    int success_count = 0;

    for (int i = 0; i < num_encoders; i++) {
        if (read_encoder(i)) {
            success_count++;
        }
    }

    return success_count;
}

float AMT223V_Manager::get_encoder_angle_rad(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return -1.0f;
    }

    return encoders[encoder_index]->get_angle_rad();
}

float AMT223V_Manager::get_encoder_angle_deg(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return -1.0f;
    }

    return encoders[encoder_index]->get_angle_deg();
}

int16_t AMT223V_Manager::get_encoder_turn_count(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return 0;
    }

    return encoders[encoder_index]->get_turn_count();
}

float AMT223V_Manager::get_encoder_continuous_angle_rad(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return 0.0f;
    }

    return encoders[encoder_index]->get_continuous_angle_rad();
}

float AMT223V_Manager::get_encoder_continuous_angle_deg(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return 0.0f;
    }

    return encoders[encoder_index]->get_continuous_angle_deg();
}

bool AMT223V_Manager::set_encoder_zero_position(int encoder_index) {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return false;
    }

    printf("Setting zero position for encoder %d...\n", encoder_index);
    return encoders[encoder_index]->set_zero_position();
}

float AMT223V_Manager::get_encoder_angular_velocity_rad(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return 0.0f;
    }

    return encoders[encoder_index]->get_angular_velocity_rad();
}

float AMT223V_Manager::get_encoder_angular_velocity_deg(int encoder_index) const {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return 0.0f;
    }

    return encoders[encoder_index]->get_angular_velocity_deg();
}

bool AMT223V_Manager::reset_encoder_turn_count(int encoder_index) {
    if (!is_valid_encoder_index(encoder_index)) {
        printf("Error: Invalid encoder index %d (valid range: 0-%d)\n", encoder_index, num_encoders - 1);
        return false;
    }

    printf("Resetting turn count for encoder %d...\n", encoder_index);
    return encoders[encoder_index]->reset_turn_count();
}
