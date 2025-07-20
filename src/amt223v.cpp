#include "amt223v.hpp"

#include <stdio.h>

#include <cmath>
#include <cstdint>

#include "hardware/gpio.h"

// M_PIが定義されていない場合の定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// AMT223D-V マルチターンコマンド定義
const uint8_t AMT223V::CMD_READ_MULTITURN[4] = {0x00, 0xA0, 0x00, 0x00};

AMT223V::AMT223V(spi_inst_t* spi_instance, int chip_select_pin, bool multiturn_support)
    : spi_port(spi_instance), cs_pin(chip_select_pin), raw_angle(0), angle_rad(0.0), angle_deg(0.0), is_multiturn(multiturn_support), turn_count(0), continuous_angle_rad(0.0), continuous_angle_deg(0.0) {
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

    printf("AMT223V initialized successfully on CS pin %d\n", cs_pin);
    printf("Initial angle: %d counts, %.2f deg, %.4f rad\n",
           raw_angle, angle_deg, angle_rad);

    return true;
}

bool AMT223V::read_angle() {
    if (is_multiturn) {
        // マルチターン対応の場合は4バイト通信
        uint8_t tx_data[4] = {0x00, 0xA0, 0x00, 0x00};
        uint8_t rx_data[4] = {0};

        // SPI通信実行
        select();
        for (int i = 0; i < 4; i++) {
            // 1バイトずつ送信して受信
            spi_transfer(&tx_data[i], &rx_data[i], 1);
            sleep_us(3);  // SPI通信の安定化のために少し待つ
        }
        deselect();

        // 最初の2バイトから角度を抽出（14ビット）
        uint16_t received_angle = (static_cast<uint16_t>(rx_data[0]) << 8) | static_cast<uint16_t>(rx_data[1]);
        raw_angle = received_angle & ANGLE_MASK;

        sleep_us(3);

        // 次の2バイトから回転回数を抽出（14ビット符号付き）
        uint16_t received_turn = (static_cast<uint16_t>(rx_data[2]) << 8) | static_cast<uint16_t>(rx_data[3]);
        received_turn = 0x0FFF & received_turn;  // 14ビットでマスクとるとなんか動かなかったから12ビットマスクをとってる

        // 14ビット符号付き数値に変換
        if (received_turn & 0x2000) {                        // MSBが1の場合は負数
            turn_count = (int32_t)(received_turn | 0xC000);  // 符号拡張
        } else {
            turn_count = (int32_t)received_turn;
        }

        // 角度変換
        angle_rad = (double)raw_angle * 2.0 * M_PI / COUNTS_PER_REV;
        angle_deg = (double)raw_angle * 360.0 / COUNTS_PER_REV;

        // 連続角度計算
        continuous_angle_rad = (double)turn_count * 2.0 * M_PI + angle_rad;
        continuous_angle_deg = (double)turn_count * 360.0 + angle_deg;

    } else {
        // 単回転モード
        uint8_t tx_data[2] = {CMD_READ_ANGLE, 0x00};  // コマンド + ダミーバイト
        uint8_t rx_data[2] = {0};

        // SPI通信実行
        select();
        spi_transfer(tx_data, rx_data, 2);
        deselect();

        // 受信データから角度を抽出（14ビット）
        uint16_t received_data = (rx_data[0] << 8) | rx_data[1];
        raw_angle = received_data & ANGLE_MASK;

        // 角度変換
        angle_rad = (double)raw_angle * 2.0 * M_PI / COUNTS_PER_REV;
        angle_deg = (double)raw_angle * 360.0 / COUNTS_PER_REV;

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

    return true;
}

void AMT223V::select() {
    gpio_put(cs_pin, 0);  // CSをLOWにして選択
    sleep_us(1);          // セットアップ時間
}

void AMT223V::deselect() {
    sleep_us(1);          // ホールド時間
    gpio_put(cs_pin, 1);  // CSをHIGHにして非選択
}

void AMT223V::spi_transfer(const uint8_t* tx_data, uint8_t* rx_buffer, size_t length) {
    // SPI通信実行
    spi_write_read_blocking(spi_port, tx_data, rx_buffer, length);
}

// ===== AMT223V_Manager クラス実装 =====

AMT223V_Manager::AMT223V_Manager(spi_inst_t* spi_instance, uint32_t baud, int miso, int sck, int mosi)
    : spi_port(spi_instance), baudrate(baud), pin_miso(miso), pin_sck(sck), pin_mosi(mosi), num_encoders(0) {
    // エンコーダポインタを初期化
    for (auto& encoder : encoders) {
        encoder = nullptr;
    }
    // CSピンを初期化
    cs_pins.fill(-1);
}

int AMT223V_Manager::add_encoder(int cs_pin, bool multiturn_support) {
    if (num_encoders >= 4) {
        printf("Error: Maximum 4 encoders supported\n");
        return -1;
    }

    cs_pins[num_encoders] = cs_pin;
    encoders[num_encoders] = new AMT223V(spi_port, cs_pin, multiturn_support);
    num_encoders++;

    printf("Added encoder %d with CS pin %d (multiturn: %s)\n",
           num_encoders - 1, cs_pin, multiturn_support ? "yes" : "no");
    return num_encoders - 1;
}

bool AMT223V_Manager::init_spi() {
    // SPI初期化
    spi_init(spi_port, baudrate);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);

    // CSピンの初期化
    for (int i = 0; i < num_encoders; i++) {
        gpio_init(cs_pins[i]);
        gpio_set_dir(cs_pins[i], GPIO_OUT);
        gpio_put(cs_pins[i], 1);  // 初期状態はHIGH（非選択）
        printf("Encoder SPI CS%d pin %d initialized\n", i, cs_pins[i]);
    }

    printf("Encoder SPI initialized: %d encoders on SPI%d at %d Hz\n",
           num_encoders, spi_port == spi0 ? 0 : 1, baudrate);
    return true;
}

bool AMT223V_Manager::init_all_encoders() {
    printf("Initializing %d AMT223-V encoders...\n", num_encoders);

    for (int i = 0; i < num_encoders; i++) {
        if (!encoders[i]->init()) {
            printf("Failed to initialize Encoder %d!\n", i);
            return false;
        }
    }

    printf("All %d encoders initialized successfully!\n", num_encoders);
    return true;
}

bool AMT223V_Manager::read_encoder(int encoder_index) {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
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

double AMT223V_Manager::get_encoder_angle_rad(int encoder_index) const {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
        return -1.0;
    }

    return encoders[encoder_index]->get_angle_rad();
}

double AMT223V_Manager::get_encoder_angle_deg(int encoder_index) const {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
        return -1.0;
    }

    return encoders[encoder_index]->get_angle_deg();
}

int16_t AMT223V_Manager::get_encoder_turn_count(int encoder_index) const {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
        return 0;
    }

    return encoders[encoder_index]->get_turn_count();
}

double AMT223V_Manager::get_encoder_continuous_angle_rad(int encoder_index) const {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
        return 0.0;
    }

    return encoders[encoder_index]->get_continuous_angle_rad();
}

double AMT223V_Manager::get_encoder_continuous_angle_deg(int encoder_index) const {
    if (encoder_index < 0 || encoder_index >= num_encoders || !encoders[encoder_index]) {
        return 0.0;
    }

    return encoders[encoder_index]->get_continuous_angle_deg();
}
