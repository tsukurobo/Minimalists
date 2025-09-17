#include "dynamixel.hpp"

unsigned short crc_table[256];

void init_crc() {
    for (int i = 0; i < 256; i++) {
        unsigned short crc = i << 8;  // 上位8bitに配置
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {           // crc値の最上位bitが1かを判別
                crc = (crc << 1) ^ POLY;  // 現在のcrc値を左に1bitシフト、生成多項式とのXOR結果を得る
            } else {
                crc = crc << 1;  // 左に1bitシフト
            }
        }
        crc_table[i] = crc;  // 生成多項式で割った余り
    }
}

unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    for (j = 0; j < data_blk_size; j++) {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;  // CRCの上位8bitとdata_blk_ptr[i]とXORした結果の下位8bitを取り出す
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void configure_uart(const uart_config_t* config, uint baudrate) {
    uart_init(config->uart_number, baudrate);
    gpio_set_function(config->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config->rx_pin, GPIO_FUNC_UART);
    gpio_init(config->de_pin);
    gpio_set_dir(config->de_pin, GPIO_OUT);
    gpio_put(config->de_pin, 0);
}

// UARTバッファclear
void uart_clear_rx_buffer_safe(const uart_config_t* config) {
    int count = 0;
    const int max_clear = 64;

    while (uart_is_readable(config->uart_number) && count < max_clear) {
        int c = uart_getc(config->uart_number);
        count++;
        // printf("int value: %d\n", c);
        tight_loop_contents();
    }

    if (uart_get_hw(config->uart_number)->rsr) {
        uart_get_hw(config->uart_number)->rsr = 0;
    }
}

void set_tx_mode(const uart_config_t* config) {
    gpio_put(config->de_pin, 1);
}

void set_rx_mode(const uart_config_t* config) {
    gpio_put(config->de_pin, 0);
}

void send_packet(const uart_config_t* config, const uint8_t* data, size_t length) {
    uart_clear_rx_buffer_safe(config);
    set_tx_mode(config);
    sleep_us(10);  // DEピン安定化待ち
    uart_write_blocking(config->uart_number, data, length);
    uart_tx_wait_blocking(config->uart_number);
    sleep_us(1);
    set_rx_mode(config);
    sleep_us(1);
}

int receive_packet(const uart_config_t* config, uint8_t* rx_buf, size_t expected_len) {
    // 先頭を探す（0xFF 0xFF 0xFD 0x00）
    uint8_t header[4] = {0};
    int sync_found = 0;

    absolute_time_t timeout = make_timeout_time_ms(10);

    while (absolute_time_diff_us(get_absolute_time(), timeout) > 0) {
        // シフト読み込み
        if (uart_is_readable(config->uart_number)) {
            header[0] = header[1];
            header[1] = header[2];
            header[2] = header[3];
            header[3] = uart_getc(config->uart_number);
            // printf("%02X\n", header[3]);
            if (header[0] == 0xFF && header[1] == 0xFF && header[2] == 0xFD && header[3] == 0x00) {
                sync_found = 1;
                break;
            }
        } else {
            tight_loop_contents();
        }
    }

    if (!sync_found) {
        printf("Sync not found!\n");
        return -1;
    }

    rx_buf[0] = 0xFF;
    rx_buf[1] = 0xFF;
    rx_buf[2] = 0xFD;
    rx_buf[3] = 0x00;
    uart_read_blocking(config->uart_number, &rx_buf[4], expected_len - 4);
    return expected_len;
}

int write_operatingMode(const uart_config_t* config, uint8_t id, bool currentControlEnable) {
    uint8_t packet[13];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;    // ID
    packet[5] = 0x06;  // LEN_L = 6
    packet[6] = 0x00;  // LEN_H
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x0B;  // Address L
    packet[9] = 0x00;  // Address H
    // 0x00 電流制御, 0x01 速度制御, 0x03 位置制御, 0x04 拡張位置制御, 0x05 電流ベース位置制御, 0x16 PWM制御
    packet[10] = currentControlEnable ? 0x04 : 0x05;

    crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 13);

    return 0;
}

int write_torqueEnable(const uart_config_t* config, uint8_t id, bool on) {
    uint8_t packet[13];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;                 // ID
    packet[5] = 0x06;               // LEN_L = 6
    packet[6] = 0x00;               // LEN_H
    packet[7] = 0x03;               // Instruction: WRITE
    packet[8] = 0x40;               // Address L
    packet[9] = 0x00;               // Address H
    packet[10] = on ? 0x01 : 0x00;  // torque_enable

    crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 13);

    return 0;
}

int write_dxl_led(const uart_config_t* config, uint8_t id, uint8_t on) {
    uint8_t packet[13];
    uint16_t crc;
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;                 // ID
    packet[5] = 0x06;               // LEN_L = 6
    packet[6] = 0x00;               // LEN_H
    packet[7] = 0x03;               // Instruction: WRITE
    packet[8] = 0x41;               // Address L
    packet[9] = 0x00;               // Address H
    packet[10] = on ? 0x01 : 0x00;  // LED value
    crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;
    send_packet(config, packet, 13);
    return 0;
}

int write_goalCurrent(const uart_config_t* config, uint8_t id, int16_t value) {
    uint8_t packet[14];
    uint16_t crc;
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x07;                  // Length L
    packet[6] = 0x00;                  // Length H
    packet[7] = 0x03;                  // Instruction: WRITE
    packet[8] = 0x66;                  // Address L
    packet[9] = 0x00;                  // Address H
    packet[10] = value & 0xFF;         // Data length L
    packet[11] = (value >> 8) & 0xFF;  // Data length H
    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;
    send_packet(config, packet, 14);
    return 0;
}

int control_SyncWrite(const uart_config_t* config, uint8_t id1, uint8_t id2, float angle1, float angle2) {
    uint8_t packet[24];
    uint16_t crc;

    uint32_t position1 = static_cast<uint32_t>((angle1 / 360.0f) * 4095.0f);
    uint32_t position2 = static_cast<uint32_t>((angle2 / 360.0f) * 4095.0f);

    packet[0] = 0xFF;  // Header
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = 0xFE;  // ID
    packet[5] = 0x11;  // Length L = 17
    packet[6] = 0x00;  // Length H = 0
    packet[7] = 0x83;  // Instruction
    packet[8] = 0x74;  // Goal Position
    packet[9] = 0x00;
    packet[10] = 0x0;
    packet[11] = 0x00;
    packet[12] = id1;
    packet[13] = position1 & 0xFF;
    packet[14] = (position1 >> 8) & 0xFF;
    packet[15] = (position1 >> 16) & 0xFF;
    packet[16] = (position1 >> 24) & 0xFF;
    packet[17] = id2;
    packet[18] = position2 & 0xFF;
    packet[19] = (position2 >> 8) & 0xFF;
    packet[20] = (position2 >> 16) & 0xFF;
    packet[21] = (position2 >> 24) & 0xFF;

    crc = update_crc(0, packet, 22);
    packet[22] = crc & 0xFF;
    packet[23] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 24);
    return 0;
}

int return_DelayTime(const uart_config_t* config, uint8_t id, uint8_t time) {
    uint8_t packet[13];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x06;  // 長さ: パラメータ1バイト + 3
    packet[6] = 0x00;
    packet[7] = 0x03;   // WRITE
    packet[8] = 0x09;   // Address L (return delay time = 0x0009)
    packet[9] = 0x00;   // Address H
    packet[10] = time;  // 0~254

    crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 13);
    return 0;
}

int write_statusReturnLevel(const uart_config_t* config, uint8_t id, uint8_t level) {
    uint8_t packet[13];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;    // ID
    packet[5] = 0x06;  // LEN_L = 6
    packet[6] = 0x00;  // LEN_H
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x44;  // Address L
    packet[9] = 0x00;  // Address H
    packet[10] = level;

    crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 13);
    return 0;
}

int read_position(const uart_config_t* config, uint8_t id, uint32_t* position) {
    uint8_t packet[14];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x07;   // Length L
    packet[6] = 0x00;   // Length H
    packet[7] = 0x02;   // Instruction: READ
    packet[8] = 132;    // Address L
    packet[9] = 0x00;   // Address H
    packet[10] = 0x04;  // Data length L
    packet[11] = 0x00;  // Data length H

    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 14);

    uint8_t rx[15];
    int len = receive_packet(config, rx, 15);

    // for (int i = 0; i < len; i++) {
    //     printf("%02X ", rx[i]);
    // }

    if (len < 15) return -1;

    uint16_t received_crc = rx[13] | (rx[14] << 8);
    uint16_t calc_crc = update_crc(0, rx, 13);
    if (received_crc != calc_crc) {
        printf("CRC mismatch! received=0x%04X, calculated=0x%04X\n", received_crc, calc_crc);
        return -1;
    }
    // else{
    //     printf("CRC match! received=0x%04X, calculated=0x%04X\n", received_crc, calc_crc);
    // }

    *position = rx[9] | (rx[10] << 8) | (rx[11] << 16) | (rx[12] << 24);
    return 0;
}

int control_position(const uart_config_t* config, uint8_t id, float angle) {
    uint8_t packet[17];
    uint16_t crc;

    uint32_t position = static_cast<uint32_t>((angle / 360.0f) * 4095.0f);

    packet[0] = 0xFF;  // Header
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x09;
    packet[6] = 0x00;
    packet[7] = 0x03;
    packet[8] = 0x74;
    packet[9] = 0x00;
    packet[10] = position & 0xFF;
    packet[11] = (position >> 8) & 0xFF;
    packet[12] = (position >> 16) & 0xFF;
    packet[13] = (position >> 24) & 0xFF;

    crc = update_crc(0, packet, 14);
    packet[14] = crc & 0xFF;
    packet[15] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 16);
    return 0;
}

int control_position_multiturn(const uart_config_t* config, uint8_t id, int32_t position) {
    uint8_t packet[17];
    uint16_t crc;

    // Extended Position Control Mode: -1,048,575 ~ 1,048,575 (-256[rev] ~ 256[rev])
    // 1回転 = 4095 pulse, マルチターンでは複数回転を考慮
    // angleは度単位で入力されるため、pulse値に変換
    // float revolutions = angle / 360.0f;  // 回転数
    // int32_t position = static_cast<int32_t>(revolutions * 4095.0f);

    // マルチターン範囲チェック：-1,048,575 ~ 1,048,575
    // これは約-256.0回転から256.0回転の範囲に相当
    if (position > 1048575) {
        position = 1048575;
        printf("Warning: Position clamped to max value (256 rev)\n");
    } else if (position < -1048575) {
        position = -1048575;
        printf("Warning: Position clamped to min value (-256 rev)\n");
    }

    packet[0] = 0xFF;  // Header
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x09;  // Length: 9 bytes (Instruction + Address + Data)
    packet[6] = 0x00;
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x74;  // Goal Position address (116 = 0x74)
    packet[9] = 0x00;

    // 32ビット符号付き整数を4バイトで送信（Little Endian）
    packet[10] = position & 0xFF;
    packet[11] = (position >> 8) & 0xFF;
    packet[12] = (position >> 16) & 0xFF;
    packet[13] = (position >> 24) & 0xFF;

    crc = update_crc(0, packet, 14);
    packet[14] = crc & 0xFF;
    packet[15] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 16);
    return 0;
}

float control_current_limit(float present_current) {
    if (present_current > current_limit) {
        present_current = current_limit;
    } else if (present_current < -current_limit) {
        present_current = -current_limit;
    }
    return present_current;
}

int write_dxl_current_limit(const uart_config_t* config, uint8_t id, uint16_t current_limit_mA) {
    uint8_t packet[14];  // 正しいサイズ
    uint16_t crc;

    // --- パケット構築 ---
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;                               // ID
    packet[5] = 0x07;                             // LEN_L = 7 (Instruction + Addr_L + Addr_H + Data_L + Data_H)
    packet[6] = 0x00;                             // LEN_H
    packet[7] = 0x03;                             // Instruction: WRITE
    packet[8] = 0x26;                             // Address L: Current Limit L (38)
    packet[9] = 0x00;                             // Address H
    packet[10] = current_limit_mA & 0xFF;         // Data L
    packet[11] = (current_limit_mA >> 8) & 0xFF;  // Data H

    // --- CRC計算 ---
    crc = update_crc(0, packet, 12);  // CRCはHEADER〜データまで
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    // --- 送信 ---
    send_packet(config, packet, 14);  // 正しいサイズで送信

    return 0;
}

int read_position_multiturn(const uart_config_t* config, uint8_t id, int32_t* position) {
    uint8_t packet[14];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x07;   // Length L
    packet[6] = 0x00;   // Length H
    packet[7] = 0x02;   // Instruction: READ
    packet[8] = 0x84;   // Address L (Present Position = 132 = 0x84)
    packet[9] = 0x00;   // Address H
    packet[10] = 0x04;  // Data length L (4 bytes for 32-bit position)
    packet[11] = 0x00;  // Data length H

    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 14);

    uint8_t rx[15];
    int len = receive_packet(config, rx, 15);

    if (len < 15) return -1;

    uint16_t received_crc = rx[13] | (rx[14] << 8);
    uint16_t calc_crc = update_crc(0, rx, 13);
    if (received_crc != calc_crc) {
        printf("CRC mismatch! received=0x%04X, calculated=0x%04X\n", received_crc, calc_crc);
        return -1;
    }

    // 32ビット符号付き整数として読み取り（Little Endian）
    uint32_t raw_position = rx[9] | (rx[10] << 8) | (rx[11] << 16) | (rx[12] << 24);
    *position = static_cast<int32_t>(raw_position);

    return 0;
}

int write_position_Pgain(const uart_config_t* config, uint8_t id, uint16_t Kp) {
    uint8_t packet[14];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;    // ID
    packet[5] = 0x07;  // LEN_L = 7
    packet[6] = 0x00;  // LEN_H
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x54;  // Address L Pgain = 84
    packet[9] = 0x00;  // Address H
    packet[10] = Kp & 0xFF;
    packet[11] = (Kp >> 8) & 0xFF;

    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 14);
    return 0;
}
int write_position_Dgain(const uart_config_t* config, uint8_t id, uint16_t Kd) {
    uint8_t packet[14];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;    // ID
    packet[5] = 0x07;  // LEN_L = 7
    packet[6] = 0x00;  // LEN_H
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x50;  // Address L Dgain = 80
    packet[9] = 0x00;  // Address H
    packet[10] = Kd & 0xFF;
    packet[11] = (Kd >> 8) & 0xFF;

    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 14);
    return 0;
}
int write_position_Igain(const uart_config_t* config, uint8_t id, uint16_t Ki) {
    uint8_t packet[14];
    uint16_t crc;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;    // ID
    packet[5] = 0x07;  // LEN_L = 7
    packet[6] = 0x00;  // LEN_H
    packet[7] = 0x03;  // Instruction: WRITE
    packet[8] = 0x52;  // Address I Pgain = 82
    packet[9] = 0x00;  // Address H
    packet[10] = Ki & 0xFF;
    packet[11] = (Ki >> 8) & 0xFF;

    crc = update_crc(0, packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    send_packet(config, packet, 14);
    return 0;
}