#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.hpp"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"

constexpr int PROTOCOL_VERSION = 2;
constexpr short POLY = 0x8005;
extern unsigned short crc_table[256];  // CRC-tabel 除算演算の事前演算結果

constexpr uint BAUD_RATE = 4'000'000;
constexpr uint LED_PIN = 25;
constexpr float CURRENT_UNIT = 2.69 * 0.001;  //[A]
constexpr uint32_t PERIOD_US = 500;           // 1ms周期
constexpr float delta_t = static_cast<float>(PERIOD_US) / 1'000'000;
constexpr float PI = 3.14159265f;
constexpr float current_limit = 1.0;  //[A]
constexpr float TORQUE_TO_CURRENT_XM430W350 = 1 / 2.3179;
constexpr float INV_MAX_POS = 1 / 4095.0;

typedef struct {
    uart_inst_t* uart_number;  // picoのUART uart0 or uart1
    uint tx_pin;
    uint rx_pin;
    uint de_pin;  // 送信時1, 受信時0
} uart_config_t;

const uart_config_t UART0 = {
    .uart_number = uart0,
    .tx_pin = 16,
    .rx_pin = 17,
    .de_pin = 18};

const uart_config_t UART1 = {
    .uart_number = uart1,
    .tx_pin = 4,
    .rx_pin = 5,
    .de_pin = 6};

void init_crc();

unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);
// UART初期化・DEピンを受信設定
void configure_uart(const uart_config_t* config, uint baudrate);

// UARTバッファclear
void uart_clear_rx_buffer_safe(const uart_config_t* config);

void set_tx_mode(const uart_config_t* config);
void set_rx_mode(const uart_config_t* config);

void send_packet(const uart_config_t* config, const uint8_t* data, size_t length);

int receive_packet(const uart_config_t* config, uint8_t* rx_buf, size_t expected_len);
int write_operatingMode(const uart_config_t* config, uint8_t id, bool currentControlEnable);
int write_torqueEnable(const uart_config_t* config, uint8_t id, bool on);
int write_dxl_led(const uart_config_t* config, uint8_t id, uint8_t on);
int write_goalCurrent(const uart_config_t* config, uint8_t id, int16_t value);
int write_statusReturnLevel(const uart_config_t* config, uint8_t id, bool read_only);
int read_position(const uart_config_t* config, uint8_t id, uint32_t* position);
int control_position(const uart_config_t* config, uint8_t id, float angle);
float control_current_limit(float present_current);

#endif