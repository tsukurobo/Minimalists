#include "mcp25625.hpp"

#include <stdio.h>

// コンストラクタ: SPIとGPIOピンを初期化
mcp25625_t::mcp25625_t(spi_inst_t* spi, uint8_t cs_pin, uint8_t rst_pin)
    : _spi(spi), _cs_pin(cs_pin), _rst_pin(rst_pin) {
    // リセットピンを初期化
    gpio_init(_rst_pin);
    gpio_set_dir(_rst_pin, GPIO_OUT);
    gpio_put(_rst_pin, 1);

    // CSピンを初期化
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);
}

// 初期化: リセット、ビットタイミング設定、通常モードへの移行
bool mcp25625_t::init(CAN_SPEED speed) {
    // _reset();
    sleep_ms(10);

    if (!_set_mode(MODE_CONFIG)) {  // コンフィグレーションモードに設定 [cite: 295]
        printf("[ERROR] Failed to set configuration mode.\n");
        return false;
    }

    if (!_set_bit_timing(speed)) {  // ビットタイミングを設定 [cite: 296]
        printf("[ERROR] Failed to set bit timing.\n");
        return false;
    }

    // 受信バッファ0の設定: 全てのメッセージをフィルタなしで受信 [cite: 608]
    _modify_register(MCP_RXB0CTRL, 0x60, 0x60);
    _modify_register(MCP_RXB1CTRL, 0x60, 0x60);  // RXB1: 全受信 ←追加

    if (!_set_mode(MODE_NORMAL)) {  // 通常動作モードに設定 [cite: 300]
        printf("[ERROR] Failed to set normal mode.\n");
        return false;
    }

    return true;
}

// CANメッセージを送信バッファにロードして送信要求
bool mcp25625_t::send_can_message(const struct can_frame_t* frame) {
    constexpr int max_wait = 250;  // 最大リトライ数（タイムアウト防止）
    for (int i = 0; i < max_wait; ++i) {
        uint8_t status = _read_register(MCP_TXB0CTRL);
        if ((status & 0x08) == 0) {
            break;  // 空きが確認できたら送信準備へ
        }
        sleep_us(1);  // 必要に応じて調整（100usなど）
        if (i == max_wait - 1) {
            return false;  // タイムアウト
        }
    }

    // 標準IDをレジスタに書き込む
    uint8_t tx_buf[13];
    tx_buf[0] = (uint8_t)(frame->can_id >> 3);  // TXB0SIDH
    tx_buf[1] = (uint8_t)(frame->can_id << 5);  // TXB0SIDL
    tx_buf[2] = 0;                              // TXB0EID8
    tx_buf[3] = 0;                              // TXB0EID0
    tx_buf[4] = frame->can_dlc;                 // TXB0DLC [cite: 485]

    // データをバッファにコピー
    for (int i = 0; i < frame->can_dlc; ++i) {
        tx_buf[5 + i] = frame->data[i];
    }

    // SPI経由で送信バッファに書き込む
    _write_registers(MCP_TXB0CTRL + 1, tx_buf, 5 + frame->can_dlc);
    sleep_us(1);

    // 送信要求 (Request-to-Send) [cite: 496]
    _request_to_send(MCP_RTS_TXB0);
    sleep_us(1);  // 送信要求後の安定化時間

    return true;
}

// 受信メッセージがあるか確認
bool mcp25625_t::check_receive() {
    uint8_t status = _read_register(MCP_CANINTF);
    // // 取得したバッファ内容をダンプ
    // printf("CANINTF: 0x%02X\n", status);
    return (status & (MCP_RX0IF | MCP_RX1IF)) != 0;
}

// 受信バッファからCANメッセージを読み出す
bool mcp25625_t::read_can_message(struct can_frame_t* frame) {
    if (!check_receive()) {
        return false;
    }
    uint8_t status = _read_register(MCP_CANINTF);
    if (status & MCP_RX0IF) {
        // RXB0にメッセージあり
        uint8_t rx_buf[13];
        _read_registers(MCP_RXB0SIDH, rx_buf, 13);

        frame->can_id = (rx_buf[0] << 3) | (rx_buf[1] >> 5);
        frame->can_dlc = rx_buf[4] & 0x0F;
        for (int i = 0; i < frame->can_dlc; ++i) {
            frame->data[i] = rx_buf[5 + i];
        }
        // RX0IFフラグをクリア
        _modify_register(MCP_CANINTF, MCP_RX0IF, 0x00);
        return true;
    } else if (status & MCP_RX1IF) {
        // RXB1にメッセージあり
        uint8_t rx_buf[13];
        _read_registers(MCP_RXB1SIDH, rx_buf, 13);

        frame->can_id = (rx_buf[0] << 3) | (rx_buf[1] >> 5);
        frame->can_dlc = rx_buf[4] & 0x0F;
        for (int i = 0; i < frame->can_dlc; ++i) {
            frame->data[i] = rx_buf[5 + i];
        }
        // RX1IFフラグをクリア
        _modify_register(MCP_CANINTF, MCP_RX1IF, 0x00);
        return true;
    }
    return false;
}

// --- プライベートヘルパー関数 ---

void mcp25625_t::_reset() {
    // ハードウェアリセット
    gpio_put(_rst_pin, 0);
    sleep_us(10);
    gpio_put(_rst_pin, 1);
    sleep_ms(10);  // オシレータ安定待ち [cite: 1034]

    // SPIソフトウェアリセット
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, (const uint8_t[]){MCP_RESET}, 1);
    gpio_put(_cs_pin, 1);
}

bool mcp25625_t::_set_mode(uint8_t mode) {
    // CANCTRLレジスタの上位3ビットを設定
    _modify_register(MCP_CANCTRL, 0xE0, mode);
    printf("[DEBUG] Setting mode to 0x%02X\n", mode);

    // モードが正しく設定されたか確認 [cite: 286]
    uint8_t status = _read_register(MCP_CANSTAT);
    printf("[DEBUG] Current CANSTAT: 0x%02X\n", status);
    return (status & 0xE0) == mode;
}

bool mcp25625_t::_set_bit_timing(CAN_SPEED speed) {
    // クロックは16MHzと仮定(FOSC = 16MHz)
    // TQ = 2 * (BRP + 1) / FOSC [ns]
    uint8_t cnf1, cnf2, cnf3;
    switch (speed) {
        case CAN_1000KBPS:  // 1Mbpsの場合 [cite: p45]
            cnf1 = 0x80;    // SJW(同期ジャンプ幅)=1, BRP(baudレートプリスケーラ)=0 TQ=125ns
            cnf2 = 0xD0;    // BTLMODE(ビット時間長)=1, SAM(サンプルポイントコンフィギュレーション)=1(3回サンプリングする), PHSEG1(PS1 セグメント長)=3, PRSEG(伝播セグメント長)=1
            cnf3 = 0x02;    // WAKFIL=0, PHSEG2=2
            break;
            // 1ビットの全長:
            // SYNC(1 TQ) + PRSEG(2 TQ) + PHSEG1(3 TQ) + PHSEG2(2 TQ) = 8 TQ
        default:
            return false;  // 他の速度は未対応
    }

    _write_register(MCP_CNF1, cnf1);
    _write_register(MCP_CNF2, cnf2);
    _write_register(MCP_CNF3, cnf3);
    return true;
}

// 低レベルSPI関数
void mcp25625_t::_write_register(uint8_t address, uint8_t value) {
    uint8_t buf[3] = {MCP_WRITE, address, value};
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, buf, 3);
    gpio_put(_cs_pin, 1);
}

void mcp25625_t::_write_registers(uint8_t address, const uint8_t* values, uint8_t len) {
    uint8_t buf[2] = {MCP_WRITE, address};
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, buf, 2);
    spi_write_blocking(_spi, values, len);
    gpio_put(_cs_pin, 1);
}

uint8_t mcp25625_t::_read_register(uint8_t address) {
    uint8_t cmd[2] = {MCP_READ, address};
    uint8_t data;
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, cmd, 2);
    spi_read_blocking(_spi, 0, &data, 1);
    gpio_put(_cs_pin, 1);
    return data;
}

void mcp25625_t::_read_registers(uint8_t address, uint8_t* values, uint8_t len) {
    uint8_t cmd[2] = {MCP_READ, address};
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, cmd, 2);
    spi_read_blocking(_spi, 0, values, len);
    gpio_put(_cs_pin, 1);
}

void mcp25625_t::_modify_register(uint8_t address, uint8_t mask, uint8_t data) {
    uint8_t buf[4] = {MCP_BITMOD, address, mask, data};
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, buf, 4);
    gpio_put(_cs_pin, 1);
}

void mcp25625_t::_request_to_send(uint8_t instruction) {
    gpio_put(_cs_pin, 0);
    spi_write_blocking(_spi, &instruction, 1);
    gpio_put(_cs_pin, 1);
}