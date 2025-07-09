#pragma once

#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

/**
 * @file mcp25625.hpp
 * @brief MCP25625 CANコントローラ用クラスの宣言と定数定義
 * @author Minimalists
 * @date 2025
 *
 * MCP25625をRaspberry Pi Picoから制御するためのクラス・定数・構造体を定義します。
 */

// データシート 表5-1: SPI命令セット [cite: 1666]
constexpr uint8_t MCP_RESET = 0xC0;        ///< SPIコマンド: ソフトウェアリセット
constexpr uint8_t MCP_READ = 0x03;         ///< SPIコマンド: レジスタ読み出し
constexpr uint8_t MCP_WRITE = 0x02;        ///< SPIコマンド: レジスタ書き込み
constexpr uint8_t MCP_READ_STATUS = 0xA0;  ///< SPIコマンド: ステータス読み出し
constexpr uint8_t MCP_RX_STATUS = 0xB0;    ///< SPIコマンド: 受信バッファステータス
constexpr uint8_t MCP_BITMOD = 0x05;       ///< SPIコマンド: ビット修正
constexpr uint8_t MCP_LOAD_TXB0 = 0x40;    ///< SPIコマンド: 送信バッファ0へのデータロード
constexpr uint8_t MCP_RTS_TXB0 = 0x81;     ///< SPIコマンド: 送信要求（バッファ0）

// データシート 4.0 レジスタマップ [cite: 1125]
constexpr uint8_t MCP_CANSTAT = 0x0E;   ///< CANステータスレジスタ
constexpr uint8_t MCP_CANCTRL = 0x0F;   ///< CAN制御レジスタ
constexpr uint8_t MCP_CNF1 = 0x2A;      ///< コンフィグレーション1 レジスタ
constexpr uint8_t MCP_CNF2 = 0x29;      ///< コンフィグレーション2 レジスタ
constexpr uint8_t MCP_CNF3 = 0x28;      ///< コンフィグレーション3 レジスタ
constexpr uint8_t MCP_CANINTF = 0x2C;   ///< 割り込みフラグレジスタ
constexpr uint8_t MCP_TXB0CTRL = 0x30;  ///< 送信バッファ0制御レジスタ
constexpr uint8_t MCP_RXB0CTRL = 0x60;  ///< 受信バッファ0制御レジスタ
constexpr uint8_t MCP_RXB1CTRL = 0x70;  ///< 受信バッファ1制御レジスタ
constexpr uint8_t MCP_RXB0SIDH = 0x61;  ///< 受信バッファ0標準ID上位レジスタ
constexpr uint8_t MCP_RXB1SIDH = 0x71;  ///< 受信バッファ1標準ID上位レジスタ

// CANCTRLレジスタのモード定義 [cite: p51]
constexpr uint8_t MODE_NORMAL = 0x00;      ///< 通常動作モード（000）
constexpr uint8_t MODE_SLEEP = 0x20;       ///< スリープモード（001）
constexpr uint8_t MODE_LOOPBACK = 0x40;    ///< ループバックモード（010）
constexpr uint8_t MODE_LISTENONLY = 0x60;  ///< リッスンオンリーモード（011）
constexpr uint8_t MODE_CONFIG = 0x80;      ///< コンフィグレーションモード（100）

// CANINTFフラグ [cite: 1591]
constexpr uint8_t MCP_RX0IF = 0x01;  ///< 受信バッファ0フル割り込みフラグ
constexpr uint8_t MCP_RX1IF = 0x02;  ///< 受信バッファ1フル割り込みフラグ

/**
 * @enum CAN_SPEED
 * @brief CAN通信のボーレート設定
 * @note 1Mbps以外の速度は未実装なので、必要に応じて追加してください。
 */
enum CAN_SPEED {
    CAN_1000KBPS
};

/**
 * @struct can_frame
 * @brief CANメッセージ1フレーム分のデータ構造
 */
struct can_frame {
    uint32_t can_id;  ///< CAN ID (11-bit or 29-bit)
    uint8_t can_dlc;  ///< データ長 (0-8)
    uint8_t data[8];  ///< データ
};

/**
 * @class MCP25625
 * @brief MCP25625 CANコントローラを制御するクラス
 *
 * SPI経由でMCP25625を操作し、CAN通信の初期化・送受信を行います。
 */
class MCP25625 {
   public:
    /**
     * @brief コンストラクタ
     * @param spi 使用するSPIインスタンス
     * @param cs_pin チップセレクトピン番号
     * @param rst_pin リセットピン番号
     */
    MCP25625(spi_inst_t* spi, uint8_t cs_pin, uint8_t rst_pin);

    /**
     * @brief MCP25625の初期化
     * @param speed CAN通信速度
     * @param clock_mhz クロック周波数（MHz）（2025/07/06現在は16MHzしか対応していません）
     * @return true:成功, false:失敗
     */
    bool init(CAN_SPEED speed, uint32_t clock_mhz = 16);

    /**
     * @brief CANメッセージの送信
     * @param frame 送信するCANフレーム
     * @return true:成功, false:失敗
     */
    bool send_can_message(const struct can_frame* frame);

    /**
     * @brief CANメッセージの受信
     * @param frame 受信したCANフレームを格納する構造体
     * @return true:成功, false:失敗
     */
    bool read_can_message(struct can_frame* frame);

    /**
     * @brief 受信メッセージがあるか確認
     * @return true:受信あり, false:なし
     */
    bool check_receive();

   private:
    // SPI通信の低レベル関数
    void _write_register(uint8_t address, uint8_t value);
    void _write_registers(uint8_t address, const uint8_t* values, uint8_t len);
    uint8_t _read_register(uint8_t address);
    void _read_registers(uint8_t address, uint8_t* values, uint8_t len);
    /**
     * @brief レジスタの特定ビットを修正
     * @param address レジスタアドレス
     * @param mask 修正するビットマスク
     * @param data 書き込むデータ
     */
    void _modify_register(uint8_t address, uint8_t mask, uint8_t data);

    // SPIコマンド関数
    void _reset();
    void _request_to_send(uint8_t instruction);

    // モード設定
    bool _set_mode(uint8_t mode);

    // ビットタイミング設定
    bool _set_bit_timing(CAN_SPEED speed, uint32_t clock_mhz);

    spi_inst_t* _spi;  ///< SPIインスタンス
    uint8_t _cs_pin;   ///< チップセレクトピン
    uint8_t _rst_pin;  ///< リセットピン
};
