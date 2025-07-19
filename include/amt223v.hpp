#ifndef AMT223V_HPP
#define AMT223V_HPP

#include <array>
#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

/**
 * @brief AMT223-V 磁気エンコーダクラス
 *
 * AMT223-Vは14ビット分解能の磁気式絶対エンコーダです
 * SPI通信で角度データを取得します
 */
class AMT223V {
   private:
    spi_inst_t* spi_port;
    int cs_pin;
    uint16_t raw_angle;
    double angle_rad;
    double angle_deg;

    // AMT223-V コマンド定義
    static const uint8_t CMD_READ_ANGLE = 0x00;  // 角度読み取りコマンド
    static const uint16_t ANGLE_MASK = 0x3FFF;   // 14ビットマスク

   public:
    static constexpr double COUNTS_PER_REV = 16384.0;  // 2^14 = 16384 counts per revolution
    /**
     * @brief コンストラクタ
     * @param spi_instance SPI インスタンス
     * @param chip_select_pin チップセレクトピン番号
     */
    AMT223V(spi_inst_t* spi_instance, int chip_select_pin);

    /**
     * @brief エンコーダの初期化
     * @return 初期化成功時true
     */
    bool init();

    /**
     * @brief 角度データの読み取り
     * @return 読み取り成功時true
     */
    bool read_angle();

    /**
     * @brief 生の角度データを取得（0-16383）
     * @return 14ビットの生角度データ
     */
    uint16_t get_raw_angle() const { return raw_angle; }

    /**
     * @brief ラジアン単位の角度を取得（0-2π）
     * @return 角度[rad]
     */
    double get_angle_rad() const { return angle_rad; }

    /**
     * @brief 度単位の角度を取得（0-360）
     * @return 角度[deg]
     */
    double get_angle_deg() const { return angle_deg; }

   private:
    /**
     * @brief CSピンを選択状態にする
     */
    void select();

    /**
     * @brief CSピンを非選択状態にする
     */
    void deselect();

    /**
     * @brief SPI通信でデータを送受信
     * @param tx_data 送信データ
     * @param rx_buffer 受信バッファ
     * @param length データ長
     */
    void spi_transfer(const uint8_t* tx_data, uint8_t* rx_buffer, size_t length);
};

/**
 * @brief AMT223-V エンコーダマネージャクラス
 *
 * 複数のAMT223-Vエンコーダを管理し、SPI設定を統一します
 */
class AMT223V_Manager {
   private:
    spi_inst_t* spi_port;
    uint32_t baudrate;
    int pin_miso;
    int pin_sck;
    int pin_mosi;
    std::array<int, 4> cs_pins;  // 最大4つのCSピン
    int num_encoders;
    std::array<AMT223V*, 4> encoders;  // エンコーダインスタンス

   public:
    /**
     * @brief コンストラクタ
     * @param spi_instance SPI インスタンス
     * @param baud SPI ボーレート
     * @param miso MISOピン
     * @param sck SCKピン
     * @param mosi MOSIピン
     */
    AMT223V_Manager(spi_inst_t* spi_instance, uint32_t baud, int miso, int sck, int mosi);

    /**
     * @brief エンコーダを追加
     * @param cs_pin CSピン番号
     * @return エンコーダのインデックス（-1は失敗）
     */
    int add_encoder(int cs_pin);

    /**
     * @brief SPI設定を初期化
     * @return 初期化成功時true
     */
    bool init_spi();

    /**
     * @brief 全エンコーダを初期化
     * @return 初期化成功時true
     */
    bool init_all_encoders();

    /**
     * @brief 指定したエンコーダの角度を読み取り
     * @param encoder_index エンコーダインデックス
     * @return 読み取り成功時true
     */
    bool read_encoder(int encoder_index);

    /**
     * @brief 全エンコーダの角度を読み取り
     * @return 読み取り成功したエンコーダ数
     */
    int read_all_encoders();

    /**
     * @brief 指定したエンコーダの角度取得
     * @param encoder_index エンコーダインデックス
     * @return 角度[rad]（失敗時は-1.0）
     */
    double get_encoder_angle_rad(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの角度取得
     * @param encoder_index エンコーダインデックス
     * @return 角度[deg]（失敗時は-1.0）
     */
    double get_encoder_angle_deg(int encoder_index) const;

    /**
     * @brief エンコーダ数を取得
     * @return エンコーダ数
     */
    int get_encoder_count() const { return num_encoders; }
};

#endif  // AMT223V_HPP
