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
    float angle_rad;
    float angle_deg;

    // マルチターン対応変数
    bool is_multiturn;           // マルチターン対応エンコーダかどうか
    int16_t turn_count;          // 回転回数（14ビット符号付き）
    int16_t initial_turn_count;  // 初期化時の回転回数
    float continuous_angle_rad;  // 連続角度[rad]
    float continuous_angle_deg;  // 連続角度[deg]

    // 角速度計算用変数
    float previous_angle_rad;    // 前回の角度[rad]
    float LPF_angle_rad;         // ローパスフィルタの初期値
    float angular_velocity_rad;  // 角速度[rad/s]
    float angular_velocity_deg;  // 角速度[deg/s]
    uint64_t previous_time_us;   // 前回の測定時刻[μs]
    bool velocity_initialized;   // 角速度初期化フラグ

    // AMT223-V コマンド定義
    static const uint8_t CMD_READ_ANGLE = 0x00;  // 角度読み取りコマンド
    static const uint16_t ANGLE_MASK = 0x3FFF;   // 14ビットマスク

    // ゼロ位置セットコマンド（単回転エンコーダのみ）
    static const uint8_t CMD_SET_ZERO[2];  // ゼロ位置セットコマンド: {0x00, 0x70}

    // AMT223D-V マルチターンコマンド定義
    static const uint8_t CMD_READ_MULTITURN[4];  // マルチターン読み取りコマンド: {0x00, 0xA0, 0x00, 0x00}
    static const uint16_t TURN_MASK = 0x3FFF;    // 14ビット回転数マスク

   public:
    static constexpr float COUNTS_PER_REV = 16384.0;  // 2^14 = 16384 counts per revolution
    /**
     * @brief コンストラクタ
     * @param spi_instance SPI インスタンス
     * @param chip_select_pin チップセレクトピン番号
     * @param multiturn_support マルチターン対応エンコーダかどうか（デフォルト: false）
     */
    AMT223V(spi_inst_t* spi_instance, int chip_select_pin, bool multiturn_support = false);

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
     * @brief ゼロ位置をセット（単回転エンコーダのみ）
     * @return セット成功時true
     * @note エンコーダは静止状態である必要があります
     * @note マルチターンエンコーダでは使用できません
     */
    bool set_zero_position();

    /**
     * @brief 生の角度データを取得（0-16383）
     * @return 14ビットの生角度データ
     */
    uint16_t get_raw_angle() const { return raw_angle; }

    /**
     * @brief ラジアン単位の角度を取得（0-2π）
     * @return 角度[rad]
     */
    float get_angle_rad() const { return angle_rad; }

    /**
     * @brief 度単位の角度を取得（0-360）
     * @return 角度[deg]
     */
    float get_angle_deg() const { return angle_deg; }

    /**
     * @brief マルチターン対応かどうか確認
     * @return マルチターン対応時true
     */
    bool is_multiturn_supported() const { return is_multiturn; }

    /**
     * @brief 回転回数を取得（マルチターン対応時のみ）
     * @return 初期化時からの回転回数差分（14ビット符号付き）
     */
    int16_t get_turn_count() const { return turn_count; }

    /**
     * @brief 連続角度を取得（マルチターン対応時のみ）
     * @return 連続角度[rad]
     */
    float get_continuous_angle_rad() const { return continuous_angle_rad; }

    /**
     * @brief 連続角度を取得（マルチターン対応時のみ）
     * @return 連続角度[deg]
     */
    float get_continuous_angle_deg() const { return continuous_angle_deg; }

    /**
     * @brief 角速度を取得（ラジアン/秒）
     * @return 角速度[rad/s]
     */
    float get_angular_velocity_rad() const { return angular_velocity_rad; }

    /**
     * @brief 角速度を取得（度/秒）
     * @return 角速度[deg/s]
     */
    float get_angular_velocity_deg() const { return angular_velocity_deg; }

    /**
     * @brief 回転回数をリセット（マルチターン対応時のみ）
     * @return リセット成功時true
     * @note 現在の回転回数を新しい基準点として設定します
     */
    bool reset_turn_count();

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

    /**
     * @brief パリティチェックを実行
     * @param response 16ビットの受信データ
     * @return パリティチェック成功時true
     */
    bool verify_parity(uint16_t response) const;

    /**
     * @brief 角速度を計算（疑似微分）
     * @param current_angle_rad 現在の角度[rad]
     * @param current_time_us 現在の時刻[μs]
     */
    void calculate_angular_velocity(float current_angle_rad, uint64_t current_time_us);
};

/**
 * @brief AMT223-V エンコーダマネージャクラス
 *
 * 複数のAMT223-Vエンコーダを管理し、SPI設定を統一します
 */
class AMT223V_Manager {
   public:
    // 最大エンコーダ数の定義（変更しやすいように定数として定義）
    static const int MAX_ENCODERS = 2;

   private:
    spi_inst_t* spi_port;
    uint32_t baudrate;
    int pin_miso;
    int pin_sck;
    int pin_mosi;
    std::array<int, MAX_ENCODERS> cs_pins;  // 最大エンコーダ数分のCSピン
    int num_encoders;
    std::array<AMT223V*, MAX_ENCODERS> encoders;  // エンコーダインスタンス

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
     * @param multiturn_support マルチターン対応エンコーダかどうか（デフォルト: false）
     * @return エンコーダのインデックス（-1は失敗）
     */
    int add_encoder(int cs_pin, bool multiturn_support = false);

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
    float get_encoder_angle_rad(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの角度取得
     * @param encoder_index エンコーダインデックス
     * @return 角度[deg]（失敗時は-1.0）
     */
    float get_encoder_angle_deg(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの回転回数取得（マルチターン対応時のみ）
     * @param encoder_index エンコーダインデックス
     * @return 回転回数（失敗時は0）
     */
    int16_t get_encoder_turn_count(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの連続角度取得（マルチターン対応時のみ）
     * @param encoder_index エンコーダインデックス
     * @return 連続角度[rad]（失敗時は0.0）
     */
    float get_encoder_continuous_angle_rad(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの連続角度取得（マルチターン対応時のみ）
     * @param encoder_index エンコーダインデックス
     * @return 連続角度[deg]（失敗時は0.0）
     */
    float get_encoder_continuous_angle_deg(int encoder_index) const;

    /**
     * @brief 指定したエンコーダのゼロ位置をセット（単回転エンコーダのみ）
     * @param encoder_index エンコーダインデックス
     * @return セット成功時true
     */
    bool set_encoder_zero_position(int encoder_index);

    /**
     * @brief 指定したエンコーダの角速度取得
     * @param encoder_index エンコーダインデックス
     * @return 角速度[rad/s]（失敗時は0.0）
     */
    float get_encoder_angular_velocity_rad(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの角速度取得
     * @param encoder_index エンコーダインデックス
     * @return 角速度[deg/s]（失敗時は0.0）
     */
    float get_encoder_angular_velocity_deg(int encoder_index) const;

    /**
     * @brief 指定したエンコーダの回転回数をリセット（マルチターン対応時のみ）
     * @param encoder_index エンコーダインデックス
     * @return リセット成功時true
     */
    bool reset_encoder_turn_count(int encoder_index);

    /**
     * @brief エンコーダ数を取得
     * @return エンコーダ数
     */
    int get_encoder_count() const { return num_encoders; }

    /**
     * @brief 最大エンコーダ数を取得
     * @return 最大エンコーダ数
     */
    static constexpr int get_max_encoders() { return MAX_ENCODERS; }

    /**
     * @brief 現在登録されているエンコーダ数を取得
     * @return 現在のエンコーダ数
     */
    int get_current_encoder_count() const { return num_encoders; }

   private:
    /**
     * @brief エンコーダインデックスの有効性をチェック
     * @param encoder_index チェックするインデックス
     * @return 有効な場合true
     */
    bool is_valid_encoder_index(int encoder_index) const {
        return (encoder_index >= 0 && encoder_index < num_encoders && encoders[encoder_index] != nullptr);
    }
};

#endif  // AMT223V_HPP
