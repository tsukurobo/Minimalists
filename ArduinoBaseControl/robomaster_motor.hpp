#ifndef ROBOMASTER_MOTOR_H
#define ROBOMASTER_MOTOR_H

#include <stdint.h>

/**
 * RoboMasterモータ制御クラス
 * M3508モータのエンコーダ読み取り、電流値変換、角度計算などを管理
 */
class robomaster_motor_t {
   private:
    // エンコーダ関連の定数
    static constexpr int16_t ENCODER_MAX = 8192;
    static constexpr double ENCODER_TO_RAD = 2.0 * 3.14159265359 / ENCODER_MAX;
    static constexpr double CURRENT_CONVERSION_FACTOR = 20.0 / 16384.0;  // 電流変換係数

    // エンコーダの状態を保持するメンバ変数
    int16_t prev_encoder_raw_;  // 前回のエンコーダ生値
    int32_t encoder_turns_;     // エンコーダの回転数（巻き数）
    double continuous_angle_;   // 現在の連続角度（ラジアン）
    double angular_velocity_;   // 現在の角速度（ラジアン/秒）

    // ギア比とその他のパラメータ
    double gear_ratio_;  // ギア比
    int16_t motor_id_;   // モータID

   public:
    /**
     * コンストラクタ
     * @param motor_id モータID
     * @param gear_ratio ギア比
     */
    robomaster_motor_t(int16_t motor_id, double gear_ratio);

    /**
     * エンコーダの生値を連続的な角度（ラジアン）に変換する関数
     * 8192→0または0→8192の遷移を検出して累積角度を計算
     * @param encoder_raw エンコーダの生値（0-8191）
     * @return 連続的な角度（ラジアン）
     */
    double update_encoder_angle(int16_t encoder_raw);

    /**
     * RPM値から角速度（ラジアン/秒）に変換する関数
     * @param rpm RPM値
     * @return 角速度（ラジアン/秒）
     */
    double rpm_to_angular_velocity(int16_t rpm);

    /**
     * 生の電流値をアンペアに変換する関数
     * 電流値は-16384～16384の範囲で、20Aが最大値
     * @param current_raw 生の電流値（-16384～16384）
     * @return アンペア単位の電流値
     */
    double raw_to_current(int16_t current_raw);

    /**
     * 電流値（アンペア）を生値に変換する関数
     * @param current_amp 電流値（アンペア）
     * @return 生の電流値（-16384～16384）
     */
    int16_t current_to_raw(double current_amp);

    /**
     * エンコーダの連続角度をリセットする関数
     * 初期化時や特定の位置を基準にしたい場合に使用
     */
    void reset_encoder();

    /**
     * CANバッファからモータデータを解析する関数
     * @param rx_buf 受信したCANデータバッファ
     * @param angle_raw エンコーダ生値の出力先
     * @param rpm RPM値の出力先
     * @param current_raw 電流生値の出力先
     * @param temperature 温度の出力先
     */
    void parse_can_data(const unsigned char* rx_buf, int16_t* angle_raw, int16_t* rpm, int16_t* current_raw, int8_t* temperature);

    // ゲッター関数
    double get_continuous_angle() const { return continuous_angle_; }
    double get_angular_velocity() const { return angular_velocity_; }
    int32_t get_encoder_turns() const { return encoder_turns_; }
    int16_t get_prev_encoder_raw() const { return prev_encoder_raw_; }
    double get_gear_ratio() const { return gear_ratio_; }
    int16_t get_motor_id() const { return motor_id_; }

    // セッター関数
    void set_gear_ratio(double gear_ratio) { gear_ratio_ = gear_ratio; }
    void set_angular_velocity(double angular_velocity) { angular_velocity_ = angular_velocity; }
};

#endif
