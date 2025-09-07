#pragma once

#include <stdint.h>

#include "mcp25625.hpp"

/**
 * @brief すべてのモータの電流を送信する関数
 *
 * @param can MCP25625インスタンス
 * @param currents 送信する電流値の配列 (アンペア)
 * @return true 送信成功
 * @return false 送信失敗
 */
bool send_all_motor_currents(mcp25625_t* can, float currents[4]);

/**
 * @brief RoboMasterモータ制御クラス
 * M3508モータのエンコーダ読み取り、電流値変換、角度計算、CAN送受信を管理
 */
class robomaster_motor_t {
   private:
    static constexpr int16_t ENCODER_MAX = 8192;
    static constexpr float ENCODER_TO_RAD = 2.0f * 3.14159265359f / ENCODER_MAX;
    static constexpr float CURRENT_CONVERSION_FACTOR = 20.0f / 16384.0f;  // 電流変換係数

    int16_t prev_encoder_raw_;
    int32_t encoder_turns_;
    float continuous_angle_;
    float angular_velocity_;
    float gear_ratio_;
    int16_t motor_id_;  // モータID

    mcp25625_t* can_;  // MCP25625インスタンスへのポインタ

   public:
    /**
     * コンストラクタ
     * @param can MCP25625インスタンス
     * @param motor_id モータID
     * @param gear_ratio ギア比
     */
    robomaster_motor_t(mcp25625_t* can, int16_t motor_id, float gear_ratio);

    /**
     * モータへ電流指令を送信
     * @param current_amp 電流値（アンペア）
     * @return 送信成功時true
     */
    bool send_current(float current_amp);

    /**
     * CANから自身のモータIDのフィードバックを受信し、値を更新
     * @return 受信成功時true
     */
    bool receive_feedback();

    /**
     * エンコーダの生値を連続的な角度（ラジアン）に変換する関数
     * 8192→0または0→8192の遷移を検出して累積角度を計算
     * @param encoder_raw エンコーダの生値（0-8191）
     * @return 連続的な角度（ラジアン）
     */
    float update_encoder_angle(int16_t encoder_raw);

    /**
     * RPM値から角速度（ラジアン/秒）に変換する関数
     * @param rpm RPM値
     * @return 角速度（ラジアン/秒）
     */
    float rpm_to_angular_velocity(int16_t rpm);

    /**
     * 生の電流値をアンペアに変換する関数
     * 電流値は-16384～16384の範囲で、20Aが最大値
     * @param current_raw 生の電流値（-16384～16384）
     * @return アンペア単位の電流値
     */
    float raw_to_current(int16_t current_raw);

    /**
     * 電流値（アンペア）を生値に変換する関数
     * @param current_amp 電流値（アンペア）
     * @return 生の電流値（-16384～16384）
     */
    int16_t current_to_raw(float current_amp);

    /**
     * エンコーダの連続角度をリセットする関数
     * 初期化時や特定の位置を基準にしたい場合に使用
     */
    void reset_encoder();

    // ゲッター関数
    float get_continuous_angle() const { return continuous_angle_; }
    float get_angular_velocity() const { return angular_velocity_; }
    int32_t get_encoder_turns() const { return encoder_turns_; }
    int16_t get_prev_encoder_raw() const { return prev_encoder_raw_; }
    float get_gear_ratio() const { return gear_ratio_; }
    int16_t get_motor_id() const { return motor_id_; }
};
