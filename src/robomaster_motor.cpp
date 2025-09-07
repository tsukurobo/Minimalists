#include "robomaster_motor.hpp"

bool send_all_motor_currents(mcp25625_t* can, float currents[4]) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x200;  // モータ1~4制御ID
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; ++i) tx_frame.data[i] = 0;

    // 4モータ分をパック
    for (int i = 0; i < 4; ++i) {
        // 変換式はrobomaster_motor_t::current_to_rawと同じ
        int16_t raw = static_cast<int16_t>(currents[i] / (20.0f / 16384.0f));
        tx_frame.data[i * 2] = static_cast<uint8_t>(raw >> 8);
        tx_frame.data[i * 2 + 1] = static_cast<uint8_t>(raw);
    }

    return can->send_can_message(&tx_frame);
}

robomaster_motor_t::robomaster_motor_t(mcp25625_t* can, int16_t motor_id, float gear_ratio)
    : prev_encoder_raw_(0),
      encoder_turns_(0),
      continuous_angle_(0.0f),
      angular_velocity_(0.0f),
      gear_ratio_(gear_ratio),
      motor_id_(motor_id),
      can_(can) {}

bool robomaster_motor_t::send_current(float current_amp) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x200;  // 例: モータ1~4制御ID
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; ++i) tx_frame.data[i] = 0;

    int16_t raw = current_to_raw(current_amp);
    // モータIDに応じて該当バイトに格納
    int idx = (motor_id_ - 1) * 2;
    tx_frame.data[idx] = (uint8_t)(raw >> 8);
    tx_frame.data[idx + 1] = (uint8_t)(raw);

    return can_->send_can_message(&tx_frame);
}

bool robomaster_motor_t::receive_feedback() {
    can_frame_t rx_frame;
    if (!can_->read_can_message(&rx_frame)) return false;
    if (rx_frame.can_id != static_cast<uint32_t>(0x200 + motor_id_)) return false;

    int16_t angle_raw = (rx_frame.data[0] << 8) | rx_frame.data[1];
    int16_t rpm = (rx_frame.data[2] << 8) | rx_frame.data[3];
    // int16_t current_raw = (rx_frame.data[4] << 8) | rx_frame.data[5];
    // int8_t temperature = rx_frame.data[6]; // 必要なら保存

    update_encoder_angle(angle_raw);
    rpm_to_angular_velocity(rpm);
    // 必要ならcurrent_rawを保存

    return true;
}

float robomaster_motor_t::update_encoder_angle(int16_t encoder_raw) {
    // エンコーダ値の差分を計算
    int16_t diff = encoder_raw - prev_encoder_raw_;

    // 8192→0への遷移を検出（正方向の回転）
    if (diff < -ENCODER_MAX / 2) {
        encoder_turns_++;
        // Serial.println("Forward wrap detected: 8192->0");
    }
    // 0→8192への遷移を検出（負方向の回転）
    else if (diff > ENCODER_MAX / 2) {
        encoder_turns_--;
        // Serial.println("Backward wrap detected: 0->8192");
    }

    // 連続角度を計算（ギア比を考慮）
    continuous_angle_ = (encoder_turns_ * ENCODER_MAX + encoder_raw) * ENCODER_TO_RAD / gear_ratio_;

    // 次回のために現在値を保存
    prev_encoder_raw_ = encoder_raw;

    return continuous_angle_;
}

float robomaster_motor_t::rpm_to_angular_velocity(int16_t rpm) {
    // RPMをラジアン/秒に変換（ギア比を考慮）
    angular_velocity_ = rpm * 2.0f * 3.14159265359f / 60.0f / gear_ratio_;
    return angular_velocity_;
}

float robomaster_motor_t::raw_to_current(int16_t current_raw) {
    // 生の電流値をアンペアに変換
    return current_raw * CURRENT_CONVERSION_FACTOR;
}

int16_t robomaster_motor_t::current_to_raw(float current_amp) {
    // 電流値（アンペア）を生値に変換
    return (int16_t)(current_amp / CURRENT_CONVERSION_FACTOR);
}

void robomaster_motor_t::reset_encoder() {
    encoder_turns_ = 0;
    continuous_angle_ = 0.0f;
    // 注意: prev_encoder_raw_はリセットしない（現在の生値を保持）
}
