#include "robomaster_motor.hpp"

robomaster_motor_t::robomaster_motor_t(int16_t motor_id, float gear_ratio)
    : motor_id_(motor_id),
      gear_ratio_(gear_ratio),
      prev_encoder_raw_(0),
      encoder_turns_(0),
      continuous_angle_(0.0),
      angular_velocity_(0.0) {
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
    angular_velocity_ = rpm * 2.0 * 3.14159265359 / 60.0 / gear_ratio_;
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
    continuous_angle_ = 0.0;
    // 注意: prev_encoder_raw_はリセットしない（現在の生値を保持）
}

void robomaster_motor_t::parse_can_data(const unsigned char* rx_buf, int16_t* angle_raw, int16_t* rpm, int16_t* current_raw, int8_t* temperature) {
    *angle_raw = rx_buf[0] << 8 | rx_buf[1];    // エンコーダ生値（0-8191）
    *rpm = rx_buf[2] << 8 | rx_buf[3];          // RPM値
    *current_raw = rx_buf[4] << 8 | rx_buf[5];  // 電流値（-16384～16384）
    *temperature = rx_buf[6];                   // 温度
}
