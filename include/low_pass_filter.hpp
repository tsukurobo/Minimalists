#pragma once

#include <cstdint>

/**
 * @brief ローパスフィルタ構造体
 */
class low_pass_filter_t {
   public:
    low_pass_filter_t(float cutoff_freq);

    // 初期化
    bool reset();

    float get_lpf_value() const;
    float get_dot_value() const;

    // 値を更新してフィルタ済み出力を返す
    float update(float new_value);

   private:
    float cutoff_freq;                  // カットオフ周波数 [rad/s]
    uint32_t previous_time_us_uint32t;  // 前回の時間 [us]
    float dot_value;                    // 変化量（1階微分
    float LPF_value;                    // フィルタ済み値
    bool is_initialized;                // 初期化フラグ
};
