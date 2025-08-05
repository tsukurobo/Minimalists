#pragma once

#include <cstdint>

/**
 * @brief ローパスフィルタ構造体
 * @note 初期化完了時には1を返し、通常時には0を返す。
 *       無効な時間差では-1を返す。
 */
class low_pass_filter_t {
   public:
    low_pass_filter_t(float cutoff_freq);

    // 初期化
    bool reset();

    float get_lpf_value() const;
    float get_dot_value() const;

    // 値を更新してフィルタ済み出力を返す
    int update(float new_value);

   private:
    float cutoff_freq;                  // カットオフ周波数 [rad/s]
    uint32_t previous_time_us_uint32t;  // 前回の時間 [us]
    float dot_value;                    // 変化量（1階微分
    float LPF_value;                    // フィルタ済み値
    bool is_initialized;                // 初期化フラグ
};
