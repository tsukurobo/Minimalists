#include "low_pass_filter.hpp"

#include "pico/time.h"  // time_us_32()

low_pass_filter_t::low_pass_filter_t(float cutoff_freq)
    : cutoff_freq(cutoff_freq), is_initialized(false) {
    reset();
}

bool low_pass_filter_t::reset() {
    dot_value = 0.0f;
    is_initialized = false;
    return true;
}

float low_pass_filter_t::get_lpf_value() const {
    return LPF_value;
}

float low_pass_filter_t::get_dot_value() const {
    return dot_value;
}

float low_pass_filter_t::update(float new_value) {
    uint32_t current_time = time_us_32();
    uint32_t delta_time_us_uint32t = current_time - previous_time_us_uint32t;

    // 初回更新時の処理
    if (!is_initialized) {
        is_initialized = true;
        previous_time_us_uint32t = current_time;
        LPF_value = new_value;
        dot_value = 0.0f;
        return LPF_value;
    }

    // 無効な時間差のチェック（ゼロや異常値はスキップ）
    if (delta_time_us_uint32t == 0 || delta_time_us_uint32t > MAX_DELTA_TIME_US) {
        reset();
        return LPF_value;
    }

    float delta_time_s = static_cast<float>(delta_time_us_uint32t) / 1e6f;

    dot_value = (new_value - LPF_value) * cutoff_freq;
    LPF_value += dot_value * delta_time_s;

    // 時間を更新
    previous_time_us_uint32t = current_time;
    return LPF_value;
}
