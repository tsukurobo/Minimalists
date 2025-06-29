#pragma once
// #include <cmath>  // For std::fabs, std::sqrt, std::pow, M_PI

// 独自のmin/max関数（Arduino環境対応）
inline double fmax_custom(double a, double b) {
    return (a > b) ? a : b;
}

inline double fmin_custom(double a, double b) {
    return (a < b) ? a : b;
}

// --- PIDController クラス ---
class pid_controller_t {
   public:
    // コンストラクタ
    pid_controller_t(double kp, double ki, double kd,
                     double integral_max, double integral_min,
                     double output_max, double output_min)
        : kp_(kp), ki_(ki), kd_(kd), integral_max_(integral_max), integral_min_(integral_min), output_max_(output_max), output_min_(output_min), prev_error_(0.0), integral_(0.0) {}

    // PIDコントローラをリセットする関数
    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

    // PID制御量を計算する関数
    double calculate(double target_val, double present_val, double dt) {
        // 誤差の計算
        double error = target_val - present_val;

        // P項 (比例項)
        double p_term = kp_ * error;

        // I項 (積分項)
        integral_ += error * dt;
        // 積分ワインドアップ防止 (積分値の制限)
        integral_ = fmax_custom(integral_min_, fmin_custom(integral_max_, integral_));
        double i_term = ki_ * integral_;

        // D項 (微分項)
        // ノイズの影響を考慮し、ここでは誤差の微分を使用していますが、
        // 実際には目標位置の微分と現在位置の微分の差（速度誤差）を使用する方が良い場合もあります。
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        // PID出力の合計
        double output = p_term + i_term + d_term;

        // 出力飽和 (リミット)
        output = fmax_custom(output_min_, fmin_custom(output_max_, output));

        // 次の計算のために現在の誤差を保存
        prev_error_ = error;

        return output;
    }

    // ゲッター関数
    double get_kp() const { return kp_; }
    double get_ki() const { return ki_; }
    double get_kd() const { return kd_; }
    double get_integral_max() const { return integral_max_; }
    double get_integral_min() const { return integral_min_; }
    double get_output_max() const { return output_max_; }
    double get_output_min() const { return output_min_; }
    double get_prev_error() const { return prev_error_; }
    double get_integral() const { return integral_; }

    // セッター関数
    void set_kp(double kp) { kp_ = kp; }
    void set_ki(double ki) { ki_ = ki; }
    void set_kd(double kd) { kd_ = kd; }
    void set_integral_max(double integral_max) { integral_max_ = integral_max; }
    void set_integral_min(double integral_min) { integral_min_ = integral_min; }
    void set_output_max(double output_max) { output_max_ = output_max; }
    void set_output_min(double output_min) { output_min_ = output_min; }

   private:
    double kp_, ki_, kd_;
    double integral_max_, integral_min_;
    double output_max_, output_min_;
    double prev_error_;
    double integral_;
};