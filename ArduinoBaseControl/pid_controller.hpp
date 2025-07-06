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

// --- 速度型PIDController クラス ---
// ΔU(k) = Kp*[e(k) - e(k-1)] + Ki*e(k)*dt + Kd*[e(k) - 2*e(k-1) + e(k-2)]/dt
// U(k) = U(k-1) + ΔU(k)
class velocity_pid_controller_t {
   public:
    // コンストラクタ
    velocity_pid_controller_t(double kp, double ki, double kd,
                              double output_max, double output_min)
        : kp_(kp), ki_(ki), kd_(kd), output_max_(output_max), output_min_(output_min), prev_error_(0.0), prev_prev_error_(0.0), prev_output_(0.0) {}

    // 速度型PIDコントローラをリセットする関数
    void reset() {
        prev_error_ = 0.0;
        prev_prev_error_ = 0.0;
        prev_output_ = 0.0;
    }

    // 速度型PID制御量を計算する関数
    // 戻り値は増分値（前回出力からの変化量）
    double calculate_increment(double target_val, double present_val, double dt) {
        // 誤差の計算
        double error = target_val - present_val;

        // 速度型PIDの増分計算
        // ΔU(k) = Kp*[e(k) - e(k-1)] + Ki*e(k)*dt + Kd*[e(k) - 2*e(k-1) + e(k-2)]/dt
        double p_increment = kp_ * (error - prev_error_);                                // 誤差の微分
        double i_increment = ki_ * error * dt;                                           // 現在の誤差のみを使用
        double d_increment = kd_ * (error - 2.0 * prev_error_ + prev_prev_error_) / dt;  // 2つ前の誤差を使用

        double increment = p_increment + i_increment + d_increment;

        // 新しい出力値を計算
        double new_output = prev_output_ + increment;

        // 出力飽和 (リミット)
        new_output = fmax_custom(output_min_, fmin_custom(output_max_, new_output));

        // 次の計算のために誤差と出力を保存
        prev_prev_error_ = prev_error_;
        prev_error_ = error;
        prev_output_ = new_output;

        return increment;
    }

    // 速度型PID制御量を計算する関数（出力値を直接返す）
    double calculate(double target_val, double present_val, double dt) {
        calculate_increment(target_val, present_val, dt);
        return prev_output_;
    }

    // 現在の出力値を取得
    double get_current_output() const { return prev_output_; }

    // 出力値を手動で設定（初期化やリセット時に使用）
    void set_current_output(double output) {
        prev_output_ = fmax_custom(output_min_, fmin_custom(output_max_, output));
    }

    // ゲッター関数
    double get_kp() const { return kp_; }
    double get_ki() const { return ki_; }
    double get_kd() const { return kd_; }
    double get_output_max() const { return output_max_; }
    double get_output_min() const { return output_min_; }
    double get_prev_error() const { return prev_error_; }
    double get_prev_prev_error() const { return prev_prev_error_; }

    // セッター関数
    void set_kp(double kp) { kp_ = kp; }
    void set_ki(double ki) { ki_ = ki; }
    void set_kd(double kd) { kd_ = kd; }
    void set_output_max(double output_max) { output_max_ = output_max; }
    void set_output_min(double output_min) { output_min_ = output_min; }

   private:
    double kp_, ki_, kd_;
    double output_max_, output_min_;
    double prev_error_;       // 前回の誤差 e(k-1)
    double prev_prev_error_;  // 前々回の誤差 e(k-2)
    double prev_output_;      // 前回の出力値 U(k-1)
};