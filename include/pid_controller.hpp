#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cstdint>

#include "pico/stdlib.h"

/**
 * @brief PIDコントローラクラス
 *
 * 汎用的なPIDコントローラを実装
 * 速度I-P制御や位置PID制御に対応
 */
class PIDController {
   private:
    // PIDゲイン
    float kp;  // 比例ゲイン
    float ki;  // 積分ゲイン
    float kd;  // 微分ゲイン

    // 内部状態
    float prev_error;  // 前回の偏差
    float integral;    // 積分値
    float prev_input;  // 前回の入力値（微分先行型用）

    // 制限値
    float output_min;    // 出力最小値
    float output_max;    // 出力最大値
    float integral_min;  // 積分最小値（アンチワインドアップ）
    float integral_max;  // 積分最大値（アンチワインドアップ）

    // 設定フラグ
    bool use_derivative_on_input;  // 微分先行型フラグ
    bool first_run;                // 初回実行フラグ

    // サンプリング時間
    float dt;  // サンプリング時間[s]

   public:
    /**
     * @brief コンストラクタ
     * @param kp_val 比例ゲイン
     * @param ki_val 積分ゲイン
     * @param kd_val 微分ゲイン
     * @param sample_time サンプリング時間[s]
     */
    PIDController(float kp_val = 0.0, float ki_val = 0.0, float kd_val = 0.0, float sample_time = 0.001);

    /**
     * @brief PIDゲインを設定
     * @param kp_val 比例ゲイン
     * @param ki_val 積分ゲイン
     * @param kd_val 微分ゲイン
     */
    void setGains(float kp_val, float ki_val, float kd_val);

    /**
     * @brief 出力制限を設定
     * @param min_val 最小値
     * @param max_val 最大値
     */
    void setOutputLimits(float min_val, float max_val);

    /**
     * @brief 積分制限を設定（アンチワインドアップ）
     * @param min_val 積分最小値
     * @param max_val 積分最大値
     */
    void setIntegralLimits(float min_val, float max_val);

    /**
     * @brief 微分先行型を有効/無効にする
     * @param enable true=微分先行型, false=通常微分
     */
    void setDerivativeOnInput(bool enable);

    /**
     * @brief サンプリング時間を設定
     * @param sample_time サンプリング時間[s]
     */
    void setSampleTime(float sample_time);

    /**
     * @brief PIDコントローラをリセット
     */
    void reset();

    /**
     * @brief PID制御計算（標準型）
     * @param setpoint 目標値
     * @param input 現在値
     * @return PID出力
     */
    float compute(float setpoint, float input);

    /**
     * @brief I-P制御計算（積分先行型）
     * 速度制御などでオーバーシュートを抑制したい場合に使用
     * @param setpoint 目標値
     * @param input 現在値
     * @return I-P出力
     */
    float computeIP(float setpoint, float input);

    /**
     * @brief PI-D制御計算（微分先行型）
     * @param setpoint 目標値
     * @param input 現在値
     * @return PI-D出力
     */
    float computePID(float setpoint, float input);

    /**
     * @brief 現在の積分値を取得
     * @return 積分値
     */
    float getIntegral() const { return integral; }

    /**
     * @brief 前回の偏差を取得
     * @return 前回偏差
     */
    float getPrevError() const { return prev_error; }

    /**
     * @brief デバッグ情報を出力
     * @param prefix 出力プレフィックス
     * @param error 現在の偏差
     * @param output 出力値
     */
    void printDebugInfo(const char* prefix, float error, float output) const;
};

/**
 * @brief 速度I-P制御クラス
 *
 * 速度制御に特化したI-P制御器
 */
class VelocityIPController : public PIDController {
   public:
    /**
     * @brief コンストラクタ
     * @param ki_val 積分ゲイン
     * @param kp_val 比例ゲイン
     * @param sample_time サンプリング時間[s]
     */
    VelocityIPController(float ki_val = 0.0, float kp_val = 0.0, float sample_time = 0.001);

    /**
     * @brief 速度I-P制御計算
     * @param target_velocity 目標速度
     * @param current_velocity 現在速度
     * @return 制御出力
     */
    float computeVelocity(float target_velocity, float current_velocity);
};

/**
 * @brief 位置PID制御クラス
 *
 * 位置制御に特化したPID制御器
 */
class PositionPIDController : public PIDController {
   public:
    /**
     * @brief コンストラクタ
     * @param kp_val 比例ゲイン
     * @param ki_val 積分ゲイン
     * @param kd_val 微分ゲイン
     * @param sample_time サンプリング時間[s]
     */
    PositionPIDController(float kp_val = 0.0, float ki_val = 0.0, float kd_val = 0.0, float sample_time = 0.001);

    /**
     * @brief 位置PID制御計算
     * @param target_position 目標位置
     * @param current_position 現在位置
     * @return 制御出力（通常は目標速度）
     */
    float computePosition(float target_position, float current_position);
};

#endif  // PID_CONTROLLER_HPP
