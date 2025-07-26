#pragma once

/**
 * 動力学制御クラス
 * 各軸（R軸、P軸）の動力学パラメータを管理し、
 * フィードフォワード制御量を計算する
 */
class dynamics_t {
   private:
    float inertia_mass_;            // 慣性モーメント (kg*m^2)
    float viscous_friction_coeff_;  // 粘性摩擦係数 (N*m*s/rad)
    float torque_constant_;         // モーターのトルク定数 (Nm/A)

   public:
    /**
     * コンストラクタ
     * @param inertia_mass 慣性モーメント
     * @param viscous_friction_coeff 粘性摩擦係数
     * @param torque_constant モーターのトルク定数
     */
    dynamics_t(float inertia_mass, float viscous_friction_coeff, float torque_constant);

    /**
     * 動力学計算に基づき、フィードフォワード制御量（トルク）を計算する関数。
     * 最終的な電流指令値への変換は呼び出し元で行う。
     * 実際にモーターに送る値は、この戻り値をトルク定数で割った電流値になります。
     * @param target_vel 目標速度 (単位: rad/s)
     * @param target_accel 目標加速度 (単位: rad/s^2)
     * @return フィードフォワード制御量（トルク）
     */
    float calculate_feedforward_control(float target_vel, float target_accel) const;

    /**
     * フィードフォワード制御量を電流値に変換する関数
     * @param feedforward_torque_force フィードフォワード制御量（トルク）
     * @return 電流値 (A)
     */
    float convert_to_current_command(float feedforward_torque_force) const;

    // ゲッター関数
    float get_inertia_mass() const { return inertia_mass_; }
    float get_viscous_friction_coeff() const { return viscous_friction_coeff_; }
    float get_torque_constant() const { return torque_constant_; }

    // セッター関数
    void set_inertia_mass(float inertia_mass) { inertia_mass_ = inertia_mass; }
    void set_viscous_friction_coeff(float viscous_friction_coeff) { viscous_friction_coeff_ = viscous_friction_coeff; }
    void set_torque_constant(float torque_constant) { torque_constant_ = torque_constant; }
};
