#ifndef DYNAMICS_H
#define DYNAMICS_H

/**
 * 動力学制御クラス
 * 各軸（R軸、P軸）の動力学パラメータを管理し、
 * フィードフォワード制御量を計算する
 */
class dynamics_t {
   private:
    double inertia_mass_;            // 慣性モーメント (R軸) または質量 (P軸)(kg*m^2 または kg)
    double viscous_friction_coeff_;  // 粘性摩擦係数
    double torque_constant_;         // モーターのトルク定数 (Nm/A または Nmm/A)
                                     // P軸の場合、力に変換するための係数 (N/A または N/電流値単位)

   public:
    /**
     * コンストラクタ
     * @param inertia_mass 慣性モーメント (R軸) または質量 (P軸)
     * @param viscous_friction_coeff 粘性摩擦係数
     * @param torque_constant モーターのトルク定数
     */
    dynamics_t(double inertia_mass, double viscous_friction_coeff, double torque_constant);

    /**
     * 動力学計算に基づき、フィードフォワード制御量（トルク/力）を計算する関数。
     * 最終的な電流指令値への変換は呼び出し元で行う。
     * 実際にモーターに送る値は、この戻り値をトルク定数で割った電流値になります。
     * @param target_vel 目標速度 (単位: rad/s または m/s)
     * @param target_accel 目標加速度 (単位: rad/s^2 または m/s^2)
     * @return フィードフォワード制御量（トルク/力）
     */
    double calculate_feedforward_control(double target_vel, double target_accel) const;

    /**
     * フィードフォワード制御量を電流値に変換する関数
     * @param feedforward_torque_force フィードフォワード制御量（トルク/力）
     * @return 電流値 (A)
     */
    double convert_to_current_command(double feedforward_torque_force) const;

    // ゲッター関数
    double get_inertia_mass() const { return inertia_mass_; }
    double get_viscous_friction_coeff() const { return viscous_friction_coeff_; }
    double get_torque_constant() const { return torque_constant_; }

    // セッター関数
    void set_inertia_mass(double inertia_mass) { inertia_mass_ = inertia_mass; }
    void set_viscous_friction_coeff(double viscous_friction_coeff) { viscous_friction_coeff_ = viscous_friction_coeff; }
    void set_torque_constant(double torque_constant) { torque_constant_ = torque_constant; }
};

#endif