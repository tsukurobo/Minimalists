#!/usr/bin/env python3
"""
位置PID+速度I-P制御のシミュレーション
Raspberry Pi Picoプロジェクト用の制御性能評価ツール
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
import json
import argparse

class PIDController:
    """基本PIDコントローラクラス"""
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_input = 0.0
        self.first_run = True
        
        # 制限値
        self.output_min = -1e6
        self.output_max = 1e6
        self.integral_min = -1e6
        self.integral_max = 1e6
    
    def set_output_limits(self, min_val: float, max_val: float):
        """出力制限を設定"""
        self.output_min = min_val
        self.output_max = max_val
    
    def set_integral_limits(self, min_val: float, max_val: float):
        """積分制限を設定"""
        self.integral_min = min_val
        self.integral_max = max_val
    
    def reset(self):
        """PIDコントローラをリセット"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_input = 0.0
        self.first_run = True
    
    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        """値を制限範囲内にクランプ"""
        return max(min_val, min(max_val, value))

class PositionPIDController(PIDController):
    """位置PIDコントローラ（位置制御用）"""
    def compute_position(self, target_position: float, current_position: float) -> float:
        """位置制御を計算（目標位置、現在位置 → 目標速度）"""
        error = target_position - current_position
        
        # 積分項の計算
        self.integral += error * self.dt
        self.integral = self.clamp(self.integral, self.integral_min, self.integral_max)
        
        # 微分項の計算（Derivative on Input）
        derivative = 0.0
        if not self.first_run:
            derivative = -(current_position - self.prev_input) / self.dt
        
        # PID制御の出力計算
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = self.clamp(output, self.output_min, self.output_max)
        
        # 次回のために保存
        self.prev_error = error
        self.prev_input = current_position
        self.first_run = False
        
        return output

class VelocityIPController(PIDController):
    """速度I-Pコントローラ（速度制御用）"""
    def __init__(self, ki: float, kp: float, dt: float):
        super().__init__(kp, ki, 0.0, dt)  # Kd=0でI-P制御
    
    def compute_velocity(self, target_velocity: float, current_velocity: float) -> float:
        """速度制御を計算（目標速度、現在速度 → 目標トルク）"""
        error = target_velocity - current_velocity
        
        # 積分項の計算
        self.integral += error * self.dt
        self.integral = self.clamp(self.integral, self.integral_min, self.integral_max)
        
        # I-P制御の出力計算（比例項は目標値に対して）
        output = self.ki * self.integral + self.kp * target_velocity
        output = self.clamp(output, self.output_min, self.output_max)
        
        # 次回のために保存
        self.prev_error = error
        self.first_run = False
        
        return output

class MotorSimulator:
    """モータシミュレータ
    
    実際のモータの動特性をシミュレートします。
    簡略化した摩擦モデル：粘性摩擦のみ（一定摩擦は外乱として無視）
    
    典型的な値の例：
    - 小型DCモータ: damping=0.05-0.2, torque_constant=0.1-0.5
    - サーボモータ: damping=0.1-0.5, torque_constant=0.2-0.8
    - 大型モータ: damping=0.5-2.0, torque_constant=0.5-2.0
    """
    def __init__(self, inertia: float = 0.01, damping: float = 0.1, 
                 torque_constant: float = 0.3, dt: float = 0.01):
        self.inertia = inertia              # 慣性モーメント [kg·m²]
        self.damping = damping              # 粘性摩擦係数 [N·m·s/rad]
        self.torque_constant = torque_constant  # トルク定数 [N·m/A]
        self.dt = dt                        # 制御周期 [s]
        
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
    
    def update(self, input_torque: float):
        """トルク入力でモータを駆動
        
        簡略化した摩擦モデル:
        τ_friction = damping * ω  （粘性摩擦のみ）
        
        運動方程式:
        J * α = τ_input - τ_friction
        """
        # 粘性摩擦項（速度に比例）のみ考慮
        viscous_friction = self.damping * self.velocity
        
        # 運動方程式: J*α = τ_input - τ_friction
        self.acceleration = (input_torque - viscous_friction) / self.inertia
        
        # オイラー積分で状態更新
        self.velocity += self.acceleration * self.dt
        self.position += self.velocity * self.dt
    
    def set_initial_state(self, position: float = 0.0, velocity: float = 0.0):
        """初期状態設定"""
        self.position = position
        self.velocity = velocity
        self.acceleration = 0.0

def step_response_test(pos_pid: PositionPIDController, vel_ip: VelocityIPController,
                      motor: MotorSimulator, step_amplitude: float = 1.0,
                      simulation_time: float = 5.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """ステップ応答テスト"""
    dt = 0.01
    num_steps = int(simulation_time / dt)
    
    # データ保存用配列
    time_data = np.zeros(num_steps)
    target_data = np.zeros(num_steps)
    position_data = np.zeros(num_steps)
    velocity_data = np.zeros(num_steps)
    torque_data = np.zeros(num_steps)
    
    # 初期化
    motor.set_initial_state(0.0, 0.0)
    pos_pid.reset()
    vel_ip.reset()
    
    # シミュレーション実行
    for i in range(num_steps):
        time = i * dt
        target_position = step_amplitude  # ステップ入力
        
        # 現在状態取得
        current_position = motor.position
        current_velocity = motor.velocity
        
        # 制御計算
        target_velocity = pos_pid.compute_position(target_position, current_position)
        target_torque = vel_ip.compute_velocity(target_velocity, current_velocity)
        
        # モータ更新
        motor.update(target_torque)
        
        # データ保存
        time_data[i] = time
        target_data[i] = target_position
        position_data[i] = current_position
        velocity_data[i] = current_velocity
        torque_data[i] = target_torque
    
    return time_data, target_data, position_data, velocity_data, torque_data

def sinusoidal_tracking_test(pos_pid: PositionPIDController, vel_ip: VelocityIPController,
                            motor: MotorSimulator, amplitude: float = 1.0, frequency: float = 0.5,
                            simulation_time: float = 10.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """正弦波追従テスト"""
    dt = 0.01
    num_steps = int(simulation_time / dt)
    
    # データ保存用配列
    time_data = np.zeros(num_steps)
    target_data = np.zeros(num_steps)
    position_data = np.zeros(num_steps)
    error_data = np.zeros(num_steps)
    
    # 初期化
    motor.set_initial_state(0.0, 0.0)
    pos_pid.reset()
    vel_ip.reset()
    
    # シミュレーション実行
    for i in range(num_steps):
        time = i * dt
        target_position = amplitude * np.sin(2.0 * np.pi * frequency * time)
        
        # 現在状態取得
        current_position = motor.position
        current_velocity = motor.velocity
        
        # 制御計算
        target_velocity = pos_pid.compute_position(target_position, current_position)
        target_torque = vel_ip.compute_velocity(target_velocity, current_velocity)
        
        # モータ更新
        motor.update(target_torque)
        
        # データ保存
        time_data[i] = time
        target_data[i] = target_position
        position_data[i] = current_position
        error_data[i] = target_position - current_position
    
    return time_data, target_data, position_data, error_data

def analyze_step_response(time_data: np.ndarray, target_data: np.ndarray, 
                         position_data: np.ndarray, step_amplitude: float) -> dict:
    """ステップ応答の性能解析"""
    dt = time_data[1] - time_data[0]
    
    # 最終値と定常偏差
    final_position = position_data[-1]
    steady_state_error = abs(step_amplitude - final_position)
    
    # 立ち上がり時間（10%-90%）
    target_10_percent = step_amplitude * 0.1
    target_90_percent = step_amplitude * 0.9
    
    idx_10 = np.where(position_data >= target_10_percent)[0]
    idx_90 = np.where(position_data >= target_90_percent)[0]
    
    rise_time = None
    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time = (idx_90[0] - idx_10[0]) * dt
    
    # 整定時間（±2%以内）
    tolerance = step_amplitude * 0.02
    settling_indices = np.where(np.abs(position_data - step_amplitude) <= tolerance)[0]
    settling_time = None
    if len(settling_indices) > 0:
        # 最後に許容範囲を外れた時刻から整定時間を計算
        for i in range(len(position_data) - 1, -1, -1):
            if abs(position_data[i] - step_amplitude) > tolerance:
                settling_time = (i + 1) * dt
                break
        if settling_time is None:
            settling_time = 0.0
    
    # オーバーシュート
    max_position = np.max(position_data)
    overshoot_percent = ((max_position - step_amplitude) / step_amplitude) * 100.0
    
    return {
        'rise_time': rise_time,
        'settling_time': settling_time,
        'overshoot_percent': overshoot_percent,
        'steady_state_error': steady_state_error,
        'final_position': final_position
    }

def plot_results(time_data: np.ndarray, target_data: np.ndarray, position_data: np.ndarray,
                velocity_data: np.ndarray, torque_data: np.ndarray, title: str, filename: Optional[str] = None):
    """結果をプロット"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(title, fontsize=16)
    
    # 位置応答
    axes[0].plot(time_data, target_data, 'r--', label='Target Position', linewidth=2)
    axes[0].plot(time_data, position_data, 'b-', label='Actual Position', linewidth=2)
    axes[0].set_ylabel('Position [rad]')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].set_title('Position Response')
    
    # 速度応答
    axes[1].plot(time_data, velocity_data, 'g-', label='Velocity', linewidth=2)
    axes[1].set_ylabel('Velocity [rad/s]')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[1].set_title('Velocity Response')
    
    # トルク出力
    axes[2].plot(time_data, torque_data, 'm-', label='Torque', linewidth=2)
    axes[2].set_ylabel('Torque [Nm]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    axes[2].set_title('Control Torque')
    
    plt.tight_layout()
    
    if filename:
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"グラフを保存しました: {filename}")
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='PID制御シミュレーション')
    parser.add_argument('--config', type=str, help='設定ファイル（JSON）')
    parser.add_argument('--save-plots', action='store_true', help='グラフを保存')
    args = parser.parse_args()
    
    # デフォルト設定
    test_cases = [
        {
            'name': 'Standard Parameters',
            'pos_kp': 2.0, 'pos_ki': 0.1, 'pos_kd': 0.05,
            'vel_ki': 0.8, 'vel_kp': 1.2,
            'motor_inertia': 0.01, 'motor_damping': 0.1, 'torque_constant': 0.3
        },
        {
            'name': 'High Gain (Fast Response)',
            'pos_kp': 5.0, 'pos_ki': 0.2, 'pos_kd': 0.1,
            'vel_ki': 1.5, 'vel_kp': 2.0,
            'motor_inertia': 0.01, 'motor_damping': 0.1, 'torque_constant': 0.3
        },
        {
            'name': 'Low Gain (Stable)',
            'pos_kp': 1.0, 'pos_ki': 0.05, 'pos_kd': 0.02,
            'vel_ki': 0.5, 'vel_kp': 0.8,
            'motor_inertia': 0.01, 'motor_damping': 0.1, 'torque_constant': 0.3
        }
    ]
    
    # 設定ファイルがある場合は読み込み
    if args.config:
        with open(args.config, 'r') as f:
            test_cases = json.load(f)
    
    dt = 0.01  # 10ms制御周期
    
    print("=== PID + I-P Control Simulation ===")
    print(f"制御周期: {dt*1000:.1f} ms")
    print()
    
    for i, params in enumerate(test_cases):
        print("=" * 60)
        print(f"テストケース {i+1}: {params['name']}")
        print("=" * 60)
        
        # PIDコントローラの作成
        pos_pid = PositionPIDController(params['pos_kp'], params['pos_ki'], params['pos_kd'], dt)
        vel_ip = VelocityIPController(params['vel_ki'], params['vel_kp'], dt)
        
        # 出力制限設定
        pos_pid.set_output_limits(-10.0, 10.0)
        pos_pid.set_integral_limits(-5.0, 5.0)
        vel_ip.set_output_limits(-16.0, 16.0)
        vel_ip.set_integral_limits(-10.0, 10.0)
        
        # モータシミュレータ作成
        # デフォルト値を設定（設定ファイルにない場合）
        motor_torque_constant = params.get('torque_constant', 0.3)  # デフォルト: 0.3 N·m/A
        motor = MotorSimulator(
            inertia=params['motor_inertia'], 
            damping=params['motor_damping'], 
            torque_constant=motor_torque_constant, 
            dt=dt
        )
        
        # ステップ応答テスト
        print("\n--- ステップ応答テスト (1.0 rad) ---")
        time_data, target_data, position_data, velocity_data, torque_data = \
            step_response_test(pos_pid, vel_ip, motor, 1.0, 5.0)
        
        # 性能解析
        analysis = analyze_step_response(time_data, target_data, position_data, 1.0)
        
        print(f"立ち上がり時間 (10%-90%): {analysis['rise_time']:.4f} [s]" if analysis['rise_time'] else "立ち上がり時間: N/A")
        print(f"整定時間 (±2%): {analysis['settling_time']:.4f} [s]" if analysis['settling_time'] else "整定時間: N/A")
        print(f"オーバーシュート: {analysis['overshoot_percent']:.2f} [%]")
        print(f"定常偏差: {analysis['steady_state_error']:.6f} [rad] ({analysis['steady_state_error']/1.0*100:.2f}%)")
        print(f"最終位置: {analysis['final_position']:.6f} [rad]")
        
        # グラフをプロット
        plot_title = f"Step Response - {params['name']}"
        filename = f"step_response_{params['name'].replace(' ', '_').lower()}.png" if args.save_plots else None
        plot_results(time_data, target_data, position_data, velocity_data, torque_data, plot_title, filename)
        
        print()
    
    print("=== シミュレーション完了 ===")

if __name__ == "__main__":
    main()
