import numpy as np
import os
import glob
from scipy.optimize import least_squares

def list_log_files(log_dir="log"):
    """logフォルダ内のCSVファイル一覧を表示し、ユーザーに選択させる"""
    if not os.path.exists(log_dir):
        print(f"ログフォルダ '{log_dir}' が見つかりません。")
        return None
    
    csv_files = glob.glob(os.path.join(log_dir, "*.csv"))
    if not csv_files:
        print(f"ログフォルダ '{log_dir}' にCSVファイルが見つかりません。")
        return None
    
    print(f"\n利用可能なログファイル ({log_dir}フォルダ内):")
    for i, file_path in enumerate(csv_files, 1):
        file_name = os.path.basename(file_path)
        file_size = os.path.getsize(file_path)
        print(f"{i}. {file_name} ({file_size:,} bytes)")
    
    while True:
        try:
            choice = input(f"\nファイルを選択してください (1-{len(csv_files)}): ").strip()
            choice_num = int(choice)
            if 1 <= choice_num <= len(csv_files):
                selected_file = csv_files[choice_num - 1]
                print(f"選択されたファイル: {selected_file}")
                return selected_file
            else:
                print(f"1から{len(csv_files)}の間で選択してください。")
        except ValueError:
            print("無効な入力です。数字を入力してください。")
        except KeyboardInterrupt:
            print("\n選択をキャンセルしました。")
            return None

def load_log_data(file_path):
    """ログファイルからデータを読み込む"""
    try:
        print(f"ログファイルを読み込み中: {file_path}")
        
        # CSVファイルを読み込み
        data = []
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            header = lines[0].strip().split(',')
            print(f"ヘッダー: {header}")
            
            for line in lines[1:]:  # ヘッダーをスキップ
                values = line.strip().split(',')
                if len(values) >= 3:  # timestamp, current, rpm
                    try:
                        timestamp_us = float(values[0])
                        current = float(values[1])
                        rpm = float(values[2])
                        data.append([timestamp_us, current, rpm])
                    except ValueError:
                        continue  # 不正なデータ行をスキップ
        
        if not data:
            print("有効なデータが見つかりませんでした。")
            return None, None, None
        
        data = np.array(data)
        
        # データを変数に分割
        timestamp_us = data[:, 0]  # マイクロ秒
        current_data = data[:, 1]  # 電流 [A]
        rpm_data = data[:, 2]      # RPM
        
        # 時刻をマイクロ秒から秒に変換
        time_data = timestamp_us / 1000000.0  # 秒に変換
        
        # RPMをrad/sに変換
        omega_out_data = rpm_data * 2 * np.pi / 60.0  # [rad/s]
        
        print(f"データ読み込み完了:")
        print(f"  データ点数: {len(time_data)}")
        print(f"  時間範囲: {time_data[0]:.3f}s ~ {time_data[-1]:.3f}s")
        print(f"  電流範囲: {np.min(current_data):.3f}A ~ {np.max(current_data):.3f}A")
        print(f"  RPM範囲: {np.min(rpm_data):.1f} ~ {np.max(rpm_data):.1f}")
        print(f"  角速度範囲: {np.min(omega_out_data):.3f} ~ {np.max(omega_out_data):.3f} [rad/s]")
        
        return time_data, current_data, omega_out_data
        
    except Exception as e:
        print(f"エラー: {e}")
        return None, None, None

# R
# Kt=0.3 # トルク定数 [N*m/A]
# G= 3591.0 / 187.0 * 3.0 # ギア比
# eta=0.5 # 効率

# P
Kt = 0.18  # トルク定数 [N*m/A]
G = 36.0  # ギア比
eta = 0.66  # 効率
# K_eq は事前に計算または仮定した値
K_eq = Kt * G * eta

# 測定データの読み込み
print("=== ログファイルの選択 ===")
log_file_path = list_log_files()
if log_file_path is None:
    print("ログファイルが選択されませんでした。プログラムを終了します。")
    exit()

time_data, current_data, omega_out_data = load_log_data(log_file_path)
if time_data is None or current_data is None or omega_out_data is None:
    print("データの読み込みに失敗しました。プログラムを終了します。")
    exit()

# 微分項の計算 (中心差分など)
dt = time_data[1] - time_data[0]
d_omega_dt_data = np.gradient(omega_out_data, dt)

# 最小二乗法で解くための誤差関数を定義
def residuals(params, omega_out, current, d_omega_dt, K_eq):
    # params = [J_eq, D_eq]
    J_eq, D_eq = params
    # モデルの予測する d_omega_dt
    predicted_d_omega_dt = (K_eq * current - D_eq * omega_out) / J_eq
    return d_omega_dt - predicted_d_omega_dt

# 初期値の設定 (重要！物理的に妥当な値から始める)
# 例: J_eq=0.001, D_eq=0.01 程度のオーダー
initial_params = [0.001, 0.01]

# 最小二乗法で最適化
# bounds=(lower_bounds, upper_bounds) でパラメータの範囲を指定すると安定しやすい
# 慣性モーメントも摩擦係数も正の値なので、下限は0にする
result = least_squares(residuals, initial_params,
                       args=(omega_out_data, current_data, d_omega_dt_data, K_eq),
                       bounds=([1e-6, 1e-6], [1, 1]))

J_eq_identified, D_eq_identified = result.x

print(f"同定された等価慣性モーメント J_eq: {J_eq_identified:.6f} kg*m^2")
print(f"同定された等価粘性摩擦係数 D_eq: {D_eq_identified:.6f} N*m*s/rad")

# 結果の検証
# 同定されたモデルを使って応答をシミュレーションし、実測データと比較する
# モデルの挙動が実測データとよく一致すれば、同定は成功とみなせる

print("\n=== 結果の検証 ===")

# 同定されたパラメータでモデルの応答をシミュレーション
simulated_d_omega_dt = (K_eq * current_data - D_eq_identified * omega_out_data) / J_eq_identified

# 誤差の計算
error = d_omega_dt_data - simulated_d_omega_dt
rmse = np.sqrt(np.mean(error**2))
mae = np.mean(np.abs(error))
max_error = np.max(np.abs(error))

# 相関係数の計算
correlation = np.corrcoef(d_omega_dt_data, simulated_d_omega_dt)[0, 1]

print(f"検証結果:")
print(f"  RMSE (Root Mean Square Error): {rmse:.6f} rad/s²")
print(f"  MAE (Mean Absolute Error): {mae:.6f} rad/s²")
print(f"  最大誤差: {max_error:.6f} rad/s²")
print(f"  相関係数: {correlation:.6f}")

# 決定係数 (R²) の計算
ss_res = np.sum(error**2)
ss_tot = np.sum((d_omega_dt_data - np.mean(d_omega_dt_data))**2)
r_squared = 1 - (ss_res / ss_tot)
print(f"  決定係数 (R²): {r_squared:.6f}")

# 適合度の判定
print(f"\n適合度評価:")
if r_squared > 0.9:
    print("  優秀: モデルは実測データを非常によく説明しています")
elif r_squared > 0.8:
    print("  良好: モデルは実測データをよく説明しています")
elif r_squared > 0.6:
    print("  普通: モデルは実測データをある程度説明しています")
else:
    print("  不良: モデルの適合度が低いです。パラメータや初期値を見直してください")

# データの統計情報
print(f"\nデータ統計:")
print(f"  実測d_omega_dt 平均: {np.mean(d_omega_dt_data):.6f} rad/s²")
print(f"  実測d_omega_dt 標準偏差: {np.std(d_omega_dt_data):.6f} rad/s²")
print(f"  シミュレーションd_omega_dt 平均: {np.mean(simulated_d_omega_dt):.6f} rad/s²")
print(f"  シミュレーションd_omega_dt 標準偏差: {np.std(simulated_d_omega_dt):.6f} rad/s²")

# 結果の保存（オプション）
try:
    import matplotlib.pyplot as plt
    
    print(f"\n=== Graph Display ===")
    
    # グラフの作成
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Parameter Identification Results', fontsize=14)
    
    # 1. 時系列比較
    axes[0, 0].plot(time_data, d_omega_dt_data, 'b-', label='Measured', alpha=0.7)
    axes[0, 0].plot(time_data, simulated_d_omega_dt, 'r-', label='Model', alpha=0.7)
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Angular Acceleration [rad/s²]')
    axes[0, 0].set_title('Time Series Comparison')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # 2. 散布図（相関）
    axes[0, 1].scatter(d_omega_dt_data, simulated_d_omega_dt, alpha=0.6)
    min_val = min(np.min(d_omega_dt_data), np.min(simulated_d_omega_dt))
    max_val = max(np.max(d_omega_dt_data), np.max(simulated_d_omega_dt))
    axes[0, 1].plot([min_val, max_val], [min_val, max_val], 'r--', label='Ideal Line')
    axes[0, 1].set_xlabel('Measured [rad/s²]')
    axes[0, 1].set_ylabel('Model [rad/s²]')
    axes[0, 1].set_title(f'Correlation Plot (R² = {r_squared:.3f})')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # 3. 誤差の時系列
    axes[1, 0].plot(time_data, error, 'g-', alpha=0.7)
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Error [rad/s²]')
    axes[1, 0].set_title(f'Error Time Series (RMSE = {rmse:.6f})')
    axes[1, 0].grid(True)
    
    # 4. 入力データ（電流と回転数）
    ax4_1 = axes[1, 1]
    ax4_2 = ax4_1.twinx()
    
    line1 = ax4_1.plot(time_data, current_data, 'b-', label='Current [A]')
    line2 = ax4_2.plot(time_data, omega_out_data, 'r-', label='Angular Velocity [rad/s]')
    
    ax4_1.set_xlabel('Time [s]')
    ax4_1.set_ylabel('Current [A]', color='b')
    ax4_2.set_ylabel('Angular Velocity [rad/s]', color='r')
    ax4_1.tick_params(axis='y', labelcolor='b')
    ax4_2.tick_params(axis='y', labelcolor='r')
    ax4_1.set_title('Input Data')
    ax4_1.grid(True)
    
    # 凡例を追加
    lines1, labels1 = ax4_1.get_legend_handles_labels()
    lines2, labels2 = ax4_2.get_legend_handles_labels()
    ax4_1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.tight_layout()
    plt.show()
    
    print("Graph displayed successfully.")
    
except ImportError:
    print("\nmatplotlib is not installed. Skipping graph display.")
    print("To display graphs, install matplotlib with 'pip install matplotlib'.")

print(f"\n=== パラメータ同定完了 ===")
print(f"同定されたパラメータ:")
print(f"  J_eq = {J_eq_identified:.6f} kg*m²")
print(f"  D_eq = {D_eq_identified:.6f} N*m*s/rad")
print(f"  K_eq = {K_eq:.6f} N*m/A (事前設定値)")
print(f"検証指標:")
print(f"  R² = {r_squared:.6f}")
print(f"  RMSE = {rmse:.6f} rad/s²")