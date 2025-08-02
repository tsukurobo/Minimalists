# PID Control Simulation Test

このディレクトリには、位置PID+速度I-P制御の単体テストが含まれています。

## 概要

Raspberry Pi Picoプロジェクト用の制御性能評価ツールです。実際のハードウェアを使わずにPID制御系の応答性能を評価できます。

### 機能

1. **ステップ応答テスト**
   - 目標位置へのステップ入力に対する応答を評価
   - 立ち上がり時間、整定時間、オーバーシュート、定常偏差を計算

2. **正弦波追従テスト**
   - 正弦波状の目標軌道に対する追従性能を評価
   - RMS追従誤差を計算

3. **複数のテストケース**
   - 標準パラメータ
   - 高ゲイン設定（応答性重視）
   - 低ゲイン設定（安定性重視）
   - 重負荷条件

4. **グラフ表示**
   - 位置応答、速度応答、制御トルクをリアルタイムで表示
   - PNG形式での保存機能

### シミュレーションモデル

- **モータモデル**: 慣性、減衰、摩擦を含む2次系
- **制御周期**: 10ms (100Hz)
- **物理パラメータ**: 実際のロボットに近い値を使用

## 実行方法

### 準備

```bash
# 必要なパッケージをインストール
pip install -r test/requirements.txt
```

### 基本実行

```bash
# Windows
run_pid_simulation.bat

# Linux/Mac
chmod +x run_pid_simulation.sh
./run_pid_simulation.sh

# 直接実行
cd test
python pid_simulation.py
```

### 設定ファイルを使用した実行

```bash
cd test
python pid_simulation.py --config pid_config.json --save-plots
```

### VS Code

1. `Ctrl+Shift+P` でコマンドパレットを開く
2. `Tasks: Run Task` を選択
3. `Run PID Simulation` または `Run PID Simulation (with config)` を選択

## 出力例

```
=== PID + I-P Control Simulation ===
制御周期: 10.0 ms

============================================================
テストケース 1: Standard Parameters
============================================================

--- ステップ応答テスト (1.0 rad) ---
立ち上がり時間 (10%-90%): 0.4200 [s]
整定時間 (±2%): 1.2300 [s]
オーバーシュート: 8.50 [%]
定常偏差: 0.002100 [rad] (0.21%)
最終位置: 0.997900 [rad]
```

## 設定ファイル（JSON）

`pid_config.json`で複数のテストケースを定義できます：

```json
[
  {
    "name": "Current Pico Parameters",
    "pos_kp": 2.0,
    "pos_ki": 0.1,
    "pos_kd": 0.05,
    "vel_ki": 0.8,
    "vel_kp": 1.2,
    "motor_inertia": 0.01,
    "motor_damping": 0.1
  }
]
```

## パラメータチューニングの指針

### 応答性を上げたい場合
- 位置PIDのKpを増加 (`pos_kp`)
- 速度I-PのKpを増加 (`vel_kp`)
- ただし、オーバーシュートや振動に注意

### 安定性を重視したい場合
- 位置PIDのKdを増加 (`pos_kd`)
- 全体的なゲインを下げる
- 積分制限を設定

### 定常偏差を減らしたい場合
- 位置PIDのKiを増加 (`pos_ki`)
- 速度I-PのKiを増加 (`vel_ki`)
- ただし、積分飽和に注意

## ファイル構成

- `pid_simulation.py`: メインのシミュレーションプログラム
- `pid_config.json`: テストケース設定ファイル
- `requirements.txt`: 必要なPythonパッケージ
- `run_pid_simulation.bat/sh`: 実行スクリプト
- `README.md`: このファイル

## 注意事項

- Pythonの仮想環境での実行を推奨します
- 実際のハードウェアとは特性が異なる場合があります
- パラメータの最終調整は実機で行うことを推奨します
- グラフ表示にはGUIが必要です（WSLの場合は別途設定が必要）
