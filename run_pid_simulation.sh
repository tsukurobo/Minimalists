#!/bin/bash
# PID制御シミュレーション実行スクリプト (Linux/Mac)

echo "=== PID Control Simulation ==="
echo

# 仮想環境の確認
if [ -f ".venv/bin/activate" ]; then
    echo "仮想環境を有効化中..."
    source .venv/bin/activate
else
    echo "仮想環境が見つかりません。Python環境を確認してください。"
fi

# 必要なパッケージをインストール
echo "必要なパッケージをインストール中..."
python -m pip install -r test/requirements.txt

echo
echo "シミュレーション実行中..."
echo "================================"

# シミュレーション実行
cd test
python pid_simulation.py

echo
echo "シミュレーション完了！"
