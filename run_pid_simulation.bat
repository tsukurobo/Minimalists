@echo off
rem PID制御シミュレーション実行スクリプト (Windows)

echo === PID Control Simulation ===
echo.

rem 仮想環境の確認
if exist ".venv\Scripts\activate.bat" (
    echo 仮想環境を有効化中...
    call .venv\Scripts\activate.bat
) else (
    echo 仮想環境が見つかりません。Python環境を確認してください。
)

rem 必要なパッケージをインストール
echo 必要なパッケージをインストール中...
python -m pip install -r test\requirements.txt

echo.
echo シミュレーション実行中...
echo ================================

rem シミュレーション実行
cd test
python pid_simulation.py

echo.
echo シミュレーション完了！
pause
