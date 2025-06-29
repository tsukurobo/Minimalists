import serial
import serial.tools.list_ports
import time
import datetime

# --- 設定 ---
SERIAL_PORT = 'COM4'  # Arduinoが接続されているCOMポート名に合わせて変更 (例: 'COM3'や'/dev/ttyUSB0')
BAUD_RATE = 115200      # Arduinoのスケッチと合わせる
LOG_FILE_BASE = 'sensor_log' # ログファイルのベース名（タイムスタンプが自動追加される）
LOG_INTERVAL_SEC = 0.002  # 何秒ごとにログを保存するか (Arduinoのdelayと合わせると良い)
START_COMMAND = "c"    # Arduinoに送信する動作開始コマンド
STOP_COMMAND = "a0"     # Arduinoに送信する動作停止コマンド
STOP_LOG_DURATION = 30 # 停止コマンド送信後のログ記録時間（秒）
RPM_STABLE_THRESHOLD = 0  # RPMがこの値以下になったら安定とみなす
RPM_STABLE_COUNT = 50     # RPMが安定値を何回連続で取ったら停止するか
# --- 設定ここまで ---

def list_available_ports():
    """利用可能なシリアルポートを一覧表示し、ユーザーに選択させる"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("利用可能なシリアルポートが見つかりませんでした。")
        return None
    
    print("\n利用可能なシリアルポート:")
    for i, port in enumerate(ports, 1):
        print(f"{i}. {port.device} - {port.description}")
    
    while True:
        try:
            choice = input(f"\nポートを選択してください (1-{len(ports)}, または 's' でスキップ): ").strip()
            
            if choice.lower() == 's':
                return None
            
            choice_num = int(choice)
            if 1 <= choice_num <= len(ports):
                selected_port = ports[choice_num - 1].device
                print(f"選択されたポート: {selected_port}")
                return selected_port
            else:
                print(f"1から{len(ports)}の間で選択してください。")
        except ValueError:
            print("無効な入力です。数字を入力してください。")
        except KeyboardInterrupt:
            print("\n選択をキャンセルしました。")
            return None

def test_port_connection(port, baud_rate):
    """指定されたポートが使用可能かテストする"""
    try:
        test_ser = serial.Serial(port, baud_rate, timeout=1)
        test_ser.close()
        return True
    except Exception as e:
        print(f"ポート {port} のテストに失敗: {e}")
        return False

def extract_rpm_value(data_line):
    """データ行からRPM値を抽出する"""
    try:
        # データがカンマ区切りの場合、RPMは2番目（index 1）と仮定
        parts = data_line.split(',')
        if len(parts) >= 3:
            rpm_value = float(parts[1].strip())
            return rpm_value
    except (ValueError, IndexError):
        pass
    return None

def log_sensor_data():
    # 最初に利用可能なポートを確認
    print("シリアルポートの接続を確認中...")
    
    # 設定されたポートをテスト
    if test_port_connection(SERIAL_PORT, BAUD_RATE):
        selected_port = SERIAL_PORT
        print(f"設定されたポート {SERIAL_PORT} が利用可能です。")
    else:
        print(f"設定されたポート {SERIAL_PORT} が利用できません。")
        selected_port = list_available_ports()
        
        if selected_port is None:
            print("有効なポートが選択されませんでした。プログラムを終了します。")
            return
        
        # 選択されたポートをテスト
        if not test_port_connection(selected_port, BAUD_RATE):
            print(f"選択されたポート {selected_port} への接続に失敗しました。")
            return
    
    try:
        # logフォルダを作成（存在しない場合）
        import os
        log_dir = "log"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            print(f"ログフォルダ '{log_dir}' を作成しました。")
        
        # タイムスタンプ付きのログファイル名を生成
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        LOG_FILE_NAME = os.path.join(log_dir, f"{LOG_FILE_BASE}_{timestamp}.csv")
        print(f"ログファイル名: {LOG_FILE_NAME}")
        
        # シリアルポートを開く
        ser = serial.Serial(selected_port, BAUD_RATE, timeout=1)
        print(f"{selected_port} に接続しました。")
        time.sleep(2) # シリアルポート接続後、安定するまで待つ

        # ログファイルを開く (書き込みモード 'w')
        with open(LOG_FILE_NAME, 'w', newline='', encoding='utf-8') as f:
            print(f"ログを {LOG_FILE_NAME} に書き込み中...")
            print("Ctrl+C を押すと停止します。")
            
            # CSVヘッダーを書き込む（新しいファイルなので常に書き込み）
            # f.write("timestamp_us_from_start, current, angle, rpm, amp, temp\n")
            f.write("timestamp_us_from_start, current, rpm\n")
            print("CSVヘッダーを書き込みました。")

            # ログ開始時刻を記録（基準時）
            log_start_time = time.time()
            print(f"基準時刻: {datetime.datetime.fromtimestamp(log_start_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            command_sent = False
            stop_requested = False
            stop_command_time = None
            lines_received = 0  # 受信した行数をカウント
            rpm_stable_count = 0  # RPMが安定値を連続で取った回数
            print("ログ記録を開始しました。0.5秒後に動作開始コマンドを送信します...")
            print("最初の3行は破棄し、4行目からログに記録します。")
            print(f"Ctrl+Cを押した後、RPMが{RPM_STABLE_THRESHOLD}以下を{RPM_STABLE_COUNT}回連続で記録したら自動停止します。")

            try:
                while True:
                    try:
                        # ログ開始から0.5秒後に動作開始コマンドを送信
                        if not command_sent and (time.time() - log_start_time) >= 0.5:
                            print(f"Arduinoに動作開始コマンド '{START_COMMAND}' を送信...")
                            ser.write((START_COMMAND + '\n').encode('utf-8'))
                            ser.flush()  # 送信バッファを確実にフラッシュ
                            print("動作開始コマンドを送信しました。")
                            command_sent = True

                        # 停止コマンド送信後、指定時間経過したらログ終了
                        if stop_requested and stop_command_time and (time.time() - stop_command_time) >= STOP_LOG_DURATION:
                            print(f"\n停止コマンド送信から{STOP_LOG_DURATION}秒経過しました。ログ記録を終了します。")
                            return

                        # 停止コマンド送信後、RPMが安定したら自動停止
                        if stop_requested and rpm_stable_count >= RPM_STABLE_COUNT:
                            print(f"\n停止後にRPMが{RPM_STABLE_COUNT}回連続で安定しました。ログ記録を終了します。")
                            return

                        if ser.in_waiting > 0:
                            # 受信時刻を高精度で記録（マイクロ秒精度）
                            current_time = time.time()
                            
                            # シリアルデータを1行読み取る (改行コードまで)
                            line = ser.readline().decode('utf-8').strip()
                            lines_received += 1
                            
                            # 最初の3行は破棄
                            if lines_received <= 3:
                                print(f"[破棄 {lines_received}/3] {line}")
                                continue
                            
                            # RPM値を抽出して安定性をチェック（停止後のみ）
                            if stop_requested:
                                rpm_value = extract_rpm_value(line)
                                if rpm_value is not None:
                                    if rpm_value <= RPM_STABLE_THRESHOLD:
                                        rpm_stable_count += 1
                                        print(f"停止後RPM安定: {rpm_value} ({rpm_stable_count}/{RPM_STABLE_COUNT})")
                                    else:
                                        rpm_stable_count = 0  # 安定していない場合はカウントリセット
                            
                            # タイムスタンプを基準時からの差分（マイクロ秒）で生成
                            timestamp_diff_us = int((current_time - log_start_time) * 1000000)
                            
                            # CSV形式でデータを記録（引用符なし）
                            csv_line = f"{timestamp_diff_us},{line}"
                            
                            # 表示用の差分時間（秒）
                            time_diff_seconds = current_time - log_start_time
                            display_line = f"[+{time_diff_seconds:.3f}s] {line}"
                            
                            # print(display_line) # コンソールにも表示
                            f.write(csv_line + '\n') # ファイルに書き込み、改行
                            f.flush() # ファイルに即座に書き出す (停電対策など)

                        time.sleep(LOG_INTERVAL_SEC) # 設定した間隔で待機

                    except KeyboardInterrupt:
                        if not stop_requested:
                            print(f"\n\nCtrl+C が押されました。動作停止コマンド '{STOP_COMMAND}' を送信...")
                            ser.write((STOP_COMMAND + '\n').encode('utf-8'))
                            ser.flush()
                            stop_command_time = time.time()
                            stop_requested = True
                            rpm_stable_count = 0  # RPMカウントをリセット
                            print(f"動作停止コマンドを送信しました。")
                            print(f"RPMが{RPM_STABLE_THRESHOLD}以下を{RPM_STABLE_COUNT}回連続で記録するか、{STOP_LOG_DURATION}秒経過後に終了します。")
                            print("再度 Ctrl+C を押すと強制終了します。")
                        else:
                            print("\n\n強制終了が要求されました。")
                            return

            except Exception as e:
                print(f"予期せぬエラー: {e}")

    except serial.SerialException as e:
        print(f"エラー: シリアルポートに接続できませんでした。({e})")
        print(f"使用を試みたポート: {selected_port}")
        print("以下の確認をしてください:")
        print("1. Arduinoが正しく接続されているか")
        print("2. 他のアプリケーション（Arduino IDE、シリアルモニターなど）がポートを使用していないか")
        print("3. 管理者権限でプログラムを実行してみる")
    except KeyboardInterrupt:
        print("\nログ記録を停止しました。")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("シリアルポートを閉じました。")

if __name__ == "__main__":
    log_sensor_data()