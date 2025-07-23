import serial
import csv
from datetime import datetime

# シリアル設定（適宜変更）
PORT = 'COM6'        # 実際のポート名に変更（例: '/dev/ttyACM0'）
BAUDRATE = 115200

# 出力ファイル名
filename = f"data_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# シリアルポートを開く
ser = serial.Serial(PORT, BAUDRATE)

with open(filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # 末尾のカンマ対策
                values = [v.strip() for v in line.split(',') if v.strip()]
                
                # 4個ずつ分割して行にする
                for i in range(0, len(values), 2):
                    group = values[i:i+2]
                    if len(group) == 2:
                        writer.writerow(group)
                        # print(group)
                    else:
                        print(f"Warning: incomplete group: {group}")
    except KeyboardInterrupt:
        print("Logging stopped.")
