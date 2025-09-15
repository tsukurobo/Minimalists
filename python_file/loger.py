# save_serial_to_csv.py
import re
import csv
import sys
import argparse
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial が必要です: pip install pyserial", file=sys.stderr)
    sys.exit(1)

LINE_RE = re.compile(
    r'^\s*([+-]?\d+(?:\.\d+)?)\s*s,\s*([+-]?\d+(?:\.\d+)?)\s*rad,\s*([+-]?\d+(?:\.\d+)?)\s*Nm'
)

def open_serial(port: str, baud: int, timeout: float = 1.0):
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)

def ensure_header(path: Path, fieldnames):
    write_header = (not path.exists()) or path.stat().st_size == 0
    f = path.open("a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if write_header:
        writer.writerow(fieldnames)
    return f, writer

def main():
    ap = argparse.ArgumentParser(
        description="Read lines like '0.12345 s, 1.234 rad, 0.567 Nm' from serial and save to CSV."
    )
    ap.add_argument("--port", required=True, help="Serial port (e.g., COM5 or /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("--outfile", default="log.csv", help="CSV output path (default: log.csv)")
    ap.add_argument("--flush-every", type=int, default=25, help="Flush to disk every N rows (default: 25)")
    args = ap.parse_args()

    out_path = Path(args.outfile)
    fieldnames = ["time_s", "position_rad", "torque_Nm"]
    f, writer = ensure_header(out_path, fieldnames)

    try:
        with open_serial(args.port, args.baud) as ser:
            print(f"Logging from {args.port} @ {args.baud} to {out_path.resolve()} (Ctrl+C で終了)")
            count_since_flush = 0
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    continue
                m = LINE_RE.match(line)
                if not m:
                    # 必要なら生データ確認用に次行のコメントを外す
                    # print(f"skip: {line}")
                    continue
                t, pos, tq = map(float, m.groups())
                writer.writerow([f"{t:.5f}", f"{pos:.3f}", f"{tq:.3f}"])
                count_since_flush += 1
                if count_since_flush >= args.flush_every:
                    f.flush()
                    count_since_flush = 0
    except KeyboardInterrupt:
        print("\n停止しました。ファイルを閉じます。")
    finally:
        try:
            f.flush()
            f.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
