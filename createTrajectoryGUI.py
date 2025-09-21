import sys
import re
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
import io
from contextlib import redirect_stdout

# ==============================================================================
# データ処理関数
# ==============================================================================

def parse_waypoints(file_path):
    """
    指定されたパスのファイルを解析し、IDをキー、ウェイポイントデータを値とする辞書を返す。
    """
    waypoints = {}
    counters = {'1行目': 1, '2行目': 1, '3行目': 1, '4行目': 1, '上段': 1, '下段': 1, '外': 1}
    section_map = {
        '1行目': '1', '2行目': '2', '3行目': '3', '4行目': '4',
        '上段': 'h', '下段': 'l', '外': 'o'
    }
    current_section_key = None
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line: continue
                found_section = False
                for key in section_map.keys():
                    if key in line:
                        current_section_key = key
                        found_section = True
                        break
                if found_section: continue
                if 'trajectory_waypoint_t' in line and current_section_key:
                    prefix = section_map[current_section_key]
                    count = counters[current_section_key]
                    waypoint_id = f"{prefix}-{count}"
                    data = line.rstrip(',')
                    waypoints[waypoint_id] = data
                    counters[current_section_key] += 1
    except FileNotFoundError:
        messagebox.showerror("エラー", f"ファイル '{file_path}' が見つかりません。")
        return None
    except Exception as e:
        messagebox.showerror("エラー", f"ファイルの解析中にエラーが発生しました: {e}")
        return None
    return waypoints

def display_and_modify_waypoints(waypoints_dict, order_list):
    """
    指定されたルールに従ってウェイポイント情報を加工し、結果を出力する。
    浮動小数点数の末尾には 'f' を付与する。
    """
    collapse_point_group = {
        # LEFT SHOOTING AREA
        # '1-6', '1-7', '1-8', '1-9', '1-10', '2-7', '2-8', '2-9', '2-10',
        # '3-9', '3-10', '4-9', '4-10'
        
        # RIGHT SHOOTING AREA
		'1-1', '1-2', '1-3', '1-4', '1-5', '2-1', '2-2', '2-3', '2-4', 
		'3-1', '3-2', '4-1', '4-2'
    }

    for i, current_id in enumerate(order_list):
        if current_id not in waypoints_dict:
            print(f"# 警告: ID '{current_id}' はデータ内に見つかりませんでした。")
            continue

        original_data = waypoints_dict[current_id]
        match = re.search(r'\((.*)\)', original_data)
        if not match: continue
        
        params = [p.strip() for p in match.group(1).split(',')]
        
        # 1番目と2番目の数字の頭に 'R_offset', 'P_offset' を付与
        if len(params) > 0:
            params[0] = 'R_offset + ' + re.sub(r'(-?\d+\.\d+)', r'\1f', params[0])
        if len(params) > 1:
            params[1] = 'P_offset + ' + re.sub(r'(-?\d+\.\d+)', r'\1f', params[1])

        if current_id.startswith('h') or current_id.startswith('o'):
            params[3] = 'Hand::LiftAngle::SHOOT_UP'
        elif current_id.startswith('l'):
            params[3] = 'Hand::LiftAngle::SHOOT_LOW'
        else:
            params[3] = re.sub(r'^(-?)0+(\d+)$', r'\1\2', params[3])
            try:
                val = int(params[3])
                if val <= 3000:
                    params[3] = str(val + 4096)
            except ValueError:
                pass

        sixth_param = ''
        if i == 0:
            sixth_param = 'Traj::PassThroughMode::DIRECT'
        else:
            previous_id = order_list[i - 1]
            if previous_id.startswith('h') or previous_id.startswith('o'):
                sixth_param = 'Traj::PassThroughMode::INTERMEDIATE_2' if current_id in collapse_point_group else 'Traj::PassThroughMode::INTERMEDIATE_1'
            elif previous_id.startswith('l'):
                sixth_param = 'Traj::PassThroughMode::INTERMEDIATE_12' if current_id in collapse_point_group else 'Traj::PassThroughMode::INTERMEDIATE_1'
            elif previous_id[0].isdigit():
                if current_id.startswith('h') or current_id.startswith('o'):
                    sixth_param = 'Traj::PassThroughMode::INTERMEDIATE_1'
                elif current_id.startswith('l'):
                    x = int(current_id.split('-')[1])
                    quotient = ((x - 1) // 5) + 1
                    sixth_param = f'Traj::PassThroughMode::INTERMEDIATE_1U{quotient}'
        
        params.append(sixth_param)

        # 3番目以降の浮動小数点数に 'f' を付与（1,2番目はすでに付与済み）
        formatted_params = []
        for idx, p in enumerate(params):
            if idx in [0, 1]:
                formatted_params.append(p)
            else:
                formatted_p = re.sub(r'(-?\d+\.\d+)', r'\1f', p)
                formatted_params.append(formatted_p)
        
        new_params_str = ", ".join(formatted_params)
        new_data_string = f"trajectory_waypoint_t({new_params_str})"
        print(f"{new_data_string}, // ID: {current_id}")
        
# ==============================================================================
# GUIアプリケーション部分
# ==============================================================================

class WaypointSelectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Waypoint 軌道順序設定ツール")
        self.root.geometry("800x600")

        self.all_waypoints = {}
        self.drag_start_index = None
        
        main_frame = tk.Frame(root, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        file_frame = tk.Frame(main_frame)
        file_frame.pack(fill=tk.X, pady=5)
        tk.Button(file_frame, text="座標ファイルを開く", command=self.load_file).pack(side=tk.LEFT)
        self.file_label = tk.Label(file_frame, text="ファイルが選択されていません")
        self.file_label.pack(side=tk.LEFT, padx=10)

        lists_parent_frame = tk.Frame(main_frame)
        lists_parent_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        numeric_frame = tk.Frame(lists_parent_frame)
        numeric_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        tk.Label(numeric_frame, text="奇数番目 (数字ID)").pack()
        self.numeric_listbox = tk.Listbox(numeric_frame, selectmode=tk.SINGLE)
        self.numeric_listbox.pack(fill=tk.BOTH, expand=True)

        alpha_frame = tk.Frame(lists_parent_frame)
        alpha_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        tk.Label(alpha_frame, text="偶数番目 (アルファベットID)").pack()
        self.alpha_listbox = tk.Listbox(alpha_frame, selectmode=tk.SINGLE)
        self.alpha_listbox.pack(fill=tk.BOTH, expand=True)

        for listbox in [self.numeric_listbox, self.alpha_listbox]:
            listbox.bind("<Button-1>", self.on_drag_start)
            listbox.bind("<B1-Motion>", self.on_drag_motion)
            listbox.bind("<ButtonRelease-1>", self.on_drag_release)

        bottom_frame = tk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X, pady=10)
        io_frame = tk.Frame(bottom_frame)
        io_frame.pack(side=tk.LEFT)
        tk.Button(io_frame, text="順序を保存", command=self.save_sequence).pack(side=tk.LEFT, padx=5)
        tk.Button(io_frame, text="順序を読み込み", command=self.load_sequence).pack(side=tk.LEFT, padx=5)
        tk.Button(bottom_frame, text="軌道生成と保存", command=self.generate_trajectory).pack(side=tk.RIGHT)

    def on_drag_start(self, event):
        index = event.widget.nearest(event.y)
        if index != -1: self.drag_start_index = index

    def on_drag_motion(self, event):
        if self.drag_start_index is None: return
        widget = event.widget
        current_index = widget.nearest(event.y)
        if current_index != self.drag_start_index:
            item = widget.get(self.drag_start_index)
            widget.delete(self.drag_start_index)
            widget.insert(current_index, item)
            self.drag_start_index = current_index

    def on_drag_release(self, event):
        self.drag_start_index = None

    def load_file(self):
        file_path = filedialog.askopenfilename(title="座標ファイルを選択", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not file_path: return
        self.all_waypoints = parse_waypoints(file_path)
        if self.all_waypoints:
            self.file_label.config(text=file_path.split('/')[-1])
            self.numeric_listbox.delete(0, tk.END)
            self.alpha_listbox.delete(0, tk.END)
            for waypoint_id in sorted(self.all_waypoints.keys()):
                if waypoint_id[0].isdigit():
                    self.numeric_listbox.insert(tk.END, waypoint_id)
                else:
                    self.alpha_listbox.insert(tk.END, waypoint_id)

    def save_sequence(self):
        numeric_ids = self.numeric_listbox.get(0, tk.END)
        alpha_ids = self.alpha_listbox.get(0, tk.END)
        if not numeric_ids and not alpha_ids:
            messagebox.showwarning("警告", "保存する順序がありません。")
            return
        file_path = filedialog.asksaveasfilename(title="順序を保存", defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not file_path: return
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write("#NUMERIC_IDS\n")
                for item_id in numeric_ids: f.write(f"{item_id}\n")
                f.write("#ALPHA_IDS\n")
                for item_id in alpha_ids: f.write(f"{item_id}\n")
            messagebox.showinfo("成功", f"順序を '{file_path}' に保存しました。")
        except Exception as e:
            messagebox.showerror("エラー", f"ファイルの保存中にエラーが発生しました: {e}")

    def load_sequence(self):
        if not self.all_waypoints:
            messagebox.showerror("エラー", "先に座標ファイルを読み込んでください。")
            return
        file_path = filedialog.askopenfilename(title="順序ファイルを読み込み", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not file_path: return
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f if line.strip()]
            self.numeric_listbox.delete(0, tk.END)
            self.alpha_listbox.delete(0, tk.END)
            current_list_target = None
            for line in lines:
                if line == '#NUMERIC_IDS':
                    current_list_target = self.numeric_listbox
                    continue
                elif line == '#ALPHA_IDS':
                    current_list_target = self.alpha_listbox
                    continue
                if current_list_target and line in self.all_waypoints:
                    current_list_target.insert(tk.END, line)
        except Exception as e:
            messagebox.showerror("エラー", f"ファイルの読み込み中にエラーが発生しました: {e}")

    def generate_trajectory(self):
        numeric_ids = self.numeric_listbox.get(0, tk.END)
        alpha_ids = self.alpha_listbox.get(0, tk.END)

        if not self.all_waypoints:
            messagebox.showerror("エラー", "座標ファイルが読み込まれていません。")
            return
        if not numeric_ids and not alpha_ids:
            messagebox.showerror("エラー", "軌道の順序が設定されていません。")
            return
        
        final_order_list = []
        len_num = len(numeric_ids)
        len_alpha = len(alpha_ids)
        max_len = max(len_num, len_alpha)

        for i in range(max_len):
            if i < len_num:
                final_order_list.append(numeric_ids[i])
            if i < len_alpha:
                final_order_list.append(alpha_ids[i])
        
        output_capture = io.StringIO()
        with redirect_stdout(output_capture):
            display_and_modify_waypoints(self.all_waypoints, final_order_list)
        result_text = output_capture.getvalue()
        
        if result_text.strip():
            save_path = filedialog.asksaveasfilename(
                title="生成した軌道を保存",
                defaultextension=".txt",
                filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
            )
            if save_path:
                try:
                    with open(save_path, 'w', encoding='utf-8') as f:
                        f.write(result_text)
                    messagebox.showinfo("保存成功", f"軌道データをファイルに保存しました。\n{save_path}")
                except Exception as e:
                    messagebox.showerror("保存エラー", f"ファイルの保存中にエラーが発生しました:\n{e}")

        self.show_results(result_text)
        
    def show_results(self, text):
        result_window = tk.Toplevel(self.root)
        result_window.title("生成結果")
        result_window.geometry("800x600")
        text_area = scrolledtext.ScrolledText(result_window, wrap=tk.WORD)
        text_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        text_area.insert(tk.INSERT, text)
        text_area.config(state=tk.DISABLED)


if __name__ == "__main__":
    root = tk.Tk()
    app = WaypointSelectorApp(root)
    root.mainloop()