#!/usr/bin/env python3
"""
ROS2 Tkinter GUI to set/get LED modes & colors for a mecanum robot.

Layout: single table with rows for corners (FL, FR, BL, BR):
Corner | Color | Mode

"""

import json
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox, simpledialog

import rclpy
from rclpy.node import Node

from mecanumbot_msgs.srv import GetLedStatus, SetLedStatus

# ---------------------------- Configurable enums ---------------------------- #
COLOR_MAP = [
    ("BLACK", 0),
    ("WHITE", 1),
    ("GREEN", 2),
    ("RED", 3),
    ("BLUE", 4),
    ("CYAN", 5),
    ("PINK", 6),
    ("YELLOW", 7),
]

MODE_MAP = [
    ("WAVE_RIGHT", 1),
    ("WAVE_LEFT", 2),
    ("PULSE", 3),
    ("SOLID", 4),
]

COLOR_NAME_TO_VAL = {name: val for name, val in COLOR_MAP}
COLOR_VAL_TO_NAME = {val: name for name, val in COLOR_MAP}
MODE_NAME_TO_VAL  = {name: val for name, val in MODE_MAP}
MODE_VAL_TO_NAME  = {val: name for name, val in MODE_MAP}

CORNERS = ["FL", "FR", "BL", "BR"]

SET_SRV = "set_led_status"
GET_SRV = "get_led_status"

CONFIG_PATH = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent /"mecanumbot_ledgui" / "configs" / "led_configs.json"


# ---------------------------- ROS2 Client Node ----------------------------- #
class LedClient(Node):
    def __init__(self):
        super().__init__('mecanumbot_ledgui_client')
        self.cli_set = self.create_client(SetLedStatus, SET_SRV)
        self.cli_get = self.create_client(GetLedStatus, GET_SRV)

    def wait_for_services(self, timeout_sec: float = 5.0) -> bool:
        ok = self.cli_set.wait_for_service(timeout_sec=timeout_sec)
        ok = ok and self.cli_get.wait_for_service(timeout_sec=timeout_sec)
        return ok

    def call_set(self, values):
        """values is a dict like { 'FL': {'mode': int, 'color': int}, ... }"""
        req = SetLedStatus.Request()
        # Service expects order: fl, fr, br, bl
        req.fl_mode  = int(values['FL']['mode'])
        req.fl_color = int(values['FL']['color'])
        req.fr_mode  = int(values['FR']['mode'])
        req.fr_color = int(values['FR']['color'])
        req.br_mode  = int(values['BR']['mode'])
        req.br_color = int(values['BR']['color'])
        req.bl_mode  = int(values['BL']['mode'])
        req.bl_color = int(values['BL']['color'])
        future = self.cli_set.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_get(self):
        req = GetLedStatus.Request()
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# ------------------------------ Tkinter GUI -------------------------------- #
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mecanum LED Controller")
        self.geometry("780x320")
        self.minsize(720, 300)
        self.update_idletasks()
        self.geometry("")   # fit window to content

        # ROS init (node in separate thread-safe context)
        rclpy.init()
        self.node = LedClient()

        # ---- Top bar ----
        top = ttk.Frame(self)
        top.pack(fill=tk.X, padx=10, pady=(10, 0))
        self.status_lbl = ttk.Label(top, text="Checking services…")
        self.status_lbl.pack(side=tk.LEFT)
        ttk.Button(top, text="Re-check", command=self.check_services).pack(side=tk.RIGHT)

        # ---- Main area ----
        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left: table for current config
        table_frame = ttk.LabelFrame(main, text="Current Configuration")
        table_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        # Right: saved configurations list + actions
        saved_frame = ttk.LabelFrame(main, text="Saved Configurations")
        saved_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=(8, 0))

        # Table headers
        ttk.Label(table_frame, text="Corner", anchor="center").grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        ttk.Label(table_frame, text="Color",  anchor="center").grid(row=0, column=1, padx=6, pady=6, sticky="ew")
        ttk.Label(table_frame, text="Mode",   anchor="center").grid(row=0, column=2, padx=6, pady=6, sticky="ew")

        # Dropdown variables
        self.color_vars = {c: tk.StringVar(value=COLOR_MAP[0][0]) for c in CORNERS}
        self.mode_vars  = {c: tk.StringVar(value=MODE_MAP[0][0])  for c in CORNERS}

        # Rows for corners
        for r, corner in enumerate(CORNERS, start=1):
            ttk.Label(table_frame, text=corner, anchor="w").grid(row=r, column=0, padx=6, pady=4, sticky="w")
            self._make_combo(table_frame, self.color_vars[corner], [n for n, _ in COLOR_MAP])\
                .grid(row=r, column=1, padx=6, pady=4, sticky="ew")
            self._make_combo(table_frame, self.mode_vars[corner], [n for n, _ in MODE_MAP])\
                .grid(row=r, column=2, padx=6, pady=4, sticky="ew")

        table_frame.grid_columnconfigure(1, weight=1)
        table_frame.grid_columnconfigure(2, weight=1)

        # Saved configs widgets
        self.saved_list = tk.Listbox(saved_frame, height=12)
        self.saved_list.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 4))
        self.saved_list.bind("<<ListboxSelect>>", self.on_saved_clicked)  # load on click

        saved_btns = ttk.Frame(saved_frame)
        saved_btns.pack(fill=tk.X, padx=8, pady=(0, 8))
        ttk.Button(saved_btns, text="Save", command=self.on_save).pack(side=tk.LEFT)
        ttk.Button(saved_btns, text="Delete", command=self.on_delete_saved).pack(side=tk.RIGHT)

        # ---- Bottom actions ----
        btns = ttk.Frame(self)
        btns.pack(fill=tk.X, padx=10, pady=(0, 10))
        ttk.Button(btns, text="Get from Robot", command=self.on_get).pack(side=tk.LEFT)
        ttk.Button(btns, text="Send to Robot", command=self.on_set).pack(side=tk.RIGHT)

        # Load saved configurations from disk
        self.saved_configs = self.load_saved_configs()
        self.refresh_saved_list()

        self.check_services()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.on_get()

    # ------------------------- UI helpers & persistence ------------------------- #
    def _make_combo(self, parent, var, values):
        return ttk.Combobox(parent, textvariable=var, values=values, state="readonly")

    def check_services(self):
        def work():
            ok = self.node.wait_for_services(timeout_sec=1.5)
            self.after(0, lambda: self.status_lbl.config(
                text=("Services: OK" if ok else f"Services not available: '{SET_SRV}', '{GET_SRV}'")
            ))
        threading.Thread(target=work, daemon=True).start()

    def collect_ui_values(self):
        vals = {}
        for c in CORNERS:
            c_name = self.color_vars[c].get()
            m_name = self.mode_vars[c].get()
            vals[c] = {
                'color': COLOR_NAME_TO_VAL.get(c_name, 0),
                'mode':  MODE_NAME_TO_VAL.get(m_name, 0),
            }
        return vals

    def apply_values_to_ui(self, mapping):
        for c in CORNERS:
            self.color_vars[c].set(COLOR_VAL_TO_NAME.get(mapping[c]['color'], COLOR_MAP[0][0]))
            self.mode_vars[c].set(MODE_VAL_TO_NAME.get(mapping[c]['mode'], MODE_MAP[0][0]))

    def load_saved_configs(self):
        try:
            if CONFIG_PATH.exists():
                with CONFIG_PATH.open("r", encoding="utf-8") as f:
                    data = json.load(f)
                    # Expected shape: { name: { 'FL': {'mode': int,'color': int}, ... } }
                    return data if isinstance(data, dict) else {}
        except Exception as e:
            messagebox.showwarning("Configs", f"Failed to read saved configs:\n{e}")
        return {}

    def save_configs_to_disk(self):
        try:
            with CONFIG_PATH.open("w", encoding="utf-8") as f:
                json.dump(self.saved_configs, f, indent=2)
        except Exception as e:
            messagebox.showerror("Configs", f"Failed to save configs:\n{e}")

    def refresh_saved_list(self):
        self.saved_list.delete(0, tk.END)
        for name in sorted(self.saved_configs.keys(), key=str.lower):
            self.saved_list.insert(tk.END, name)

    # ------------------------------ Button handlers ---------------------------- #
    def on_set(self):
        vals = self.collect_ui_values()
        def work():
            try:
                res = self.node.call_set(vals)
                if res and getattr(res, 'success', True):
                    msg = getattr(res, 'message', 'OK')
                    self.after(0, lambda: messagebox.showinfo("SET", f"Success: {msg}"))
                else:
                    msg = getattr(res, 'message', 'Unknown error') if res else 'No response'
                    self.after(0, lambda: messagebox.showerror("SET", f"Failed: {msg}"))
            except Exception as e:
                self.after(0, lambda: messagebox.showerror("SET", f"Exception: {e}"))
        threading.Thread(target=work, daemon=True).start()

    def on_get(self):
        def work():
            try:
                res = self.node.call_get()
                if not res:
                    self.after(0, lambda: messagebox.showerror("GET", "No response"))
                    return
                mapping = {
                    'FL': {'mode': res.fl_mode, 'color': res.fl_color},
                    'FR': {'mode': res.fr_mode, 'color': res.fr_color},
                    'BR': {'mode': res.br_mode, 'color': res.br_color},
                    'BL': {'mode': res.bl_mode, 'color': res.bl_color},
                }
                self.after(0, lambda: self.apply_values_to_ui(mapping))
            except Exception as e:
                self.after(0, lambda: messagebox.showerror("GET", f"Exception: {e}"))
        threading.Thread(target=work, daemon=True).start()

    def on_save(self):
        name = simpledialog.askstring("Save Configuration", "Name this configuration:")
        if not name:
            return
        name = name.strip()
        if not name:
            return
        if name in self.saved_configs:
            if not messagebox.askyesno("Overwrite?", f"'{name}' already exists. Overwrite it?"):
                return
        self.saved_configs[name] = self.collect_ui_values()
        self.save_configs_to_disk()
        self.refresh_saved_list()
        # Optionally select the saved item
        names = list(sorted(self.saved_configs.keys(), key=str.lower))
        try:
            idx = names.index(name)
            self.saved_list.selection_clear(0, tk.END)
            self.saved_list.selection_set(idx)
        except ValueError:
            pass

    def on_delete_saved(self):
        sel = self.saved_list.curselection()
        if not sel:
            return
        name = self.saved_list.get(sel[0])
        if not messagebox.askyesno("Delete", f"Delete saved configuration '{name}'?"):
            return
        self.saved_configs.pop(name, None)
        self.save_configs_to_disk()
        self.refresh_saved_list()

    def on_saved_clicked(self, _event=None):
        sel = self.saved_list.curselection()
        if not sel:
            return
        name = self.saved_list.get(sel[0])
        mapping = self.saved_configs.get(name)
        if mapping:
            self.apply_values_to_ui(mapping)

    def on_close(self):
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        self.destroy()

def main():
    App().mainloop()

if __name__ == '__main__':
    main()
