#!/usr/bin/env python3
"""
Simple ROS2 Tkinter GUI to set/get LED modes & colors for a mecanum robot.
Layout: two tables side-by-side — left = Colors, right = Modes.
Tables are arranged as:
  FL   FR
  BL   BR

Requires:
  - ROS2 (rclpy)
  - mecanumbot_msgs (SetLedStatus, GetLedStatus)
"""

import threading
import tkinter as tk
from tkinter import ttk, messagebox

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
MODE_NAME_TO_VAL = {name: val for name, val in MODE_MAP}
MODE_VAL_TO_NAME = {val: name for name, val in MODE_MAP}

# Order of corners used in UI and mapping
CORNERS = ["FL", "FR", "BL", "BR"]  # display order

# Service names
SET_SRV = "set_led_status"
GET_SRV = "get_led_status"

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
        self.geometry("560x280")
        self.minsize(520, 260)

        # ROS init (node in separate thread-safe context)
        rclpy.init()
        self.node = LedClient()

        # Top bar (service availability)
        top = ttk.Frame(self)
        top.pack(fill=tk.X, padx=10, pady=(10, 0))
        self.status_lbl = ttk.Label(top, text="Checking services…")
        self.status_lbl.pack(side=tk.LEFT)
        check_btn = ttk.Button(top, text="Re-check", command=self.check_services)
        check_btn.pack(side=tk.RIGHT)

        # Main content: two side-by-side tables
        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        color_frame = ttk.LabelFrame(main, text="Colors")
        mode_frame  = ttk.LabelFrame(main, text="Modes")
        color_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        mode_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # Headers in both tables
        for frame in (color_frame, mode_frame):
            ttk.Label(frame, text="FL", anchor="center").grid(row=0, column=1, padx=6, pady=4, sticky="ew")
            ttk.Label(frame, text="FR", anchor="center").grid(row=0, column=2, padx=6, pady=4, sticky="ew")
            ttk.Label(frame, text="BL", anchor="center").grid(row=2, column=1, padx=6, pady=4, sticky="ew")
            ttk.Label(frame, text="BR", anchor="center").grid(row=2, column=2, padx=6, pady=4, sticky="ew")

        # Dropdowns
        self.color_vars = {c: tk.StringVar(value="BLACK") for c in CORNERS}
        self.mode_vars  = {c: tk.StringVar(value="BLACK") for c in CORNERS}

        # Row 1 (FL, FR)
        ttk.Label(color_frame, text="").grid(row=1, column=0)  # spacer
        self._make_combo(color_frame, self.color_vars['FL'], [n for n,_ in COLOR_MAP]).grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        self._make_combo(color_frame, self.color_vars['FR'], [n for n,_ in COLOR_MAP]).grid(row=1, column=2, padx=6, pady=6, sticky="ew")

        ttk.Label(mode_frame, text="").grid(row=1, column=0)
        self._make_combo(mode_frame, self.mode_vars['FL'], [n for n,_ in MODE_MAP]).grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        self._make_combo(mode_frame, self.mode_vars['FR'], [n for n,_ in MODE_MAP]).grid(row=1, column=2, padx=6, pady=6, sticky="ew")

        # Row 2 (BL, BR)
        ttk.Label(color_frame, text="").grid(row=3, column=0)  # spacer
        self._make_combo(color_frame, self.color_vars['BL'], [n for n,_ in COLOR_MAP]).grid(row=3, column=1, padx=6, pady=6, sticky="ew")
        self._make_combo(color_frame, self.color_vars['BR'], [n for n,_ in COLOR_MAP]).grid(row=3, column=2, padx=6, pady=6, sticky="ew")

        ttk.Label(mode_frame, text="").grid(row=3, column=0)
        self._make_combo(mode_frame, self.mode_vars['BL'], [n for n,_ in MODE_MAP]).grid(row=3, column=1, padx=6, pady=6, sticky="ew")
        self._make_combo(mode_frame, self.mode_vars['BR'], [n for n,_ in MODE_MAP]).grid(row=3, column=2, padx=6, pady=6, sticky="ew")

        # Action buttons
        btns = ttk.Frame(self)
        btns.pack(fill=tk.X, padx=10, pady=(0, 10))
        ttk.Button(btns, text="Get from Robot", command=self.on_get).pack(side=tk.LEFT)
        ttk.Button(btns, text="Send to Robot", command=self.on_set).pack(side=tk.RIGHT)

        self.check_services()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _make_combo(self, parent, var, values):
        cb = ttk.Combobox(parent, textvariable=var, values=values, state="readonly")
        return cb

    def check_services(self):
        def work():
            ok = self.node.wait_for_services(timeout_sec=1.5)
            self.after(0, lambda: self.status_lbl.config(
                text=("Services: OK" if ok else f"Services not available: '{SET_SRV}', '{GET_SRV}'")
            ))
        threading.Thread(target=work, daemon=True).start()

    def collect_ui_values(self):
        # Map UI strings to numeric values
        vals = {}
        for c in CORNERS:
            c_name = self.color_vars[c].get()
            m_name = self.mode_vars[c].get()
            vals[c] = {
                'color': COLOR_NAME_TO_VAL.get(c_name, 0),
                'mode':  MODE_NAME_TO_VAL.get(m_name, 0)
            }
        return vals

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
                # Update UI from response
                mapping = {
                    'FL': {'mode': res.fl_mode, 'color': res.fl_color},
                    'FR': {'mode': res.fr_mode, 'color': res.fr_color},
                    'BR': {'mode': res.br_mode, 'color': res.br_color},
                    'BL': {'mode': res.bl_mode, 'color': res.bl_color},
                }
                def update_ui():
                    for c in CORNERS:
                        self.color_vars[c].set(COLOR_VAL_TO_NAME.get(mapping[c]['color'], 'BLACK'))
                        self.mode_vars[c].set(MODE_VAL_TO_NAME.get(mapping[c]['mode'], 'BLACK'))
                self.after(0, update_ui)
            except Exception as e:
                self.after(0, lambda: messagebox.showerror("GET", f"Exception: {e}"))
        threading.Thread(target=work, daemon=True).start()

    def on_close(self):
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        self.destroy()

if __name__ == '__main__':
    App().mainloop()
