#!/usr/bin/env python3
"""GUI serial configuration tool for STM32U073 inclinometer V3.3."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Dict, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "Missing dependency",
        "缺少 pyserial。请先运行：\n\npython -m pip install pyserial",
    )
    raise SystemExit(2) from exc


APP_TITLE = "V3.3 参数配置工具"
DEFAULT_BAUD = "115200"

CONFIG_KEYS = [
    ("initial_delay_sec", "上电首发等待(s)"),
    ("interval_sec", "正常发送间隔(s)"),
    ("burst_ms", "突发持续时间(ms)"),
    ("burst_period_ms", "突发帧间隔(ms)"),
    ("lte_wait_ms", "4G上电等待(ms)"),
    ("fast_count", "上电首发帧数"),
    ("fast_gap_ms", "首发帧间隔(ms)"),
    ("drain_ms", "4G收尾等待(ms)"),
    ("connect_timeout_ms", "4G连接超时(ms)"),
    ("mqtt_host", "MQTT服务器"),
    ("mqtt_port", "MQTT端口"),
    ("mqtt_user", "MQTT用户名"),
    ("mqtt_pass", "MQTT密码"),
    ("mqtt_client_id", "MQTT ClientID"),
    ("device_id", "设备ID"),
    ("mqtt_topic", "MQTT发布话题"),
]


class SerialConfigApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1060x720")
        self.minsize(900, 620)

        self.ser: Optional[serial.Serial] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.stop_rx = threading.Event()
        self.rx_queue: queue.Queue[bytes] = queue.Queue()
        self.rx_text_tail = ""
        self.config_vars: Dict[str, tk.StringVar] = {}
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.status_var = tk.StringVar(value="未打开串口")
        self.command_var = tk.StringVar()
        self.auto_exit_var = tk.BooleanVar(value=False)

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._poll_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        port_bar = ttk.Frame(root)
        port_bar.pack(fill=tk.X)

        ttk.Label(port_bar, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(port_bar, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Button(port_bar, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(port_bar, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(port_bar, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(6, 10))

        self.open_button = ttk.Button(port_bar, text="打开串口", command=self.toggle_port)
        self.open_button.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Checkbutton(port_bar, text="保存后自动EXIT", variable=self.auto_exit_var).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(port_bar, textvariable=self.status_var, foreground="#1f5fbf").pack(side=tk.LEFT, padx=(10, 0))

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 8))

        log_frame = ttk.LabelFrame(body, text="串口收发")
        body.add(log_frame, weight=3)

        self.log_text = tk.Text(log_frame, height=22, wrap=tk.WORD, font=("Consolas", 10))
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        cfg_frame = ttk.LabelFrame(body, text="参数表")
        body.add(cfg_frame, weight=2)

        canvas = tk.Canvas(cfg_frame, highlightthickness=0)
        cfg_scroll = ttk.Scrollbar(cfg_frame, orient=tk.VERTICAL, command=canvas.yview)
        form = ttk.Frame(canvas)
        form.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=form, anchor="nw")
        canvas.configure(yscrollcommand=cfg_scroll.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        cfg_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        for row, (key, label) in enumerate(CONFIG_KEYS):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", padx=6, pady=4)
            ttk.Label(form, text=key, foreground="#666666").grid(row=row, column=1, sticky="w", padx=6, pady=4)
            var = tk.StringVar()
            self.config_vars[key] = var
            ttk.Entry(form, textvariable=var, width=34).grid(row=row, column=2, sticky="ew", padx=6, pady=4)
        form.columnconfigure(2, weight=1)

        command_bar = ttk.Frame(root)
        command_bar.pack(fill=tk.X)

        ttk.Button(command_bar, text="HELP", command=lambda: self.send_command("HELP")).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="读取配置(GET)", command=self.get_config).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="写入参数(SET)", command=self.set_all_fields).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="保存(SAVE)", command=self.save_config).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="写入并保存", command=self.set_and_save).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="恢复默认(DEFAULT)", command=lambda: self.send_command("DEFAULT")).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="EXIT", command=lambda: self.send_command("EXIT")).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(command_bar, text="清空日志", command=self.clear_log).pack(side=tk.RIGHT)

        manual_bar = ttk.Frame(root)
        manual_bar.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(manual_bar, text="手工命令").pack(side=tk.LEFT)
        command_entry = ttk.Entry(manual_bar, textvariable=self.command_var)
        command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        command_entry.bind("<Return>", lambda _e: self.send_manual_command())
        ttk.Button(manual_bar, text="发送", command=self.send_manual_command).pack(side=tk.LEFT)

    def refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [f"{p.device}  {p.description}" for p in ports]
        self.port_combo["values"] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])
            self.status_var.set("请选择串口后打开")
        elif not values:
            self.port_var.set("")
            self.status_var.set("未找到串口，插入USB后点刷新")

    def selected_port(self) -> str:
        text = self.port_var.get().strip()
        return text.split()[0] if text else ""

    def toggle_port(self) -> None:
        if self.ser and self.ser.is_open:
            self.close_port()
        else:
            self.open_port()

    def open_port(self) -> None:
        port = self.selected_port()
        if not port:
            messagebox.showwarning("未选择串口", "未找到串口。请插入 USB 后点击刷新。")
            return
        try:
            baud = int(self.baud_var.get())
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=1.0,
            )
            ser.dtr = False
            ser.rts = False
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.ser = ser
            self.stop_rx.clear()
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()
            self.open_button.configure(text="关闭串口")
            self.status_var.set(f"已打开 {port} @ {baud}")
            self.append_log(f"[PC] Opened {port} @ {baud}\n")
            self.append_log("[PC] 如果板子在STOP中，请确认USB插入触发唤醒；看到 CFG READY 后即可 GET。\n")
        except Exception as exc:
            messagebox.showerror("打开串口失败", str(exc))
            self.status_var.set("打开串口失败")

    def close_port(self) -> None:
        self.stop_rx.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.open_button.configure(text="打开串口")
        self.status_var.set("串口已关闭")
        self.append_log("[PC] Port closed\n")

    def _rx_worker(self) -> None:
        while not self.stop_rx.is_set():
            ser = self.ser
            if ser is None or not ser.is_open:
                break
            try:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    self.rx_queue.put(data)
            except Exception as exc:
                self.rx_queue.put(f"\n[PC] RX error: {exc}\n".encode("utf-8", errors="replace"))
                break

    def _poll_rx_queue(self) -> None:
        while True:
            try:
                data = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            text = data.decode("utf-8", errors="replace")
            self.append_log(text)
            self._parse_config_lines(text)
        self.after(50, self._poll_rx_queue)

    def _parse_config_lines(self, text: str) -> None:
        self.rx_text_tail += text
        while "\n" in self.rx_text_tail:
            line, self.rx_text_tail = self.rx_text_tail.split("\n", 1)
            line = line.strip("\r ")
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key in self.config_vars:
                self.config_vars[key].set(value)

    def append_log(self, text: str) -> None:
        self.log_text.insert(tk.END, text)
        self.log_text.see(tk.END)

    def clear_log(self) -> None:
        self.log_text.delete("1.0", tk.END)

    def send_command(self, command: str) -> bool:
        if self.ser is None or not self.ser.is_open:
            messagebox.showwarning("串口未打开", "请先选择并打开串口。")
            return False
        command = command.strip()
        if not command:
            return False
        try:
            self.ser.write((command + "\r\n").encode("ascii"))
            self.ser.flush()
            self.append_log(f"\n[TX] {command}\n")
            return True
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))
            return False

    def send_manual_command(self) -> None:
        command = self.command_var.get().strip()
        if self.send_command(command):
            self.command_var.set("")

    def get_config(self) -> None:
        self.send_command("GET")

    def set_all_fields(self) -> None:
        for key, _label in CONFIG_KEYS:
            value = self.config_vars[key].get().strip()
            if value:
                if not self.send_command(f"SET {key}={value}"):
                    return
                self.update()
                time.sleep(0.03)

    def save_config(self) -> None:
        if self.send_command("SAVE") and self.auto_exit_var.get():
            self.after(300, lambda: self.send_command("EXIT"))

    def set_and_save(self) -> None:
        self.set_all_fields()
        self.after(300, self.save_config)

    def on_close(self) -> None:
        self.close_port()
        self.destroy()


def main() -> int:
    app = SerialConfigApp()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
