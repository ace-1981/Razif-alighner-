"""
Razif Aligner Dashboard
=======================
Two modes:
  1. SERIAL  – reads live telemetry from a real Arduino via COM port
  2. SIMULATION – offline physics simulation (no hardware needed)

Arduino telemetry format (each ~100 ms):
  T seek=1 cen=0 limL=0 limR=0 rawD2=1 rawD3=1 rawLL=0 rawLR=0 aV=2.345 dir=1 pwm=90 hs=0 seekDir=1
"""

import math
import random
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports as list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ───────────────────── constants ─────────────────────
HOMING_NAMES = {0: "RIGHT→LIMIT", 1: "LEFT→CENTER", 2: "SLOW RIGHT", 3: "DONE"}
DIR_NAMES = {-1: "LEFT", 0: "STOP", 1: "RIGHT"}
TELEMETRY_RE = re.compile(
    r"seek=(\d+)\s+cen=(\d+)\s+limL=(\d+)\s+limR=(\d+)"
    r".*?aV=([\d.]+)"
    r".*?dir=(-?\d+)\s+pwm=(\d+)"
    r".*?hs=(\d+)\s+seekDir=(\d+)"
)


# ───────────────────── serial reader ─────────────────────
@dataclass
class SerialData:
    """Latest parsed telemetry snapshot from the Arduino."""
    seek: bool = False
    center: bool = False
    limit_left: bool = False
    limit_right: bool = False
    analog_v: float = 0.0
    g_dir: int = 0
    g_pwm: int = 0
    homing_state: int = 0
    seek_dir_right: bool = True
    raw_line: str = ""
    updated: bool = False        # set True after each successful parse
    connected: bool = False
    error_msg: str = ""
    t: float = 0.0


class SerialReader:
    """Background thread that reads & parses Arduino telemetry lines."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self.data = SerialData()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._start_time = time.monotonic()

    # ── public api ──
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass

    def snapshot(self) -> SerialData:
        with self._lock:
            d = SerialData(**{k: getattr(self.data, k) for k in self.data.__dataclass_fields__})
            self.data.updated = False
            return d

    # ── background loop ──
    def _run(self) -> None:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
            # DTR reset to reboot Arduino and start fresh telemetry
            self._ser.dtr = False
            time.sleep(0.1)
            self._ser.dtr = True
            time.sleep(2.0)  # wait for Arduino bootloader
            self._ser.reset_input_buffer()
            with self._lock:
                self.data.connected = True
                self.data.error_msg = ""
        except Exception as exc:
            with self._lock:
                self.data.connected = False
                self.data.error_msg = str(exc)
            return

        buf = ""
        while not self._stop.is_set():
            try:
                raw = self._ser.read(self._ser.in_waiting or 1)
                if not raw:
                    continue
                buf += raw.decode("ascii", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("T "):
                        self._parse(line)
            except Exception as exc:
                with self._lock:
                    self.data.error_msg = str(exc)
                    self.data.connected = False
                break

        if self._ser and self._ser.is_open:
            self._ser.close()

    def _parse(self, line: str) -> None:
        m = TELEMETRY_RE.search(line)
        if not m:
            return
        with self._lock:
            self.data.seek = m.group(1) == "1"
            self.data.center = m.group(2) == "1"
            self.data.limit_left = m.group(3) == "1"
            self.data.limit_right = m.group(4) == "1"
            self.data.analog_v = float(m.group(5))
            self.data.g_dir = int(m.group(6))
            self.data.g_pwm = int(m.group(7))
            self.data.homing_state = int(m.group(8))
            self.data.seek_dir_right = m.group(9) == "1"
            self.data.raw_line = line
            self.data.updated = True
            self.data.t = time.monotonic() - self._start_time


# ───────────────────── simulation engine (unchanged) ─────────────────────
class HomingState(IntEnum):
    SEEK_GO_RIGHT_TO_LIMIT = 0
    SEEK_LEFT_TO_CENTER    = 1
    SEEK_SLOW_RIGHT_REENTER = 2
    SEEK_DONE = 3


@dataclass
class AlignerConfig:
    analog_center_v: float = 2.00
    analog_deadband_v: float = 0.08
    kp: float = 90.0
    pwm_min: int = 55
    pwm_max: int = 200
    err_pos_move_right: bool = True
    seek_pwm_fast: int = 90
    seek_pwm_slow: int = 45
    seek_pwm_fine: int = 25
    bounce_delay_s: float = 0.08
    center_on_width: float = 0.015
    center_off_width: float = 0.022


@dataclass
class AlignerState:
    position: float = 0.75
    velocity: float = 0.0
    mode_seek: bool = True
    homing_state: HomingState = HomingState.SEEK_GO_RIGHT_TO_LIMIT
    seek_dir_right: bool = True
    seek_passed_center: bool = False
    center_sensor_active: bool = False
    limit_left_active: bool = False
    limit_right_active: bool = False
    g_dir: int = 0
    g_pwm: int = 0
    analog_v: float = 0.0
    error_v: float = 0.0
    t: float = 0.0
    last_bounce_t: float = -10.0


@dataclass
class AlignerSimulator:
    cfg: AlignerConfig = field(default_factory=AlignerConfig)
    st: AlignerState = field(default_factory=AlignerState)
    sensor_bias_v: float = 0.0
    sensor_noise_v: float = 0.01

    def reset_homing(self) -> None:
        self.st.homing_state = HomingState.SEEK_GO_RIGHT_TO_LIMIT
        self.st.seek_passed_center = False
        self.st.seek_dir_right = True
        self.motor_stop()

    def motor_stop(self) -> None:
        self.st.g_dir = 0; self.st.g_pwm = 0

    def motor_right(self, pwm: int) -> None:
        self.st.g_dir = +1; self.st.g_pwm = max(0, min(255, pwm))

    def motor_left(self, pwm: int) -> None:
        self.st.g_dir = -1; self.st.g_pwm = max(0, min(255, pwm))

    def _update_limits(self) -> None:
        self.st.limit_left_active = self.st.position <= 0.0
        self.st.limit_right_active = self.st.position >= 1.0

    def _update_center_sensor(self) -> None:
        d = abs(self.st.position - 0.5)
        if self.st.center_sensor_active:
            self.st.center_sensor_active = d <= self.cfg.center_off_width
        else:
            self.st.center_sensor_active = d <= self.cfg.center_on_width

    def _position_to_voltage(self) -> float:
        base = self.st.position * 4.0
        noise = random.uniform(-self.sensor_noise_v, self.sensor_noise_v)
        return max(0.0, min(5.0, base + self.sensor_bias_v + noise))

    def _update_physics(self, dt: float) -> None:
        target_speed = (self.st.g_pwm / 255.0) * 0.9 * float(self.st.g_dir)
        alpha = 0.2
        self.st.velocity = (1 - alpha) * self.st.velocity + alpha * target_speed
        self.st.position += self.st.velocity * dt
        self.st.position = max(0.0, min(1.0, self.st.position))
        if self.st.position <= 0.0 or self.st.position >= 1.0:
            self.st.velocity = 0.0

    def _update_seek_dir_on_limits(self) -> None:
        if self.st.t - self.st.last_bounce_t < self.cfg.bounce_delay_s:
            self.motor_stop(); return
        if self.st.seek_dir_right and self.st.limit_right_active:
            self.motor_stop(); self.st.seek_dir_right = False; self.st.last_bounce_t = self.st.t
        elif (not self.st.seek_dir_right) and self.st.limit_left_active:
            self.motor_stop(); self.st.seek_dir_right = True; self.st.last_bounce_t = self.st.t

    def _homing_step(self) -> None:
        cen = self.st.center_sensor_active
        if self.st.limit_left_active and self.st.limit_right_active:
            self.motor_stop(); return
        hs = self.st.homing_state

        # Stage 0: go RIGHT until right limit (ignore center)
        if hs == HomingState.SEEK_GO_RIGHT_TO_LIMIT:
            if self.st.limit_right_active:
                self.motor_stop()
                self.st.homing_state = HomingState.SEEK_LEFT_TO_CENTER
                return
            self.motor_right(self.cfg.seek_pwm_fast)
            return

        # Stage 1: go LEFT until center sensor ON (first pass)
        if hs == HomingState.SEEK_LEFT_TO_CENTER:
            if cen:
                self.motor_stop()
                self.st.seek_passed_center = False
                self.st.homing_state = HomingState.SEEK_SLOW_RIGHT_REENTER
                return
            if self.st.limit_left_active:
                self.motor_stop(); return
            self.motor_left(self.cfg.seek_pwm_slow)
            return

        # Stage 2: reverse RIGHT slowly; wait for center OFF then ON again
        if hs == HomingState.SEEK_SLOW_RIGHT_REENTER:
            if not self.st.seek_passed_center:
                if not cen:
                    self.st.seek_passed_center = True
            else:
                if cen:
                    self.motor_stop()
                    self.st.homing_state = HomingState.SEEK_DONE
                    return
            if self.st.limit_right_active:
                self.motor_stop(); return
            self.motor_right(self.cfg.seek_pwm_fine)
            return

        # DONE
        self.motor_stop()

    def _analog_control_step(self) -> None:
        v = self.st.analog_v
        err = v - self.cfg.analog_center_v
        self.st.error_v = err
        if abs(err) <= self.cfg.analog_deadband_v:
            self.motor_stop(); return
        pwm = min(int(self.cfg.pwm_min + self.cfg.kp * abs(err)), self.cfg.pwm_max)
        move_right = (err > 0) if self.cfg.err_pos_move_right else (err <= 0)
        if move_right:
            self.motor_stop() if self.st.limit_right_active else self.motor_right(pwm)
        else:
            self.motor_stop() if self.st.limit_left_active else self.motor_left(pwm)

    def step(self, dt: float) -> None:
        self.st.t += dt
        self._update_limits(); self._update_center_sensor()
        self.st.analog_v = self._position_to_voltage()
        if self.st.mode_seek:
            self.motor_stop() if self.st.homing_state == HomingState.SEEK_DONE else self._homing_step()
        else:
            if self.st.homing_state != HomingState.SEEK_GO_RIGHT_TO_LIMIT:
                self.reset_homing()
            self._analog_control_step()
        self._update_physics(dt)
        self._update_limits(); self._update_center_sensor()


# ───────────────────── COM-port helpers ─────────────────────
def available_ports() -> list[str]:
    if not HAS_SERIAL:
        return []
    return [p.device for p in list_ports.comports()]


# ───────────────────── dashboard ─────────────────────
class DashboardApp:
    SRC_SERIAL = "serial"
    SRC_SIM = "simulation"

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Razif Aligner Dashboard")
        self.root.geometry("900x850")

        # data source
        self.source = self.SRC_SIM
        self.sim = AlignerSimulator()
        self.serial_reader: Optional[SerialReader] = None

        self.running = True
        self.history_v = deque(maxlen=500)
        self.history_pwm = deque(maxlen=500)

        # current snapshot for visualization
        self._vis_position = 0.5   # 0..1 normalized
        self._vis_dir = 0
        self._vis_pwm = 0
        self._vis_center = False
        self._vis_lim_l = False
        self._vis_lim_r = False
        self._vis_seek = False
        self._vis_hs = 0
        self._vis_analog_v = 0.0

        # tk vars
        self.source_var = tk.StringVar(value=self.SRC_SIM)
        self.port_var = tk.StringVar(value="")
        self.mode_seek_var = tk.BooleanVar(value=True)
        self.center_target_var = tk.DoubleVar(value=self.sim.cfg.analog_center_v)
        self.bias_var = tk.DoubleVar(value=0.0)

        self._build_ui()
        self._start_sim_worker()
        self._ui_tick()

    # ──────── UI construction ────────
    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)

        # ── connection bar ──
        conn = ttk.LabelFrame(frame, text="Data Source", padding=6)
        conn.pack(fill=tk.X)

        ttk.Radiobutton(conn, text="Simulation", variable=self.source_var,
                        value=self.SRC_SIM, command=self._on_source_change).grid(row=0, column=0, padx=4)
        ttk.Radiobutton(conn, text="Serial (Arduino)", variable=self.source_var,
                        value=self.SRC_SERIAL, command=self._on_source_change).grid(row=0, column=1, padx=4)

        ttk.Label(conn, text="COM:").grid(row=0, column=2, padx=(12, 2))
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=3, padx=2)
        self._refresh_ports()

        ttk.Button(conn, text="Refresh", command=self._refresh_ports, width=8).grid(row=0, column=4, padx=4)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_serial, width=10)
        self.connect_btn.grid(row=0, column=5, padx=4)

        self.conn_status_lbl = ttk.Label(conn, text="Disconnected", foreground="gray")
        self.conn_status_lbl.grid(row=0, column=6, padx=8)

        # ── simulation controls (only active in sim mode) ──
        sim_ctrl = ttk.LabelFrame(frame, text="Simulation Controls", padding=6)
        sim_ctrl.pack(fill=tk.X, pady=(6, 0))

        ttk.Checkbutton(sim_ctrl, text="Seek mode", variable=self.mode_seek_var,
                        command=self._on_mode_change).grid(row=0, column=0, sticky="w")

        ttk.Label(sim_ctrl, text="Target [V]").grid(row=0, column=1, padx=(12, 2))
        ttk.Scale(sim_ctrl, from_=0.5, to=4.5, variable=self.center_target_var,
                  command=lambda _: self._on_target_change()).grid(row=0, column=2, sticky="ew", padx=4)

        ttk.Label(sim_ctrl, text="Bias [V]").grid(row=0, column=3, padx=(12, 2))
        ttk.Scale(sim_ctrl, from_=-0.8, to=0.8, variable=self.bias_var,
                  command=lambda _: self._on_bias_change()).grid(row=0, column=4, sticky="ew", padx=4)

        ttk.Button(sim_ctrl, text="Disturb L", command=lambda: self._kick(-0.07)).grid(row=0, column=5, padx=4)
        ttk.Button(sim_ctrl, text="Disturb R", command=lambda: self._kick(+0.07)).grid(row=0, column=6, padx=4)
        ttk.Button(sim_ctrl, text="Reset Home", command=self.sim.reset_homing).grid(row=0, column=7, padx=4)
        sim_ctrl.columnconfigure(2, weight=1)
        sim_ctrl.columnconfigure(4, weight=1)
        self.sim_ctrl_frame = sim_ctrl

        # ── live state ──
        state = ttk.LabelFrame(frame, text="Live State", padding=6)
        state.pack(fill=tk.X, pady=(6, 0))

        self.state_labels: dict[str, ttk.Label] = {}
        keys = ["mode", "homing_state", "seek_dir", "dir", "pwm",
                "analog_v", "center", "limit_left", "limit_right", "raw_line"]
        for i, key in enumerate(keys):
            r, c = divmod(i, 2)
            ttk.Label(state, text=f"{key}:").grid(row=r, column=c * 2, sticky="w", padx=2)
            lbl = ttk.Label(state, text="-", width=28, anchor="w")
            lbl.grid(row=r, column=c * 2 + 1, sticky="w", padx=(2, 10))
            self.state_labels[key] = lbl

        # ── machine view ──
        machine_box = ttk.LabelFrame(frame, text="Machine View", padding=6)
        machine_box.pack(fill=tk.X, pady=(6, 0))

        self.machine_canvas = tk.Canvas(machine_box, height=120, background="#181c24", highlightthickness=0)
        self.machine_canvas.pack(fill=tk.X)

        # ── homing steps progress ──
        steps_box = ttk.LabelFrame(frame, text="Homing Steps", padding=6)
        steps_box.pack(fill=tk.X, pady=(6, 0))

        self.step_labels: dict[int, ttk.Label] = {}
        step_names = [
            (0, "1. RIGHT→LIMIT", "Go right to limit switch"),
            (1, "2. LEFT→CENTER", "Go left until center sensor ON"),
            (2, "3. SLOW RIGHT", "Reverse right slowly, re-enter center"),
            (3, "4. DONE", "Centered!"),
        ]
        for i, (idx, name, desc) in enumerate(step_names):
            lbl = ttk.Label(steps_box, text=f"  {name}  ", font=("Consolas", 10),
                           background="#2a2e38", foreground="#666", relief="groove", padding=(6, 3))
            lbl.grid(row=0, column=i, padx=4, pady=2, sticky="ew")
            self.step_labels[idx] = lbl
            ttk.Label(steps_box, text=desc, font=("Segoe UI", 7), foreground="#888").grid(
                row=1, column=i, padx=4, sticky="w")
        steps_box.columnconfigure((0, 1, 2, 3), weight=1)

        # ── telemetry graph ──
        plot_box = ttk.LabelFrame(frame, text="Telemetry Graph", padding=6)
        plot_box.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

        self.canvas = tk.Canvas(plot_box, height=160, background="#101216", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    # ──────── port helpers ────────
    def _refresh_ports(self) -> None:
        ports = available_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_serial(self) -> None:
        if self.serial_reader:
            self.serial_reader.stop()
            self.serial_reader = None
            self.connect_btn.config(text="Connect")
            self.conn_status_lbl.config(text="Disconnected", foreground="gray")
            return

        port = self.port_var.get()
        if not port:
            self.conn_status_lbl.config(text="No port selected", foreground="red")
            return

        self.serial_reader = SerialReader(port, baud=115200)
        self.serial_reader.start()
        self.connect_btn.config(text="Disconnect")
        self.conn_status_lbl.config(text=f"Connecting to {port}...", foreground="orange")

    def _on_source_change(self) -> None:
        self.source = self.source_var.get()
        is_sim = self.source == self.SRC_SIM
        for child in self.sim_ctrl_frame.winfo_children():
            try:
                child.config(state="normal" if is_sim else "disabled")
            except Exception:
                pass

    # ──────── sim callbacks ────────
    def _on_mode_change(self) -> None:
        self.sim.st.mode_seek = self.mode_seek_var.get()

    def _on_target_change(self) -> None:
        self.sim.cfg.analog_center_v = float(self.center_target_var.get())

    def _on_bias_change(self) -> None:
        self.sim.sensor_bias_v = float(self.bias_var.get())

    def _kick(self, delta: float) -> None:
        self.sim.st.position = max(0.0, min(1.0, self.sim.st.position + delta))

    # ──────── workers ────────
    def _start_sim_worker(self) -> None:
        def worker() -> None:
            dt = 0.01
            while self.running:
                self.sim.step(dt)
                time.sleep(dt)
        threading.Thread(target=worker, daemon=True).start()

    # ──────── UI tick ────────
    def _ui_tick(self) -> None:
        if self.source == self.SRC_SERIAL and self.serial_reader:
            self._update_from_serial()
        else:
            self._update_from_sim()

        self._draw_machine()
        self._update_homing_steps()
        self._draw_plot()
        self.root.after(80, self._ui_tick)

    def _update_from_serial(self) -> None:
        d = self.serial_reader.snapshot()

        # connection status
        if d.connected:
            self.conn_status_lbl.config(text=f"Connected ({self.serial_reader.port})", foreground="green")
        elif d.error_msg:
            self.conn_status_lbl.config(text=f"Error: {d.error_msg[:40]}", foreground="red")

        self.history_v.append(d.analog_v)
        self.history_pwm.append(d.g_pwm)

        self.state_labels["mode"].config(text="SEEK" if d.seek else "ANALOG")
        self.state_labels["homing_state"].config(text=HOMING_NAMES.get(d.homing_state, str(d.homing_state)))
        self.state_labels["seek_dir"].config(text="RIGHT" if d.seek_dir_right else "LEFT")
        self.state_labels["dir"].config(text=DIR_NAMES.get(d.g_dir, str(d.g_dir)))
        self.state_labels["pwm"].config(text=str(d.g_pwm))
        self.state_labels["analog_v"].config(text=f"{d.analog_v:.3f} V")
        self.state_labels["center"].config(text="ON" if d.center else "OFF")
        self.state_labels["limit_left"].config(text="ON" if d.limit_left else "OFF")
        self.state_labels["limit_right"].config(text="ON" if d.limit_right else "OFF")
        self.state_labels["raw_line"].config(text=d.raw_line[-60:] if d.raw_line else "-")

        # update vis snapshot
        self._vis_position = max(0.0, min(1.0, d.analog_v / 4.0))  # approximate from voltage
        self._vis_dir = d.g_dir
        self._vis_pwm = d.g_pwm
        self._vis_center = d.center
        self._vis_lim_l = d.limit_left
        self._vis_lim_r = d.limit_right
        self._vis_seek = d.seek
        self._vis_hs = d.homing_state
        self._vis_analog_v = d.analog_v

    def _update_from_sim(self) -> None:
        st = self.sim.st
        self.history_v.append(st.analog_v)
        self.history_pwm.append(st.g_pwm)

        self.state_labels["mode"].config(text="SEEK" if st.mode_seek else "ANALOG")
        self.state_labels["homing_state"].config(text=HOMING_NAMES.get(int(st.homing_state), str(st.homing_state)))
        self.state_labels["seek_dir"].config(text="RIGHT" if st.seek_dir_right else "LEFT")
        self.state_labels["dir"].config(text=DIR_NAMES.get(st.g_dir, str(st.g_dir)))
        self.state_labels["pwm"].config(text=str(st.g_pwm))
        self.state_labels["analog_v"].config(text=f"{st.analog_v:.3f} V")
        self.state_labels["center"].config(text="ON" if st.center_sensor_active else "OFF")
        self.state_labels["limit_left"].config(text="ON" if st.limit_left_active else "OFF")
        self.state_labels["limit_right"].config(text="ON" if st.limit_right_active else "OFF")
        self.state_labels["raw_line"].config(text="(simulation)")

        # update vis snapshot
        self._vis_position = st.position
        self._vis_dir = st.g_dir
        self._vis_pwm = st.g_pwm
        self._vis_center = st.center_sensor_active
        self._vis_lim_l = st.limit_left_active
        self._vis_lim_r = st.limit_right_active
        self._vis_seek = st.mode_seek
        self._vis_hs = int(st.homing_state)
        self._vis_analog_v = st.analog_v

    # ──────── machine view ────────
    def _draw_machine(self) -> None:
        c = self.machine_canvas
        w = max(10, c.winfo_width())
        h = max(10, c.winfo_height())
        c.delete("all")

        pad = 30
        rail_y = h // 2
        rx0, rx1 = pad + 20, w - pad - 20
        rail_w = rx1 - rx0

        # ── rail ──
        c.create_line(rx0, rail_y, rx1, rail_y, fill="#555", width=3)

        # ── center zone ──
        cx = rx0 + rail_w * 0.5
        zone_half = rail_w * 0.03
        cen_color = "#00ff88" if self._vis_center else "#2a3a30"
        c.create_rectangle(cx - zone_half, rail_y - 18, cx + zone_half, rail_y + 18,
                           fill=cen_color, outline="#00cc66", width=2)
        c.create_text(cx, rail_y - 28, text="CENTER", fill="#00cc66", font=("Consolas", 8))

        # ── limit switches ──
        lim_size = 10
        # left limit
        ll_color = "#ff4444" if self._vis_lim_l else "#3a2a2a"
        c.create_rectangle(rx0 - lim_size, rail_y - lim_size, rx0 + lim_size, rail_y + lim_size,
                           fill=ll_color, outline="#cc3333", width=2)
        c.create_text(rx0, rail_y - 20, text="LIM-L", fill="#cc3333", font=("Consolas", 7))

        # right limit
        lr_color = "#ff4444" if self._vis_lim_r else "#3a2a2a"
        c.create_rectangle(rx1 - lim_size, rail_y - lim_size, rx1 + lim_size, rail_y + lim_size,
                           fill=lr_color, outline="#cc3333", width=2)
        c.create_text(rx1, rail_y - 20, text="LIM-R", fill="#cc3333", font=("Consolas", 7))

        # ── motor block ──
        pos = max(0.0, min(1.0, self._vis_position))
        mx = rx0 + rail_w * pos
        bw, bh = 20, 28

        # color by PWM intensity
        intensity = min(255, int(self._vis_pwm * 1.2))
        if self._vis_dir == 0:
            motor_color = "#4a5568"
        elif self._vis_dir > 0:
            motor_color = f"#00{intensity:02x}ff"
        else:
            motor_color = f"#ff{max(0,180-intensity):02x}00"

        c.create_rectangle(mx - bw, rail_y - bh, mx + bw, rail_y + bh,
                           fill=motor_color, outline="#e0e0e0", width=2)
        c.create_text(mx, rail_y - 4, text="M", fill="white", font=("Consolas", 12, "bold"))

        # ── direction arrow ──
        if self._vis_dir != 0:
            arrow_len = 18 + self._vis_pwm / 15
            ax = mx + (arrow_len if self._vis_dir > 0 else -arrow_len)
            c.create_line(mx, rail_y + bh + 8, ax, rail_y + bh + 8,
                          fill="#ffcc00", width=3, arrow=tk.LAST, arrowshape=(8, 10, 4))
            c.create_text((mx + ax) / 2, rail_y + bh + 20,
                          text=f"PWM={self._vis_pwm}", fill="#ffcc00", font=("Consolas", 7))
        else:
            c.create_text(mx, rail_y + bh + 12, text="STOP", fill="#888", font=("Consolas", 8))

        # ── analog voltage bar ──
        bar_y = 12
        bar_h = 8
        bar_x0 = rx0
        bar_x1 = rx1
        c.create_rectangle(bar_x0, bar_y, bar_x1, bar_y + bar_h, fill="#1a1e28", outline="#3d4550")
        v_frac = max(0.0, min(1.0, self._vis_analog_v / 5.0))
        c.create_rectangle(bar_x0, bar_y, bar_x0 + (bar_x1 - bar_x0) * v_frac, bar_y + bar_h,
                           fill="#39c5bb", outline="")
        c.create_text(bar_x0 - 5, bar_y + bar_h // 2, text=f"{self._vis_analog_v:.2f}V",
                      fill="#39c5bb", font=("Consolas", 7), anchor="e")

    def _update_homing_steps(self) -> None:
        for idx, lbl in self.step_labels.items():
            if idx < self._vis_hs:
                lbl.config(background="#1a3a1a", foreground="#44cc44")   # completed
            elif idx == self._vis_hs:
                if self._vis_seek:
                    lbl.config(background="#3a3a00", foreground="#ffee44")  # active
                else:
                    lbl.config(background="#2a2e38", foreground="#666")    # inactive (analog)
            else:
                lbl.config(background="#2a2e38", foreground="#666")        # pending

    # ──────── telemetry plot ────────
    def _draw_plot(self) -> None:
        w = max(10, self.canvas.winfo_width())
        h = max(10, self.canvas.winfo_height())
        self.canvas.delete("all")

        pad = 20
        x0, y0, x1, y1 = pad, pad, w - pad, h - pad
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#3d4550")
        self.canvas.create_text(x0 + 8, y0 + 8, text="5V", fill="#98a3b3", anchor="nw")
        self.canvas.create_text(x0 + 8, y1 - 8, text="0V", fill="#98a3b3", anchor="sw")

        if len(self.history_v) < 2:
            return

        n = len(self.history_v)

        def x_at(i: int) -> float:
            return x0 + (i / (n - 1)) * (x1 - x0)

        def y_v(v: float) -> float:
            return y1 - (max(0.0, min(5.0, v)) / 5.0) * (y1 - y0)

        def y_pwm(p: int) -> float:
            return y1 - (max(0, min(255, p)) / 255.0) * (y1 - y0)

        # target line
        target = self.sim.cfg.analog_center_v if self.source == self.SRC_SIM else 2.0
        self.canvas.create_line(x0, y_v(target), x1, y_v(target), fill="#b48cf8", dash=(3, 3))

        pts_v = []
        pts_pwm = []
        for i, (v, p) in enumerate(zip(self.history_v, self.history_pwm)):
            xx = x_at(i)
            pts_v.extend((xx, y_v(v)))
            pts_pwm.extend((xx, y_pwm(p)))

        self.canvas.create_line(*pts_v, fill="#39c5bb", width=2)
        self.canvas.create_line(*pts_pwm, fill="#ffb454", width=1)

        legend = "Voltage: cyan | PWM: orange | Target: purple"
        self.canvas.create_text(x1 - 8, y0 + 8, text=legend, fill="#98a3b3", anchor="ne")

    # ──────── cleanup ────────
    def close(self) -> None:
        self.running = False
        if self.serial_reader:
            self.serial_reader.stop()


# ───────────────────── main ─────────────────────
def main() -> None:
    root = tk.Tk()
    app = DashboardApp(root)

    def on_close() -> None:
        app.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
