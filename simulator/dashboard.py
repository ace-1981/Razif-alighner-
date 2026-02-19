import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
import tkinter as tk
from tkinter import ttk


class HomingState(IntEnum):
    SEEK_FIND_CENTER_FAST = 0
    SEEK_BACKOFF_UNTIL_OFF = 1
    SEEK_APPROACH_FINE = 2
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
    homing_state: HomingState = HomingState.SEEK_FIND_CENTER_FAST
    seek_dir_right: bool = True

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
        self.st.homing_state = HomingState.SEEK_FIND_CENTER_FAST
        self.st.seek_dir_right = True
        self.motor_stop()

    def motor_stop(self) -> None:
        self.st.g_dir = 0
        self.st.g_pwm = 0

    def motor_right(self, pwm: int) -> None:
        pwm = max(0, min(255, pwm))
        self.st.g_dir = +1
        self.st.g_pwm = pwm

    def motor_left(self, pwm: int) -> None:
        pwm = max(0, min(255, pwm))
        self.st.g_dir = -1
        self.st.g_pwm = pwm

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
        v = base + self.sensor_bias_v + noise
        return max(0.0, min(5.0, v))

    def _update_physics(self, dt: float) -> None:
        target_speed = (self.st.g_pwm / 255.0) * 0.9 * float(self.st.g_dir)
        alpha = 0.2
        self.st.velocity = (1 - alpha) * self.st.velocity + alpha * target_speed
        self.st.position += self.st.velocity * dt

        if self.st.position < 0.0:
            self.st.position = 0.0
            self.st.velocity = 0.0
        if self.st.position > 1.0:
            self.st.position = 1.0
            self.st.velocity = 0.0

    def _update_seek_dir_on_limits(self) -> None:
        if self.st.t - self.st.last_bounce_t < self.cfg.bounce_delay_s:
            self.motor_stop()
            return

        if self.st.seek_dir_right and self.st.limit_right_active:
            self.motor_stop()
            self.st.seek_dir_right = False
            self.st.last_bounce_t = self.st.t
        elif (not self.st.seek_dir_right) and self.st.limit_left_active:
            self.motor_stop()
            self.st.seek_dir_right = True
            self.st.last_bounce_t = self.st.t

    def _homing_step(self) -> None:
        cen = self.st.center_sensor_active

        if self.st.limit_left_active and self.st.limit_right_active:
            self.motor_stop()
            return

        hs = self.st.homing_state

        if hs == HomingState.SEEK_FIND_CENTER_FAST:
            if cen:
                self.motor_stop()
                self.st.homing_state = HomingState.SEEK_BACKOFF_UNTIL_OFF
                return

            self._update_seek_dir_on_limits()
            if self.st.g_dir == 0 and self.st.g_pwm == 0 and self.st.t - self.st.last_bounce_t < self.cfg.bounce_delay_s:
                return

            if self.st.seek_dir_right:
                if not self.st.limit_right_active:
                    self.motor_right(self.cfg.seek_pwm_fast)
                else:
                    self.motor_stop()
            else:
                if not self.st.limit_left_active:
                    self.motor_left(self.cfg.seek_pwm_fast)
                else:
                    self.motor_stop()
            return

        if hs == HomingState.SEEK_BACKOFF_UNTIL_OFF:
            if not cen:
                self.motor_stop()
                self.st.homing_state = HomingState.SEEK_APPROACH_FINE
                return

            if self.st.seek_dir_right:
                if not self.st.limit_left_active:
                    self.motor_left(self.cfg.seek_pwm_slow)
                elif not self.st.limit_right_active:
                    self.motor_right(self.cfg.seek_pwm_slow)
                else:
                    self.motor_stop()
            else:
                if not self.st.limit_right_active:
                    self.motor_right(self.cfg.seek_pwm_slow)
                elif not self.st.limit_left_active:
                    self.motor_left(self.cfg.seek_pwm_slow)
                else:
                    self.motor_stop()
            return

        if hs == HomingState.SEEK_APPROACH_FINE:
            if cen:
                self.motor_stop()
                self.st.homing_state = HomingState.SEEK_DONE
                return

            self._update_seek_dir_on_limits()
            if self.st.g_dir == 0 and self.st.g_pwm == 0 and self.st.t - self.st.last_bounce_t < self.cfg.bounce_delay_s:
                return

            if self.st.seek_dir_right:
                if not self.st.limit_right_active:
                    self.motor_right(self.cfg.seek_pwm_fine)
                else:
                    self.motor_stop()
            else:
                if not self.st.limit_left_active:
                    self.motor_left(self.cfg.seek_pwm_fine)
                else:
                    self.motor_stop()
            return

        self.motor_stop()

    def _analog_control_step(self) -> None:
        v = self.st.analog_v
        err = v - self.cfg.analog_center_v
        self.st.error_v = err

        if abs(err) <= self.cfg.analog_deadband_v:
            self.motor_stop()
            return

        pwm = int(self.cfg.pwm_min + self.cfg.kp * abs(err))
        pwm = min(pwm, self.cfg.pwm_max)

        move_right = (err > 0) if self.cfg.err_pos_move_right else (err <= 0)

        if move_right:
            if self.st.limit_right_active:
                self.motor_stop()
            else:
                self.motor_right(pwm)
        else:
            if self.st.limit_left_active:
                self.motor_stop()
            else:
                self.motor_left(pwm)

    def step(self, dt: float) -> None:
        self.st.t += dt

        self._update_limits()
        self._update_center_sensor()
        self.st.analog_v = self._position_to_voltage()

        if self.st.mode_seek:
            if self.st.homing_state == HomingState.SEEK_DONE:
                self.motor_stop()
            else:
                self._homing_step()
        else:
            if self.st.homing_state != HomingState.SEEK_FIND_CENTER_FAST:
                self.reset_homing()
            self._analog_control_step()

        self._update_physics(dt)
        self._update_limits()
        self._update_center_sensor()


class DashboardApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Razif Aligner Simulator")
        self.sim = AlignerSimulator()

        self.running = True
        self.history_t = deque(maxlen=400)
        self.history_v = deque(maxlen=400)
        self.history_pos = deque(maxlen=400)

        self.mode_seek_var = tk.BooleanVar(value=True)
        self.center_target_var = tk.DoubleVar(value=self.sim.cfg.analog_center_v)
        self.bias_var = tk.DoubleVar(value=0.0)

        self._build_ui()
        self._start_worker()
        self._ui_tick()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        ctrl = ttk.LabelFrame(frame, text="Control", padding=8)
        ctrl.pack(fill=tk.X)

        ttk.Checkbutton(
            ctrl,
            text="Seek mode",
            variable=self.mode_seek_var,
            command=self._on_mode_change,
        ).grid(row=0, column=0, sticky="w")

        ttk.Label(ctrl, text="Analog target [V]").grid(row=1, column=0, sticky="w")
        ttk.Scale(ctrl, from_=0.5, to=4.5, variable=self.center_target_var, command=lambda _e: self._on_target_change()).grid(
            row=1, column=1, sticky="ew", padx=8
        )

        ttk.Label(ctrl, text="Sensor bias [V]").grid(row=2, column=0, sticky="w")
        ttk.Scale(ctrl, from_=-0.8, to=0.8, variable=self.bias_var, command=lambda _e: self._on_bias_change()).grid(
            row=2, column=1, sticky="ew", padx=8
        )

        ttk.Button(ctrl, text="Disturb Left", command=lambda: self._kick(-0.07)).grid(row=0, column=2, padx=6)
        ttk.Button(ctrl, text="Disturb Right", command=lambda: self._kick(+0.07)).grid(row=0, column=3, padx=6)
        ttk.Button(ctrl, text="Reset Homing", command=self.sim.reset_homing).grid(row=1, column=2, columnspan=2, padx=6)

        ctrl.columnconfigure(1, weight=1)

        state = ttk.LabelFrame(frame, text="Live State", padding=8)
        state.pack(fill=tk.X, pady=(8, 0))

        self.state_labels = {}
        rows = [
            "time",
            "position",
            "analog_v",
            "error_v",
            "mode",
            "homing_state",
            "seek_dir",
            "dir",
            "pwm",
            "center",
            "limit_left",
            "limit_right",
        ]
        for i, key in enumerate(rows):
            ttk.Label(state, text=f"{key}:").grid(row=i // 2, column=(i % 2) * 2, sticky="w")
            lbl = ttk.Label(state, text="-")
            lbl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(4, 12))
            self.state_labels[key] = lbl

        plot_box = ttk.LabelFrame(frame, text="Telemetry", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.canvas = tk.Canvas(plot_box, height=240, background="#101216", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def _on_mode_change(self) -> None:
        self.sim.st.mode_seek = self.mode_seek_var.get()

    def _on_target_change(self) -> None:
        self.sim.cfg.analog_center_v = float(self.center_target_var.get())

    def _on_bias_change(self) -> None:
        self.sim.sensor_bias_v = float(self.bias_var.get())

    def _kick(self, delta: float) -> None:
        self.sim.st.position = max(0.0, min(1.0, self.sim.st.position + delta))

    def _start_worker(self) -> None:
        def worker() -> None:
            dt = 0.01
            while self.running:
                self.sim.step(dt)
                time.sleep(dt)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def _ui_tick(self) -> None:
        st = self.sim.st

        self.history_t.append(st.t)
        self.history_v.append(st.analog_v)
        self.history_pos.append(st.position)

        self.state_labels["time"].config(text=f"{st.t:0.2f}s")
        self.state_labels["position"].config(text=f"{st.position:0.3f}")
        self.state_labels["analog_v"].config(text=f"{st.analog_v:0.3f} V")
        self.state_labels["error_v"].config(text=f"{st.error_v:0.3f} V")
        self.state_labels["mode"].config(text="SEEK" if st.mode_seek else "ANALOG")
        self.state_labels["homing_state"].config(text=st.homing_state.name)
        self.state_labels["seek_dir"].config(text="RIGHT" if st.seek_dir_right else "LEFT")
        self.state_labels["dir"].config(text={-1: "LEFT", 0: "STOP", 1: "RIGHT"}[st.g_dir])
        self.state_labels["pwm"].config(text=str(st.g_pwm))
        self.state_labels["center"].config(text="ON" if st.center_sensor_active else "OFF")
        self.state_labels["limit_left"].config(text="ON" if st.limit_left_active else "OFF")
        self.state_labels["limit_right"].config(text="ON" if st.limit_right_active else "OFF")

        self._draw_plot()
        self.root.after(80, self._ui_tick)

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
            v = max(0.0, min(5.0, v))
            return y1 - (v / 5.0) * (y1 - y0)

        def y_pos(p: float) -> float:
            p = max(0.0, min(1.0, p))
            return y1 - p * (y1 - y0)

        target_y = y_v(self.sim.cfg.analog_center_v)
        self.canvas.create_line(x0, target_y, x1, target_y, fill="#b48cf8", dash=(3, 3))

        pts_v = []
        pts_p = []
        for i, (v, p) in enumerate(zip(self.history_v, self.history_pos)):
            xx = x_at(i)
            pts_v.extend((xx, y_v(v)))
            pts_p.extend((xx, y_pos(p)))

        self.canvas.create_line(*pts_v, fill="#39c5bb", width=2)
        self.canvas.create_line(*pts_p, fill="#ffb454", width=2)

        self.canvas.create_text(x1 - 8, y0 + 8, text="V: cyan | Pos: orange", fill="#98a3b3", anchor="ne")

    def close(self) -> None:
        self.running = False


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
