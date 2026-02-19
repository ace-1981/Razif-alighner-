# Python Simulator Dashboard

This dashboard simulates your Arduino aligner logic in parallel:
- `SEEK` mode with homing state machine
- `ANALOG` mode with deadband and proportional PWM
- Center sensor + left/right limits
- Live telemetry graph for analog voltage and position

## Run
From repo root:

```powershell
python .\simulator\dashboard.py
```

If `python` is not in PATH, use your local Python launcher:

```powershell
py .\simulator\dashboard.py
```

## Controls
- **Seek mode**: switch between homing and analog control.
- **Analog target [V]**: center setpoint equivalent to `ANALOG_CENTER_V`.
- **Sensor bias [V]**: emulate sensor offset/noise conditions.
- **Disturb Left/Right**: apply manual position disturbance.
- **Reset Homing**: reset state machine to `SEEK_FIND_CENTER_FAST`.
