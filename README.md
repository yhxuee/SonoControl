# SonoControl

Ultrasound thermal controller — desktop application for driving an FPGA-based ultrasound device while monitoring and controlling temperature via a thermocouple meter.

---

## Overview

SonoControl sends serial and UDP commands to an ultrasound FPGA to transmit therapeutic ultrasound bursts at configurable frequency, power, duty cycle, and repetition rate. It simultaneously reads a Mastech HH806AU dual-channel thermocouple meter over a second serial port and optionally closes a PID loop to regulate tissue temperature. All data is streamed to a timestamped CSV log in real time.

The project provides:
- A **Qt GUI** (`sonocontrol_gui`) for interactive experiment control with live temperature and waveform plots.
- A **CLI binary** (`sonocontrol`) for scripted or headless operation.

---

## Features

| Category | Feature |
|----------|---------|
| Ultrasound | Amplitude, carrier frequency (kHz), PRF (Hz), duty cycle, duration, interval |
| Waveforms | Sine, square, or triangle envelope over 4096-sample DDS table via UDP |
| Temperature | HH806AU dual thermocouple (T1, T2, average), configurable channel fallback |
| PID control | Kp/Ki/Kd with predictive thermal model: `T_future = T + τ × dT/dt × (1 − e^{−t/τ})` |
| Experiment modes | Total duration, repeating cycles, or hold-after-target-reached |
| Cycling | Alternating heating/cooling phases with configurable durations |
| Safety | Configurable temperature cutoff with debounce (N samples, min spacing) |
| Logging | Streaming CSV log + JSON metadata; auto-save artifacts on completion |
| GUI | Dark/light theme, real-time temperature plot, waveform preview, preflight checks |
| Reliability | Exponential-backoff serial retry, persistent COM3 option, stall watchdog |

---

## Requirements

### Software
| Requirement | Version |
|-------------|---------|
| C++ standard | C++17 |
| CMake | 3.20 or later |
| Qt | 5.15 or Qt 6.x (Widgets module) |
| Compiler | MSVC 2019+, MinGW-w64 8+, or GCC 9+ (Linux) |

> **ABI warning**: Qt and the compiler must share the same ABI. Do not mix an MSVC Qt build with a MinGW compiler or vice versa. CMake will print a fatal error if it detects the mismatch.

### Hardware
| Device          | Connection | Purpose |
|-----------------|-----------|---------|
| Ultrasound FPGA | COM3 (9600 8N1) | Receives serial control packets |
| Ultrasound FPGA | UDP target (default `192.168.0.2:5000`) | Receives 4096-sample waveform table |
| Omega HH806AU   | COM5 (19200 8E1) | Dual-channel thermocouple readout |

All port names, baud rates, and the UDP target are configurable.

---

## Build Instructions

### 1. Clone / extract the repository

```
sonocontrol_fixed/
├── CMakeLists.txt
├── include/
├── src/
│   ├── main.cpp          (CLI entry point)
│   └── gui/
│       └── main_gui.cpp  (Qt GUI entry point)
└── README.md
```

### 2. Configure with CMake

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

To build only the CLI (skip Qt):

```bash
cmake -S . -B build -DSONOCONTROL_BUILD_GUI=OFF -DCMAKE_BUILD_TYPE=Release
```

If Qt is installed in a non-standard location, set `CMAKE_PREFIX_PATH`:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:/Qt/6.6.0/msvc2019_64" -DCMAKE_BUILD_TYPE=Release
```

### 3. Build

```bash
cmake --build build --config Release
```

Binaries are placed in `build/bin/`.

### 4. Windows deployment (Qt DLLs)

After building, run `windeployqt` to copy required Qt DLLs next to the executable:

```powershell
windeployqt build\bin\sonocontrol_gui.exe
```

### 5. Debug builds

Debug builds set `SONOCONTROL_DEBUG_SIM=1`, which enables the "Sim Temp" and "Sim US" checkboxes in the GUI and allows testing without hardware. Release builds hard-disable simulation at compile time.

---

## Usage

### GUI

Launch `sonocontrol_gui.exe`. The interface has four tabs on the left panel:

| Tab | Purpose |
|-----|---------|
| **PARAMS** | Ultrasound parameters, timing, experiment length, safety cutoff |
| **PID** | Enable PID, setpoint, Kp/Ki/Kd, thermal prediction constant and horizon |
| **CYCLE** | Heating/cooling cycle durations and cooling behaviour |
| **CONNECT** | COM ports, UDP host/port, HH806AU settings, appearance |

**Workflow:**
1. Select COM ports in the CONNECT tab; click **Scan Ports** if needed.
2. If using temperature monitoring, click **Test** next to the HH806AU port to verify sensor response.
3. Set ultrasound parameters and experiment length in PARAMS.
4. Optionally configure PID in the PID tab.
5. Click **▶ START**. A preflight dialog shows a full configuration summary and requires confirmation.
6. Click **■ EMERGENCY STOP** to stop. A graceful stop is attempted first; after 2 s the stop escalates to cancel pending I/O if the worker is wedged.

**Saving / loading configuration:**
- **File → Save Configuration** writes a `.config` file with all current settings.
- **File → Load Configuration** restores a previously saved configuration.
- **File → Write Configuration Template** creates a fully-commented `.config` template.

**Data export:**
- **File → Export CSV** exports the temperature log from the current session.
- **File → Export Picture** saves the temperature plot as PNG.
- After a successful experiment, all artifacts (CSV, JSON metadata, plots) are auto-saved under `./experiments/<session>/` or the configured auto-save directory.

### CLI

```
sonocontrol [options]
```

**Key options:**

```
Ultrasound:
  --amplitude X         Amplitude 0..1 (default 0.5)
  --cfreq-khz X         Carrier frequency in kHz (default 500)
  --prf-hz X            Pulse repetition frequency in Hz (default 1000)
  --duty-pct X          Duty cycle 0..100 % (default 50)
  --duration-ms N       Ultrasound burst duration in ms (default 1000)
  --interval-s X        Interval between bursts in seconds (default 5)
  --wave sine|square|triangle  Waveform shape (default sine)

Experiment length:
  --total-min X         Total duration in minutes (default 60)
  --repeating N         Repeating cycles mode (N cycles)
  --hold-after-target-min X  Hold for X minutes after target temp is reached

Temperature / PID:
  --com11 PORT          Temperature sensor COM port (default COM5)
  --pid                 Enable PID control
  --setpoint X          PID temperature setpoint in °C (default 40)
  --pid-kp X            Proportional gain (default 0.8)
  --pid-ki X            Integral gain (default 0.05)
  --pid-kd X            Derivative gain (default 0.2)
  --pid-tau X           Thermal time constant in seconds (default 25)
  --pid-horizon X       Prediction horizon in seconds (0 = use interval)

Hardware:
  --com3 PORT           Ultrasound serial port (default COM3)
  --udp-host HOST       UDP target host (default 192.168.0.2)
  --udp-port PORT       UDP target port (default 5000)

Config:
  --config FILE         Load settings from .config file
  --write-config-template FILE  Write a template .config file and exit

Diagnostics:
  --protocol-check      Print encoded packets for default params and exit
  --help                Show full option list
```

**Example:**

```bash
sonocontrol --cfreq-khz 500 --amplitude 0.6 --duty-pct 50 --duration-ms 1000 --interval-s 5 --total-min 30 --pid --setpoint 40 --com3 COM3 --com11 COM5
```

---

## Configuration File

A `.config` file is a UTF-8 key-value text file. Lines starting with `#` or `;` are comments. Sections (`[section]`) are optional and ignored.

Example:

```ini
[ultrasound]
amplitude           = 0.6
cfreq_hz            = 500000
prf_hz              = 1000
duty_cycle          = 0.50
wave_shape          = sine
duration_ms         = 1000
interval_time_s     = 5.0

[experiment]
length_mode         = total_duration   # total_duration | repeating_cycles | hold_after_target
total_duration_mins = 60.0

[temperature]
temperature_enabled = true
temp_channel        = T1               # T1 | T2 | Average
temp_channel_fallback = false
min_plausible_temp_c = 10.0
max_plausible_temp_c = 80.0
max_temp_rate_c_per_s = 15.0
sampling_rate_hz    = 2.0
cutoff_temp         = 45.0
cutoff_confirm_samples = 2
cutoff_confirm_min_spacing_ms = 150

[pid]
pid_enabled         = true
pid_setpoint        = 40.0
pid_kp              = 0.8
pid_ki              = 0.05
pid_kd              = 0.2
pid_prediction_tau_s = 25.0
pid_prediction_horizon_s = 0.0

[hardware]
com3_port           = COM3
com11_port          = COM5
udp_host            = 192.168.0.2
udp_port            = 5000

[reliability]
persistent_com3     = true
communication_retry = true
communication_retry_attempts = 3
communication_retry_initial_backoff_ms = 80
emergency_stop_repeats = 5
watchdog_timeout_ms = 5000
```

Generate a fully-commented template:

```bash
sonocontrol --write-config-template sonocontrol_config_template.config
```

---

## Hardware Connection Notes

### Ultrasound FPGA (COM3 + UDP)

- Serial port: 9600 baud, 8 data bits, no parity, 1 stop bit.
- Each control packet is 8 bytes: `AA 03 <cmd> <payload LE 4 bytes> 88`.
- Commands: `0x10` = PRF, `0x11` = carrier frequency (both as 32-bit DDS word), `0x06` = duration in ms. Duration = 0 is the hardware STOP command.
- Waveform table: 4096 UDP packets, each 8 bytes: `BB 03 00 <index hi> <index lo> <amp hi> <amp lo> 88`. DDS amplitude is 13-bit (0–8191 = MIDPOINT, range 0–16381).
- Set `persistent_com3 = true` (default) to keep the serial port open across bursts; set to `false` to open/close per burst (useful if the OS resets the device on open).

### HH806AU Thermocouple Meter (COM5)

- Port: 19200 baud, 8 data bits, **even parity**, 1 stop bit.
- Poll command: `#0A0000NA2\r\n` (12 bytes). Response: 14 bytes.
- Valid frame starts with `3E 0F`; status byte for each channel must be `0x10`.
- Temperature is encoded as a BCD-like 16-bit word; the driver auto-detects byte order.
- If the meter does not respond within 300 ms or returns a malformed frame, the read returns `nullopt` for both channels.
- Connect T1 and/or T2 probes. Select the active channel in the GUI CONNECT tab. Enable **Allow fallback** only when both probes are physically installed; otherwise a spurious fallback to an open probe may log room-temperature readings.

### Network / UDP

- The source port is fixed at **4561** and must be bindable (no other SonoControl instance running).
- The UDP destination defaults to `192.168.0.2:5000`. Change this in the CONNECT tab or via `--udp-host` / `--udp-port`.
- If the FPGA is on a different subnet, ensure the host machine has a static IP on that subnet and no firewall blocks port 5000 UDP.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| "Cannot open ultrasound serial port" | Port in use by another process or COM number wrong | Close other terminal programs; rescan in CONNECT tab |
| "Cannot bind UDP source port 4561" | Another SonoControl instance is still running | Kill the old process (Task Manager) and restart |
| HH806AU Test returns "N/C" after 3 attempts | Wrong COM port, meter powered off, or 8E1 mismatch | Verify COM port in Device Manager; confirm meter is on |
| Temperature pilot shows only one channel | Probe unplugged or physically broken | Check thermocouple wires; enable fallback only if the other probe is installed |
| Worker stall watchdog fires mid-run | USB-serial driver hang | Unplug/replug the USB-serial cable; update FTDI/CH340 driver |
| Cutoff triggers unexpectedly | Single noisy sample from the meter | Increase `cutoff_confirm_samples` (default 2); add `cutoff_confirm_min_spacing_ms` |
| PID overshoots severely | Kp too high, or tau/horizon mismatch for the sample | Lower Kp; increase tau to match the thermal time constant of your load |
| Experiment ends immediately | Total duration set to 0 min, or `repeating` cycles = 0 | Check PARAMS tab experiment length section |
| Build error: "Incompatible Qt/compiler" | MSVC build with MinGW Qt (or vice versa) | Match Qt ABI to compiler; see Build Instructions |

---

## Known Limitations

- **Windows only** for the GUI binary in practice (the core library is cross-platform, and the CLI builds on Linux).
- The UDP waveform burst sends 4096 individual 8-byte datagrams sequentially. On congested networks this can take tens of milliseconds; keep the FPGA on a direct or isolated Ethernet link.
- Temperature sampling is limited to 2 Hz by the HH806AU poll cycle (~360 ms per read). Higher `sampling_rate_hz` settings do not increase actual data rate.
- PID integral windup limit is hardcoded at ±10 (not configurable via the config file). For very slow systems this may be too tight.
- The EMA smoothing coefficient (`α = 0.25`) is compile-time constant, not configurable. Adjust in `src/experiment.cpp` if needed.
- Cycling mode is only available in Total Duration mode; it is silently disabled in Repeating or Hold-After-Target modes.
- Auto-save creates files only on clean completion (exit code 0). Manual stops and cutoff events do not trigger auto-save.

---

## Development Notes

### Source layout

```
include/
  config.hpp          Config + ActiveParams structs; enums
  config_io.hpp       load/save/template/to_text declarations
  experiment.hpp      ExperimentRunner class + ExperimentCallbacks
  logger.hpp          ExperimentLogger (streaming CSV + JSON metadata)
  pid.hpp             PIDController + apply_pid_to_params()
  protocol.hpp        Serial packet helpers; UDP burst builder; waveform arrays
  serial_port.hpp     Platform-abstracted serial port (Windows + POSIX)
  temperature.hpp     ITemperatureSensor; HH806AUSensor; TemperatureSimulator
  udp_socket.hpp      UdpSender wrapper

src/
  config_io.cpp
  experiment.cpp      Main experiment loop (state machine)
  logger.cpp
  pid.cpp
  protocol.cpp        DDS math; waveform table generators
  serial_port.cpp     Windows (CreateFile/ReadFile/WriteFile) + POSIX (termios)
  temperature.cpp     HH806AU frame parser; plausibility filter; physics simulator
  udp_socket.cpp
  main.cpp            CLI entry point + argument parser
  gui/
    main_gui.cpp      Qt Widgets GUI (single-file, ~2500 lines)
```

### Key design decisions

- `ExperimentRunner::run()` is a blocking call, run on a `QThread` worker; all callbacks are emitted as Qt signals across the thread boundary.
- Missing temperature readings use `NaN` as a sentinel (not `0.0`) so that readings near 0 °C are not incorrectly suppressed.
- `force_stop()` calls `cancel_io()` on the serial and UDP handles, unblocking any kernel-level I/O call on the worker thread. This is the mechanism behind both the 2 s auto-escalation and the stall watchdog.
- Serial packets include a 100 ms inter-packet gap (`COM_GAP_S`). This is hardware-required and must not be reduced.

### Adding a new length mode or waveform

1. Add a value to `LengthMode` or `WaveShape` in `include/config.hpp`.
2. Add `to_string` / `from_string` cases in `src/protocol.cpp`.
3. Add a config key in `src/config_io.cpp` (`parse_kv` and `config_to_text`).
4. Handle the new mode in `src/experiment.cpp` (`run()`).
5. Add a UI control in `src/gui/main_gui.cpp` (`buildParamsTab` or `buildPidTab`) and update `buildConfig()` / `applyConfigToUi()`.
