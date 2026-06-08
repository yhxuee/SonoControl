# SonoControl

Ultrasound thermal controller — desktop application for driving an FPGA-based ultrasound device while monitoring and controlling temperature via a thermocouple meter.

Current release: **2.0.0**

---

## Overview

SonoControl supports two ultrasound controller families:

- **Zhuhai / SonoControl FPGA**: serial COM control plus UDP waveform-table upload.
- **Hyus LAN device**: Ethernet discovery/control using the TCP/UDP protocol captured in the development notes.

Both device modes share the HH806AU dual-channel thermocouple path, safety cutoff, PID controls, after-target hold mode, sequence runner, and experiment logging. All data is streamed to timestamped CSV/JSON artifacts in real time.

The project provides:
- A **Qt GUI** (`sonocontrol_gui`) for interactive experiment control with live temperature and pulse-sequence plots.
- A **CLI binary** (`sonocontrol`) for scripted or headless operation of the original FPGA workflow.

---

## Features

| Category | Feature |
|----------|---------|
| Device modes | Startup device picker for Zhuhai / SonoControl FPGA or Hyus LAN device |
| FPGA ultrasound | Amplitude, carrier frequency (kHz), PRF (Hz), duty cycle, duration, interval |
| FPGA waveforms | Sine, square, or triangle envelope over 4096-sample DDS table via UDP |
| Hyus ultrasound | LAN discovery/control, pulse frequency/amplitude, pulse length/period, sequence length/period |
| Temperature | HH806AU dual thermocouple (T1, T2, average), configurable channel fallback |
| PID control | Kp/Ki/Kd with predictive thermal model: `T_future = T + τ × dT/dt × (1 − e^{−t/τ})` |
| Experiment modes | Total duration, repeating cycles, or hold-after-target-reached on both device modes |
| Cycling | Alternating heating/cooling phases with configurable durations |
| **Sequence** | **Queue multiple `.config` files back-to-back with per-gap intervals; each entry runs as an independent experiment with its own log** |
| **Monitoring web server** | **Optional read-only HTTP endpoint (default 127.0.0.1:50896) that serves a self-contained HTML+JS page with the latest cached temperature samples, elapsed/remaining time, and run state. Configurable port, snapshot interval, and LAN-bind toggle. Off by default at every app launch.** |
| Safety | Configurable temperature cutoff with debounce (N samples, min spacing); optional fail-closed mode that aborts on sustained sensor loss; PIN-gated manual stop (per run and per sequence) |
| Logging | Streaming CSV log + JSON metadata; auto-save artifacts on completion |
| GUI | Dark/light theme, real-time temperature plot, reference-style pulse-sequence preview, preflight checks |
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
| Hyus LAN device | Ethernet, TCP server on `8192`, UDP beacon on `8193` | Receives scalar pulse/sequence parameters and run/stop commands |
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

Launch `sonocontrol_gui.exe`. At startup, choose the target device mode:

- **Zhuhai / SonoControl FPGA** for the original COM3 + UDP hardware.
- **Hyus LAN** for the Ethernet device. The PC listens on TCP `8192` and broadcasts the discovery beacon on UDP `8193`; the detected device appears in the CONNECT tab.

The interface has four tabs on the left panel:

| Tab | Purpose |
|-----|---------|
| **PARAMS** | Device-specific ultrasound timing, experiment length, safety cutoff, pulse-sequence preview |
| **PID** | Enable PID, setpoint, Kp/Ki/Kd, thermal prediction constant, horizon, temperature smoothing window, and Hyus controlled-variable selection |
| **CYCLE** | Heating/cooling cycle durations and cooling behaviour |
| **CONNECT** | FPGA COM/UDP fields or Hyus LAN discovery, HH806AU settings, appearance |

**Workflow (single run):**
1. Select the device connection in the CONNECT tab. For FPGA, choose COM ports and UDP target; for Hyus, click **Scan** and select the detected LAN device.
2. If using temperature monitoring, click **Test** next to the HH806AU port to verify sensor response.
3. Set ultrasound parameters and experiment length in PARAMS. The pulse-sequence panel previews pulse length, pulse period, sequence length, sequence period, amplitude, and total-duration span.
4. Optionally configure PID in the PID tab.
5. Click **▶ START**. A preflight dialog shows a full configuration summary and requires confirmation.
6. Click **■ EMERGENCY STOP** to stop. A graceful stop is attempted first; after 2 s the stop escalates to cancel pending I/O if the worker is wedged.

For unattended back-to-back runs use **Edit → Sequence…** (see the *Experiment Sequence* section below) instead of the per-run workflow.

**Saving / loading configuration:**
- **File → Save Configuration** writes a `.config` file with all current settings.
- **File → Load Configuration** restores a previously saved configuration.
- **File → Write Configuration Template** creates a fully-commented `.config` template.

**Data export:**
- **File → Export CSV** exports the temperature log from the current session.
- **File → Export Picture** saves the temperature plot as PNG.
- After a successful experiment, all artifacts (CSV, JSON metadata, plots) are auto-saved under `./experiments/<session>/` or the configured auto-save directory.

**Experiment Sequence** (Edit → Sequence…)

Queue multiple `.config` files to run back-to-back as independent experiments. Open the dialog with **Edit → Sequence…**, then:

1. Add configs by dragging `.config` files from Explorer onto the list, or click **Add Configs…** to pick them. Each row uses `↑` / `↓` to reorder and `×` to remove.
2. Between every adjacent pair of configurations the dialog shows an **Interval** spin box in minutes (range `0..600`, default 5). Set each gap independently — e.g. 30 min between configs 1 and 2, 15 min between configs 2 and 3.
3. Optionally tick **Abort the rest of the sequence if a configuration fails** so a safety cutoff, hardware error, or watchdog force-stop ends the whole queue instead of advancing to the next config.
4. Optionally enable **Manual-stop PIN protection (sequence)** with a username and a 4-digit PIN. The sequence STOP button will then require the PIN before stopping.
5. Click **▶ Start Sequence**. The dialog can be closed while the sequence runs — the queue keeps executing, and reopening the dialog re-binds to the live state.

Notes:
- The main page **▶ START** and **■ EMERGENCY STOP** are locked out while a sequence is queued; use the dialog's buttons instead.
- The sequence has no preflight confirmation window. Each queued configuration is run as if started normally, and **each one writes its own timestamped log** under `./logs/` — there is no separate "sequence log" type.
- **Estimated total time** sums every config's expected duration plus every per-gap interval. It is shown as *unavailable* when any queued configuration uses *After-target Hold* mode, because the hold duration depends on a not-yet-known temperature event.
- The 2-second force-stop escalation and the stall watchdog still bypass the sequence PIN, so safety paths cannot be locked out.

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
  --interval-s X        Interval between bursts in seconds (default 5, min 0.2 — see "Interval floor")
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
  --temp-rate-window X  dT/dt smoothing window in seconds (default 30, range 1/sample-rate to 60)

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
[device]
device_kind          = sonocontrol_fpga  # sonocontrol_fpga | hyus

[hyus]
hyus_device_ip       = 192.168.0.10
hyus_pulse_len_us    = 160
hyus_pulse_period_us = 400
hyus_seq_len_ms      = 1
hyus_seq_period_ms   = 1000
hyus_run_mode        = 0
hyus_pid_var         = 1              # 0 = amplitude, 1 = pulse duty, 2 = sequence duty

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
temperature_required = false           # true = abort run on 3 consecutive invalid HH806AU reads
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
temp_rate_window_s  = 30.0   # dT/dt smoothing window (s); range 1/sampling_rate_hz to 60

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

[web_server]
web_server_port            = 50896
web_server_snapshot_interval_s = 900   # 5..3600 seconds
web_server_lan             = false     # true = bind all interfaces (LAN-reachable)
# web_server_enabled is intentionally NOT serialized — the server always
# starts disabled on app launch and must be turned on manually.
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

### Hyus LAN Device (TCP/UDP)

- Device mode is selected at GUI startup. Hyus mode uses its own PARAMS/CONNECT controls and does not upload the FPGA waveform table.
- The PC runs the TCP server on port `8192` and broadcasts the discovery message on UDP port `8193`.
- The device connects back to the PC and receives scalar parameter frames for carrier frequency, amplitude, pulse length/period, sequence length/period, run mode, total duration, trigger source, and RUN start/stop.
- Hyus Total Duration uses the device total-duration mode. Repeating cycles and After-target Hold are software-timed and stopped with `RUN=0`.
- After-target Hold in Hyus mode requires HH806AU temperature feedback, just like PID.

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
- The EMA coefficient (`α = 0.25`) that smooths the temperature *level* is compile-time constant, not configurable. Adjust in `src/experiment.cpp` if needed. (The separate dT/dt *slope* smoothing window **is** configurable — see `temp_rate_window_s` / `--temp-rate-window` / the PID tab's "Temp smoothing window", default 30 s.)
- Cycling mode is only available in Total Duration mode; it is silently disabled in Repeating or Hold-After-Target modes.
- Auto-save creates files only on clean completion (exit code 0). Manual stops and cutoff events do not trigger auto-save.
- An experiment sequence continues to the next queued configuration even if the current one ends with an error (e.g. watchdog force-stop or hardware error). To abort the whole sequence in that case, either tick **Abort the rest of the sequence if a configuration fails** in the Sequence dialog before starting, or click the sequence's **■ Stop Sequence** button manually. The safety cutoff fires per-configuration as usual.

### Monitoring web server (optional)

The CONNECT tab has a **Monitoring Web Server** group that exposes an optional, read-only HTTP endpoint for unattended monitoring. When enabled, the server serves:

- `GET /` — a self-contained HTML page (≈ 4 KB + JSON blob) with the latest cached temperature samples, elapsed / remaining time, run state, and a `<canvas>`-drawn line chart of T1 / T2 / Avg. All chart drawing happens in the browser — no external CDN, no server-side rendering.
- `GET /data.json` — the same payload as JSON for scripting / external tooling.

Behaviour:

- **Off by default at every app launch.** The enable flag is deliberately session-only — loading a `.config` file cannot silently turn the server on. Port, snapshot interval, and the LAN-bind flag *are* persisted in `.config`.
- **Default bind:** `127.0.0.1` (localhost only). Tick **Allow LAN access** to bind all interfaces — this makes the wireless NIC's LAN IP reachable. The status line then lists all bound URLs (`http://127.0.0.1:port/` plus every non-loopback IPv4 on an up interface). There is **no authentication and no HTTPS**, so only enable LAN access on trusted networks.
- **Snapshot interval:** 5..3600 s, default 900 (15 min). The server rebuilds the cached HTML+JSON every interval, AND immediately on key events (server start, run start, first sample after idle, run finish). A 5–10 s interval is appropriate for live-ish monitoring; longer intervals are fine for fire-and-forget multi-day runs.
- **Live buffer cap:** 500 most recent samples (~16 KB). Long-running experiments don't grow memory.
- **Robustness:** request header capped at 8 KiB, 5 s read timeout per connection, `GET`-only, query strings stripped before path matching, `Cache-Control: no-store`, `Connection: close`.

Compile-time: requires Qt's `Network` module. If the GUI was built against a Qt without `Network`, the group is replaced with an explanatory note and the feature is unavailable.

### Interval floor

`interval_time_s` is clamped to `max(duration_ms / 1000, 0.2 s)` in **every** entry point — the GUI spin box minimum, the GUI's `validate_config`, the CLI's `validate_config`, and the per-config validation that the sequence runner performs before launching each queued experiment. A `.config` file or `--interval-s 0.01` is therefore raised to the floor (the CLI prints a `[WARN]` line; the GUI silently clamps because the spin box doesn't let the value through in the first place). The 0.2 s floor reflects the fact that each transmit cycle issues a serial CFREQ/PRF/DUR triple — each followed by the hardware-required 100 ms COM gap — plus the 4096-datagram UDP waveform burst, and cannot physically complete faster than ~0.3 s on real hardware. The shared constant lives in `include/config.hpp` as `sonocontrol::kMinIntervalTimeS`.

---

## Development Notes

### Source layout

```
include/
  config.hpp          Config + ActiveParams structs; enums
  config_io.hpp       load/save/template/to_text declarations
  experiment.hpp      ExperimentRunner class + ExperimentCallbacks
  hyus_net.hpp        Hyus TCP server / UDP discovery wrapper
  hyus_protocol.hpp   Hyus frame builders and status parser
  logger.hpp          ExperimentLogger (streaming CSV + JSON metadata)
  pid.hpp             PIDController + apply_pid_to_params()
  protocol.hpp        Serial packet helpers; UDP burst builder; waveform arrays
  serial_port.hpp     Platform-abstracted serial port (Windows + POSIX)
  temperature.hpp     ITemperatureSensor; HH806AUSensor; TemperatureSimulator
  udp_socket.hpp      UdpSender wrapper

src/
  config_io.cpp
  experiment.cpp      Main experiment loop (state machine)
  hyus_net.cpp
  hyus_protocol.cpp
  logger.cpp
  pid.cpp
  protocol.cpp        DDS math; waveform table generators
  serial_port.cpp     Windows (CreateFile/ReadFile/WriteFile) + POSIX (termios)
  temperature.cpp     HH806AU frame parser; plausibility filter; physics simulator
  udp_socket.cpp
  main.cpp            CLI entry point + argument parser
  gui/
    device_dialog.hpp Startup device picker
    main_gui.cpp      Qt Widgets GUI (device modes, Sequence dialog, runners)
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
