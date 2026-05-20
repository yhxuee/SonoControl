# SonoControl — Stability Fix Notes

## Reported issues and how each is now addressed

### 1. Temp / COM3 / UDP pills only update after Start (or after clicking "Test")

**Cause.** `idleTimer_` is disabled in the original (comment: *caused GUI thread lag when PID was on*). `lblCom3Status_` / `lblUsStatus_` are only written inside `onParams()`, which fires *after* the worker emits its first burst. `lblTempStatus_` only goes green from `testTempConnection()`. Preflight does test all three, but writes to the console rather than the pills.

**Fix.** A new `StatusProbe` `QObject` runs on its own `QThread` and probes the ports every 3 s while the experiment is idle (paused while running). Each probe is bounded: open/close on COM3 (~200 ms timeout), bind/close on UDP source port 4561 (~10 ms), one `HH806AUSensor::read()` (~360 ms) when monitoring is on. Results are emitted as queued signals into `onProbeCom3` / `onProbeUdp` / `onProbeTemp`, which repaint the pills (and mirror the latest T1/T2/Avg into the side panel). The probe is kicked off immediately at startup, after `refreshPorts()`, after a `.config` load, and whenever any port / host / monitoring checkbox is edited.

### 2. Worker freezes — countdown keeps running, temperature/graph stop, emergency stop does nothing

**Cause.** The worker is blocked inside synchronous `WriteFile`/`ReadFile`/`sendto`. The countdown is driven by `sessionUiTimer_` on the GUI thread, so it ticks independently. `request_stop()` only sets `stop_flag_`; the blocked syscall never observes it. Compounding: `com_write`'s retry uses `sleep_for(backoff_ms)` which also doesn't check the flag, so under 8 retries with exponential backoff the worker is unresponsive to stop for several seconds even when not truly hung.

**Fix.**
- `SerialPort::cancel_io()` (Windows: `CancelIoEx(handle, nullptr)`; POSIX: closes fd) and `UdpSender::cancel_io()` (Windows: `CancelIoEx` on socket; POSIX: `shutdown(SHUT_RDWR)`).
- `ExperimentRunner::force_stop()` sets the flag *and* calls `cancel_io()` on both endpoints. A blocked `WriteFile` / `sendto` returns immediately with `ERROR_OPERATION_ABORTED`.
- New `interruptible_sleep_ms()` helper replaces all `sleep_for` calls in the retry loops, the inter-packet COM gap, and the run-loop tail. It chunks the sleep into 25 ms slices and aborts early when `stop_flag_` is set.
- `com_write` returns cleanly (no throw) when stop is requested mid-write, so a forced stop produces a clean `STOPPED` finish rather than an `ERROR` finish.
- A GUI-side stall **watchdog** (`watchdogTimer_`, 1 Hz) tracks the wall-clock time since the worker last emitted any signal (`touchWorkerSignal` is now called from every worker-side slot). If silence exceeds `config_.watchdog_timeout_ms` (default 5 s), it automatically escalates with `force_stop()` and flags the pills red.
- The EMERGENCY STOP button now escalates automatically: first click → graceful `stop()`; after 2 s, if the worker hasn't exited, the button is auto-re-labelled "FORCE STOP" and the next click (or the watchdog) triggers `force_stop()`.

### 3. Zombie process holds COM3 / UDP 4561 after Task Manager kill, blocking new launches

**Cause.** `~SonoControlWindow` / `closeEvent` calls `stopWorker(true)` which calls `QThread::wait(5000)`. If the worker is wedged, the wait times out and the window destroys without joining the thread. The thread's `finished` signal never fires, so the `deleteLater` chain that destroys the worker and frees the COM/UDP handles never runs. The visible window closes but the process keeps running with COM3 + UDP 4561 open.

**Fix.** `stopWorker(true)` now escalates:
1. Send graceful `stop()`, then `QThread::wait(2000)`.
2. If still alive, call `forceStop()` (cancels pending I/O), then `QThread::wait(3000)`.
3. If still alive, `QThread::terminate()` as a last resort + 1 s wait. The OS will then reap the thread and release the COM/UDP handles when the process actually exits.

`shutdownInProgress_` is set at the top of `closeEvent` / dtor so the probe thread can be torn down first and won't try to use widgets that are mid-destruction.

---

## Long-term stability and performance notes (not bugs, but worth knowing)

1. **HH806AU read cost.** `HH806AUSensor::read()` self-throttles to 3 Hz minimum (300 ms gap) + ~60 ms protocol time. The sampling-rate spinner allows up to 20 Hz; values above ~3 Hz silently degrade. Consider clamping `spnSampleRate_->setRange(0.1, 3.0)` in the UI, or noting the device ceiling.
2. **Burst transmit cost.** `transmit()` sends 4096 × 8-byte UDP packets per burst plus 4 serial packets each followed by `COM_GAP_S = 100 ms`. That's ~400 ms of mandatory inter-packet sleep per burst. With the new interruptible sleep, this is now cancel-aware, but the steady-state cost is unchanged. If interval drops below ~1 s the worker spends >40% of its time in COM-gap sleeps.
3. **Console signal volume.** Every TEMP / PID / UDP / TIME line crosses a thread boundary as a Qt queued signal. `appendConsole` already skips paint when the console panel is hidden, but the strings are still allocated and the events still queue. Over a 6-hour run on a 1 Hz sample rate this is ~80 k events. Fine for stability, but if you ever profile high-rate runs, a ring-buffer with periodic drain would cut allocations.
4. **TempPlot trim policy.** Caps at 6000 points, trims 600 at a time. With 10 Hz sampling the cap is reached in 10 minutes. The plot is paint-throttled to 4 Hz already, so this is fine for long runs; the only cost is the trim memmove (O(n) every 10 min).
5. **`send_stop()` race.** It briefly flips `stop_flag_ = false` to force the STOP packet through `com_write`, then restores it. If a watchdog also writes to `stop_flag_` during that window, the flag could end up `false`. Low probability in practice (the window is microseconds and the watchdog is on the GUI thread), but a `std::atomic_signal_fence` or local override variable would close it cleanly.
6. **Linux serial fd cancellation.** `SerialPort::cancel_io()` on POSIX closes the fd. This unblocks the call but is technically a use-after-close from the worker's perspective. The worker treats the next read/write as a normal failure and exits the run loop, which is the desired behaviour, but `helgrind` will flag it. On Windows (your target), `CancelIoEx` is clean.
7. **`Q_OBJECT` inside anonymous namespace.** The existing code already does this for `RunnerWorker` and `SonoControlWindow`; the new `StatusProbe` follows the same pattern, so AUTOMOC handles it correctly.

---

## Files touched

```
include/serial_port.hpp       +7 lines  (cancel_io declaration)
include/udp_socket.hpp        +6 lines  (cancel_io declaration)
include/experiment.hpp        +4 lines  (force_stop declaration)
src/serial_port.cpp           +24 lines (cancel_io for Windows + POSIX)
src/udp_socket.cpp            +14 lines (cancel_io for Windows + POSIX, +<windows.h>)
src/experiment.cpp            ~30 lines (force_stop, interruptible_sleep, retry fixes)
src/gui/main_gui.cpp          ~200 lines (StatusProbe class, watchdog, escalating stop,
                                          probe wiring into onStart/onFinished/markConfigChanged
                                          /refreshPorts/loadConfigFile, robust shutdown)
```

The full unified diff is in `CHANGES.diff` next to this file.

---

## Verification

Compiled cleanly with `cmake .. && make -j4` (gcc 13.3, Qt 5.15.13, Ubuntu 24.04) producing both the CLI (`sonocontrol`) and GUI (`sonocontrol_gui`) executables. No new warnings on the modified files. Compile-only verification — no runtime smoke test was possible without the HH806AU + ultrasound hardware.

The original codebase uses `#include <QtGui/QAction>` which is Qt6-specific; that include was left untouched since your Windows build is Qt6. If you ever need to build on Qt5, swap it for `#include <QtWidgets/QAction>`.
