#include "config.hpp"
#include "config_io.hpp"
#include "experiment.hpp"
#include "logger.hpp"
#include "protocol.hpp"
#include "temperature.hpp"
#include "serial_port.hpp"
#include "udp_socket.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QIODevice>
#include <QtCore/QMetaType>
#include <QtCore/QMutex>
#include <QtCore/QSignalBlocker>
#include <QtCore/QTime>
#include <QtCore/QFileInfo>
#include <QtCore/QUrl>
#include <QtCore/QStandardPaths>
#include <QtCore/QThread>
#include <QtCore/QRegularExpression>
#include <QtCore/QTimer>
#include <QtGui/QCloseEvent>
#include <QtGui/QDesktopServices>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QPixmap>
#include <QtGui/QPolygonF>
#include <QtGui/QTextOption>
#include <QtWidgets/QAbstractSpinBox>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtGui/QAction>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <windows.h>
#endif

#ifndef SONOCONTROL_DEBUG_SIM
#define SONOCONTROL_DEBUG_SIM 0
#endif

Q_DECLARE_METATYPE(sonocontrol::ActiveParams)
Q_DECLARE_METATYPE(std::vector<float>)

namespace {

bool debug_sim_build() {
#if SONOCONTROL_DEBUG_SIM
    return true;
#else
    return false;
#endif
}

QString fmt_time(double seconds) {
    int s = std::max(0, static_cast<int>(seconds));
    int h = s / 3600;
    int m = (s % 3600) / 60;
    int sec = s % 60;
    return QString("%1:%2:%3").arg(h, 2, 10, QChar('0')).arg(m, 2, 10, QChar('0')).arg(sec, 2, 10, QChar('0'));
}

QString qstr(const char* s) { return QString::fromUtf8(s); }
QString qstr(const std::string& s) { return QString::fromStdString(s); }

QString statusStyle(const QString& color) {
    // Compact status badge. Keep it readable without dominating the header.
    QString bg = "#fff4e5";
    if (color == "#1e8e3e") bg = "#e6f4ea";
    else if (color == "#d93025") bg = "#fce8e6";
    else if (color == "#667085") bg = "#f3f5f8";
    else if (color == "#e6930a" || color == "#b54708") bg = "#fff4e5";
    return QString("background:%1; color:%2; border:1px solid rgba(128,128,128,0.16); border-radius:7px; padding:2px 7px; font-weight:600; font-size:10px; letter-spacing:0.35px;").arg(bg, color);
}

void validate_config(sonocontrol::Config& c) {
    c.amplitude = std::clamp(c.amplitude, 0.0, 1.0);
    c.duty_cycle = std::clamp(c.duty_cycle, 0.0, 1.0);
    c.duration_ms = std::max(0, c.duration_ms);
    c.interval_time_s = std::max(c.interval_time_s, static_cast<double>(c.duration_ms) / 1000.0);
    c.sampling_rate_hz = std::max(0.1, c.sampling_rate_hz);
    c.total_duration_mins = std::max(0.0, c.total_duration_mins);
    c.repeating = std::max(1, c.repeating);
    c.hold_after_target_mins = std::max(0.0, c.hold_after_target_mins);
    c.target_tolerance_c = std::clamp(c.target_tolerance_c, 0.05, 5.0);
    c.use_total_duration = (c.length_mode != sonocontrol::LengthMode::RepeatingCycles);
    if (c.pid_enabled && c.length_mode == sonocontrol::LengthMode::RepeatingCycles) {
        c.length_mode = sonocontrol::LengthMode::TotalDuration;
        c.use_total_duration = true;
    }
    if (c.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
        c.temperature_enabled = true;
        c.use_cycling = false;
    }
    if (c.use_cycling && c.length_mode != sonocontrol::LengthMode::TotalDuration) {
        c.use_cycling = false;
    }
    c.udp_port = std::max<uint16_t>(1, c.udp_port);
    c.communication_retry_attempts = std::clamp(c.communication_retry_attempts, 1, 8);
    c.communication_retry_initial_backoff_ms = std::clamp(c.communication_retry_initial_backoff_ms, 20, 2000);
    c.emergency_stop_repeats = std::clamp(c.emergency_stop_repeats, 1, 10);
    c.watchdog_timeout_ms = std::clamp(c.watchdog_timeout_ms, 1000, 60000);
    c.pid_prediction_tau_s = std::clamp(c.pid_prediction_tau_s, 0.0, 3600.0);
    c.pid_prediction_horizon_s = std::clamp(c.pid_prediction_horizon_s, 0.0, 360.0);
    if (c.pid_enabled) {
        c.temperature_enabled = true;
    }
    if (!c.temperature_enabled) {
        c.simulate_temp = false;
    }
    if (!debug_sim_build()) {
        c.simulate_temp = false;
        c.simulate_us = false;
    }
}

namespace Theme {
    inline bool& dark_flag() { static bool b = false; return b; }
    inline bool isDark() { return dark_flag(); }
    inline void setDark(bool d) { dark_flag() = d; }

    // Color tokens that switch on the dark flag.
    inline QString bg()         { return isDark() ? "#0f172a" : "#f3f5f8"; }
    inline QString panel()      { return isDark() ? "#1e293b" : "#ffffff"; }
    inline QString panel2()     { return isDark() ? "#0f172a" : "#eef0f4"; }
    inline QString border()     { return isDark() ? "#334155" : "#e4e7ec"; }
    inline QString border2()    { return isDark() ? "#475569" : "#c0c8d4"; }
    inline QString text()       { return isDark() ? "#e2e8f0" : "#1a1a2e"; }
    inline QString text2()      { return isDark() ? "#94a3b8" : "#475467"; }
    inline QString text3()      { return isDark() ? "#64748b" : "#98a2b3"; }
    inline QString accent()     { return isDark() ? "#2dd4bf" : "#00897b"; }
    inline QString accent2()    { return isDark() ? "#5eead4" : "#00695c"; }
    inline QString chartBg()    { return isDark() ? "#0f172a" : "#ffffff"; }
    inline QString chartGrid()  { return isDark() ? "#1e293b" : "#e8eaee"; }
    inline QString chartAxis()  { return isDark() ? "#475569" : "#d0d5dd"; }
    inline QString chartLabel() { return isDark() ? "#94a3b8" : "#475467"; }
    inline QString setpointColor() { return "#f59e0b"; }
    inline QString cutoffColor()   { return "#ef4444"; }
    inline QString t1Color()       { return isDark() ? "#60a5fa" : "#1a73e8"; }
    inline QString t2Color()       { return isDark() ? "#f87171" : "#ea4335"; }
}

class TemperaturePlot final : public QWidget {
public:
    explicit TemperaturePlot(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(220);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        // Throttle repaints to 10 Hz max — keeps GUI responsive at high sample rates
        repaintTimer_.setSingleShot(true);
        repaintTimer_.setInterval(250);
        QObject::connect(&repaintTimer_, &QTimer::timeout, this, [this]{ update(); });
    }

    void clear() {
        times_.clear(); avg_.clear(); t1_.clear(); t2_.clear(); update();
    }
    void setLines(double setpoint, double cutoff) { setpoint_ = setpoint; cutoff_ = cutoff; scheduleRepaint(); }
    void append(double t, double t1, double t2, double avg) {
        times_.push_back(t); t1_.push_back(t1); t2_.push_back(t2); avg_.push_back(avg);
        constexpr size_t kMaxPlotPoints = 6000;
        constexpr size_t kTrimPlotPoints = 600;
        if (times_.size() > kMaxPlotPoints) {
            times_.erase(times_.begin(), times_.begin() + kTrimPlotPoints);
            t1_.erase(t1_.begin(), t1_.begin() + kTrimPlotPoints);
            t2_.erase(t2_.begin(), t2_.begin() + kTrimPlotPoints);
            avg_.erase(avg_.begin(), avg_.begin() + kTrimPlotPoints);
        }
        scheduleRepaint();
    }
private:
    QTimer repaintTimer_;
    void scheduleRepaint() {
        if (!repaintTimer_.isActive()) repaintTimer_.start();
    }
public:

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.fillRect(rect(), QColor(Theme::chartBg()));
        p.setRenderHint(QPainter::Antialiasing, false);
        p.setRenderHint(QPainter::TextAntialiasing, true);
        const QRectF r = rect().adjusted(64, 26, -20, -48);
        p.setPen(QPen(QColor(Theme::chartAxis()), 1));
        p.drawRect(r);
        p.setPen(QPen(QColor(Theme::chartGrid()), 1));
        for (int i = 1; i < 5; ++i) {
            const double y = r.top() + r.height() * i / 5.0;
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        }
        p.setPen(QColor(Theme::chartLabel()));
        p.drawText(QRectF(4, r.top() - 22, 150, 20), Qt::AlignLeft, QString::fromUtf8("Temperature (°C)"));
        p.drawText(QRectF(r.left(), r.bottom() + 18, r.width(), 22), Qt::AlignCenter, "Elapsed Time (s)");

        const double xmax = times_.empty() ? 60.0 : std::max(60.0, times_.back() * 1.05);
        double ymin = 35.0, ymax = 45.0;
        if (!avg_.empty()) {
            auto [mn, mx] = std::minmax_element(avg_.begin(), avg_.end());
            ymin = std::min(35.0, std::floor(*mn - 1.0));
            ymax = std::max(45.0, std::ceil(*mx + 1.0));
            if (ymax - ymin < 4.0) { ymax += 2.0; ymin -= 2.0; }
        }
        auto mapx = [&](double x) { return r.left() + r.width() * (x / xmax); };
        auto mapy = [&](double y) { return r.bottom() - r.height() * ((y - ymin) / (ymax - ymin)); };

        p.setPen(QColor(Theme::chartLabel()));
        for (int i = 0; i <= 5; ++i) {
            const double val = ymin + (ymax - ymin) * i / 5.0;
            const double y = mapy(val);
            p.drawLine(QPointF(r.left() - 4, y), QPointF(r.left(), y));
            p.drawText(QRectF(2, y - 9, 56, 18), Qt::AlignRight, QString::number(val, 'f', 1));
        }
        for (int i = 0; i <= 4; ++i) {
            const double val = xmax * i / 4.0;
            const double x = mapx(val);
            p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() + 4));
            p.drawText(QRectF(x - 36, r.bottom() + 2, 72, 18), Qt::AlignCenter, QString::number(val, 'f', 0));
        }

        auto hline = [&](double value, QColor c, Qt::PenStyle style) {
            if (value < ymin || value > ymax) return;
            QPen pen(c, 1); pen.setStyle(style); p.setPen(pen);
            const double y = mapy(value);
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        };
        hline(setpoint_, QColor(Theme::setpointColor()), Qt::DashLine);
        hline(cutoff_, QColor(Theme::cutoffColor()), Qt::DotLine);

        auto drawSeries = [&](const std::vector<double>& ys, QColor c, int width) {
            if (times_.size() < 2 || ys.size() != times_.size()) return;
            p.setPen(QPen(c, width));
            // Draw as separate line segments, skipping NaN points so a
            // disconnected channel doesn't render a spurious line.
            QPolygonF poly;
            const size_t step = std::max<size_t>(1, times_.size() / 1200);
            for (size_t i = 0; i < times_.size(); i += step) {
                if (std::isnan(ys[i])) {
                    if (poly.size() > 1) p.drawPolyline(poly);
                    poly.clear();
                    continue;
                }
                poly << QPointF(mapx(times_[i]), mapy(ys[i]));
            }
            if (poly.size() > 1) p.drawPolyline(poly);
        };
        drawSeries(t1_, QColor(Theme::t1Color()), 1);
        drawSeries(t2_, QColor(Theme::t2Color()), 1);
        drawSeries(avg_, QColor(Theme::accent()), 3);

        p.setPen(QColor(Theme::chartLabel()));
        p.drawText(QRectF(r.right() - 220, r.top() + 6, 210, 24), Qt::AlignRight, "T1     T2     Avg");
    }

private:
    std::vector<double> times_, avg_, t1_, t2_;
    double setpoint_ = 40.0;
    double cutoff_ = 45.0;
};

class WaveformPlot final : public QWidget {
public:
    explicit WaveformPlot(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(150);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        wfTimer_.setSingleShot(true);
        wfTimer_.setInterval(250);
        QObject::connect(&wfTimer_, &QTimer::timeout, this, [this]{ update(); });
    }
    void setWaveform(std::vector<float> data) {
        data_ = std::move(data);
        if (!wfTimer_.isActive()) wfTimer_.start();
    }
private:
    QTimer wfTimer_;
public:
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, false);
        p.setRenderHint(QPainter::TextAntialiasing, true);
        p.fillRect(rect(), QColor(Theme::chartBg()));
        const QRectF r = rect().adjusted(64, 22, -20, -42);
        p.setPen(QPen(QColor(Theme::chartAxis()), 1));
        p.drawRect(r);
        p.setPen(QPen(QColor(Theme::chartGrid()), 1));
        for (int i = 1; i < 4; ++i) {
            const double y = r.top() + r.height() * i / 4.0;
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        }
        for (int i = 1; i < 4; ++i) {
            const double x = r.left() + r.width() * i / 4.0;
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        }
        p.setPen(QColor(Theme::chartLabel()));
        p.drawText(QRectF(r.left(), r.bottom() + 18, r.width(), 22), Qt::AlignCenter, "Sample Index");
        p.drawText(QRectF(4, r.top() - 20, 140, 18), Qt::AlignLeft, "Amplitude (0–1)");
        p.setPen(QColor(Theme::chartLabel()));
        for (int i = 0; i <= 4; ++i) {
            const double v = i / 4.0;
            const double y = r.bottom() - r.height() * ((v + 0.05) / 1.15);
            p.drawLine(QPointF(r.left() - 4, y), QPointF(r.left(), y));
            p.drawText(QRectF(8, y - 9, 50, 18), Qt::AlignRight, QString::number(v, 'f', 2));
        }
        for (int i = 0; i <= 4; ++i) {
            const double idx = (data_.empty() ? 4095.0 : static_cast<double>(data_.size() - 1)) * i / 4.0;
            const double x = r.left() + r.width() * i / 4.0;
            p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() + 4));
            p.drawText(QRectF(x - 42, r.bottom() + 2, 84, 18), Qt::AlignCenter, QString::number(idx, 'f', 0));
        }
        if (data_.size() < 2) return;
        QPolygonF poly;
        const size_t step = std::max<size_t>(1, data_.size() / 1400);
        for (size_t i = 0; i < data_.size(); i += step) {
            const double x = r.left() + r.width() * static_cast<double>(i) / static_cast<double>(data_.size() - 1);
            const double y = r.bottom() - r.height() * ((std::clamp<double>(data_[i], -0.05, 1.1) + 0.05) / 1.15);
            poly << QPointF(x, y);
        }
        p.setPen(QPen(QColor(Theme::accent()), 2));
        p.drawPolyline(poly);
    }
private:
    std::vector<float> data_;
};

class RunnerWorker final : public QObject {
    Q_OBJECT
public:
    RunnerWorker(sonocontrol::Config config, sonocontrol::ExperimentLogger* logger)
        : config_(std::move(config)), logger_(logger) {}

public slots:
    void run() {
        try {
            std::unique_ptr<sonocontrol::ITemperatureSensor> sensor;
            if (!config_.temperature_enabled) sensor = std::make_unique<sonocontrol::NullTemperatureSensor>();
            else if (config_.simulate_temp) sensor = std::make_unique<sonocontrol::TemperatureSimulator>();
            else sensor = std::make_unique<sonocontrol::HH806AUSensor>(config_.com11_port, config_.min_plausible_temp_c, config_.max_plausible_temp_c, config_.max_temp_rate_c_per_s);

            sonocontrol::ExperimentCallbacks cb;
            cb.console = [this](const std::string& s) { emit console(qstr(s)); };
            cb.temperature = [this](double t1, double t2) { emit temperature(t1, t2); };
            cb.avg_temp = [this](double t) { emit avgTemp(t); };
            cb.params = [this](const sonocontrol::ActiveParams& p) { emit params(p); };
            cb.time = [this](double e, double r) { emit timeUpdate(e, r); };
            cb.cycle = [this](const std::string& s) { emit cycle(qstr(s)); };
            cb.waveform = [this](const std::vector<float>& w) { emit waveform(w); };
            cb.cutoff = [this](double t) { emit cutoff(t); };
            cb.error = [this](const std::string& s) { emit error(qstr(s)); };

            runner_ = std::make_unique<sonocontrol::ExperimentRunner>(config_, std::move(sensor), *logger_, std::move(cb));
            runnerRaw_.store(runner_.get(), std::memory_order_release);
            const int code = runner_->run();
            runnerRaw_.store(nullptr, std::memory_order_release);
            runner_.reset();
            emit finished(code);
        } catch (const std::exception& e) {
            if (logger_) logger_->log_error(e.what());
            if (runner_) runner_->emergency_stop_noexcept();
            runnerRaw_.store(nullptr, std::memory_order_release);
            runner_.reset();
            emit error(qstr(e.what()));
            emit finished(1);
        }
    }

    void stop() {
        if (auto* r = runnerRaw_.load(std::memory_order_acquire)) r->request_stop();
    }

    // Aggressive variant: also cancels any pending serial/UDP I/O so a wedged
    // worker thread unblocks immediately. Called by the EMERGENCY STOP button
    // (after a graceful stop request) and by the stall watchdog.
    void forceStop() {
        if (auto* r = runnerRaw_.load(std::memory_order_acquire)) r->force_stop();
    }

signals:
    void console(QString);
    void temperature(double, double);
    void avgTemp(double);
    void params(sonocontrol::ActiveParams);
    void timeUpdate(double, double);
    void cycle(QString);
    void waveform(std::vector<float>);
    void cutoff(double);
    void error(QString);
    void finished(int);

private:
    sonocontrol::Config config_;
    sonocontrol::ExperimentLogger* logger_ = nullptr;
    std::unique_ptr<sonocontrol::ExperimentRunner> runner_;
    std::atomic<sonocontrol::ExperimentRunner*> runnerRaw_{nullptr};
};

// Lightweight background probe that updates the COM3 / UDP / Temp status pills
// in the header before the experiment starts. Runs on its own QThread so a slow
// HH806AU read (~360 ms) or a stuck serial open does not stall the GUI thread.
// Paused while an experiment is running — the runner owns the ports then.
class StatusProbe final : public QObject {
    Q_OBJECT
public:
    struct Settings {
        QString com3_port;
        QString com11_port;
        QString udp_host;
        uint16_t udp_source_port = 4561;
        bool check_temperature = false;
        double temp_min_c = 10.0;
        double temp_max_c = 80.0;
        double temp_max_rate = 15.0;
    };

    // Thread-safe writer. Called from the GUI thread whenever the user edits a
    // port / host / monitoring checkbox. The probe loop picks up the new
    // values on its next iteration. Calling this is *not* a Qt signal/slot
    // emit, just a regular method call — the mutex is what makes it safe.
    void updateSettings(const Settings& s) {
        QMutexLocker lk(&mutex_);
        settings_ = s;
    }

    void setPaused(bool paused) {
        paused_ = paused;
        if (!paused) {
            // Kick a probe immediately when resumed so the pills update without
            // waiting for the next periodic tick.
            QMetaObject::invokeMethod(this, "runOnce", Qt::QueuedConnection);
        }
    }

public slots:
    void runOnce() {
        if (paused_) return;
        Settings s;
        { QMutexLocker lk(&mutex_); s = settings_; }

        // COM3 — fast open/close. If the port is missing or in use by another
        // process the open fails immediately; if it succeeds we know the OS
        // will let us own it at start.
        if (!s.com3_port.isEmpty() && !s.com3_port.contains("no ports", Qt::CaseInsensitive)) {
            sonocontrol::SerialPort port;
            const bool ok = port.open(s.com3_port.toStdString(), 9600, 8, 'N', 1, 200);
            port.close();
            emit com3Status(ok);
        } else {
            emit com3Status(false);
        }
        if (paused_) return;

        // UDP — try to bind the source port. If another instance of this
        // program leaked a process, the bind fails here and the user sees the
        // pill stay red, which is the early-warning we want.
        {
            sonocontrol::UdpSender udp;
            const bool ok = udp.open(s.udp_source_port);
            udp.close();
            emit udpStatus(ok);
        }
        if (paused_) return;

        // Temperature — only when monitoring is enabled, since opening the
        // HH806AU port and waiting ~360 ms otherwise costs a probe cycle for
        // nothing.
        if (s.check_temperature && !s.com11_port.isEmpty() &&
            !s.com11_port.contains("no ports", Qt::CaseInsensitive)) {
            try {
                sonocontrol::HH806AUSensor sensor(s.com11_port.toStdString(),
                                                  s.temp_min_c, s.temp_max_c, s.temp_max_rate);
                auto pair = sensor.read();
                const bool any = pair.first.has_value() || pair.second.has_value();
                emit tempStatus(any,
                                pair.first.value_or(std::numeric_limits<double>::quiet_NaN()),
                                pair.second.value_or(std::numeric_limits<double>::quiet_NaN()));
            } catch (...) {
                emit tempStatus(false,
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN());
            }
        } else {
            emit tempStatus(false,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN());
        }
    }

signals:
    void com3Status(bool ok);
    void udpStatus(bool ok);
    // ok=true when at least one channel returned a valid reading.
    void tempStatus(bool ok, double t1, double t2);

private:
    QMutex mutex_;
    Settings settings_;
    std::atomic<bool> paused_{false};
};

class SonoControlWindow final : public QMainWindow {
    Q_OBJECT
public:
    explicit SonoControlWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        setWindowTitle("SonoControl  ·  Ultrasound Temperature Controller");
        setMinimumSize(1600, 900);
        // Auto-detect theme by time of day: dark from 17:00 to 08:00.
        Theme::setDark(isNightTime());
        setStyleSheet(styleSheetText());
        statusBar()->showMessage("Ready");
        buildUi();
        refreshPorts();
        updateRepeatingFromDuration();
        onPidChanged(false);
        // Idle timer disabled — no background sensor polling.

        // Theme timer: check every 5 minutes; auto-switch if enabled.
        themeTimer_.setInterval(5 * 60 * 1000);
        connect(&themeTimer_, &QTimer::timeout, this, &SonoControlWindow::checkThemeByTime);
        themeTimer_.start();

        // The session timer is intentionally independent from the worker thread.
        // Ultrasound transmission can briefly block the worker; the wall-clock UI must still update smoothly.
        sessionUiTimer_.setInterval(250);
        connect(&sessionUiTimer_, &QTimer::timeout, this, &SonoControlWindow::refreshSessionTimerUi);
        installConfigChangeTracking();

        // Status probe runs in its own thread so a slow HH806AU read or a
        // wedged serial open can't freeze the UI. It updates the header pills
        // periodically while the experiment is idle, and pauses while a run
        // is in progress (the runner owns the ports then).
        setupStatusProbe();

        // Stall watchdog. Independent of the experiment worker. Fires if no
        // worker-side signal arrives for `watchdog_timeout_ms` while running,
        // which is the signature of a USB-serial driver hang (countdown keeps
        // ticking on the UI thread, but temperature, params, and graph stop
        // updating). When that happens, escalate the emergency stop with a
        // force_stop that cancels pending I/O so the worker can exit.
        watchdogTimer_.setInterval(1000);
        connect(&watchdogTimer_, &QTimer::timeout, this, &SonoControlWindow::checkWorkerProgress);
    }

    ~SonoControlWindow() override {
        shutdownInProgress_ = true;
        teardownStatusProbe();
        stopWorker(true);
        deleteTemporaryConfigFile();
    }

protected:
    void closeEvent(QCloseEvent* e) override {
        shutdownInProgress_ = true;
        teardownStatusProbe();
        stopWorker(true);
        deleteTemporaryConfigFile();
        QMainWindow::closeEvent(e);
    }

private slots:
    void onStart() {
        if (running_) return;
        config_ = buildConfig();
        attachActiveConfigProvenance(config_);
        validate_config(config_);
        if (!preflightCheck(config_)) return;
        fatalErrorShown_ = false;
        running_ = true;
        targetHoldMode_ = (config_.length_mode == sonocontrol::LengthMode::HoldAfterTarget);
        targetHoldStarted_ = false;
        targetHoldStartWallS_ = 0.0;
        targetHoldTotalS_ = std::max(0.0, config_.hold_after_target_mins * 60.0);
        plannedTotalS_ = (config_.length_mode == sonocontrol::LengthMode::TotalDuration)
            ? config_.total_duration_mins * 60.0
            : ((config_.length_mode == sonocontrol::LengthMode::RepeatingCycles)
                ? static_cast<double>(config_.repeating) * config_.interval_time_s
                : targetHoldTotalS_);
        startWall_.start();
        refreshSessionTimerUi();
        sessionUiTimer_.start();
        tempPlot_->clear();
        tempPlot_->setLines(spnSetpoint_->value(), spnCutoff_->value());
        idleTimer_.stop();
        btnStart_->setEnabled(false);
        btnStop_->setEnabled(true);
        // Reset the emergency-stop button to its graceful state for this run.
        btnStop_->setText(" ■  EMERGENCY STOP ");
        lblCycle_->setText("IDLE");
        appendConsole("=== Experiment started ===");
        statusBar()->showMessage("Experiment running");

        // Hand the ports over to the worker; pause idle probing so we don't
        // race for COM3 / HH806AU. The probe will resume when running_ flips
        // back to false in onFinished().
        if (probe_) probe_->setPaused(true);

        // Initialize the watchdog timestamp and start the timer. The watchdog
        // is independent of the experiment thread — it lives on the GUI
        // thread and checks elapsed time since the last worker signal.
        lastWorkerSignalMs_ = QDateTime::currentMSecsSinceEpoch();
        workerStallNotified_ = false;
        watchdogTimer_.start();

        workerThread_ = new QThread(this);
        worker_ = new RunnerWorker(config_, &logger_);
        worker_->moveToThread(workerThread_);
        connect(workerThread_, &QThread::started, worker_, &RunnerWorker::run);
        connect(worker_, &RunnerWorker::console, this, &SonoControlWindow::appendConsole);
        connect(worker_, &RunnerWorker::temperature, this, &SonoControlWindow::onTemperature);
        connect(worker_, &RunnerWorker::params, this, &SonoControlWindow::onParams);
        connect(worker_, &RunnerWorker::timeUpdate, this, &SonoControlWindow::onTimeUpdate);
        connect(worker_, &RunnerWorker::cycle, this, &SonoControlWindow::onCycle);
        connect(worker_, &RunnerWorker::waveform, this, &SonoControlWindow::onWaveform);
        connect(worker_, &RunnerWorker::cutoff, this, &SonoControlWindow::onCutoff);
        connect(worker_, &RunnerWorker::error, this, &SonoControlWindow::onError);
        connect(worker_, &RunnerWorker::finished, this, &SonoControlWindow::onFinished);
        connect(worker_, &RunnerWorker::finished, workerThread_, &QThread::quit);
        connect(workerThread_, &QThread::finished, worker_, &QObject::deleteLater);
        connect(workerThread_, &QThread::finished, workerThread_, &QObject::deleteLater);
        workerThread_->start();
    }

    void onStop() {
        appendConsole(">>> EMERGENCY STOP requested");
        statusBar()->showMessage("Emergency stop requested");
        if (!worker_) return;
        worker_->stop();
        // Visually mark the button as already pressed and disable a second
        // graceful click — the next click should be the user explicitly
        // asking for force-stop after they see the runner isn't responding.
        btnStop_->setEnabled(false);
        // Schedule an automatic escalation: if the worker hasn't exited
        // gracefully within 2 seconds, cancel pending I/O so a wedged
        // WriteFile/sendto unblocks and the worker can exit. The button
        // is re-enabled with a "Force Stop" label so the user can also
        // trigger this manually if they don't want to wait.
        QTimer::singleShot(2000, this, [this]() {
            if (!running_ || !worker_) return;
            appendConsole(">>> EMERGENCY STOP escalating to force-stop (cancelling pending I/O)");
            statusBar()->showMessage("Force-stop: cancelling pending serial/UDP I/O");
            worker_->forceStop();
            btnStop_->setText(" ■  FORCE STOP ");
            btnStop_->setEnabled(true);
        });
    }

    // Called when the user clicks the EMERGENCY STOP button after the
    // graceful attempt timed out — at this point we go straight to force_stop.
    void onForceStop() {
        if (!worker_) return;
        appendConsole(">>> User requested force-stop");
        worker_->forceStop();
        btnStop_->setEnabled(false);
    }

    void loadConfigFile() {
        if (running_) return;
        const QString path = QFileDialog::getOpenFileName(this, "Load SonoControl config", QString(), "SonoControl Config (*.config);;All Files (*)");
        if (path.isEmpty()) return;
        try {
            auto cfg = sonocontrol::load_config_file(path.toStdString());
            validate_config(cfg);
            loadingConfigUi_ = true;
            applyConfigToUi(cfg);
            loadingConfigUi_ = false;
            deleteTemporaryConfigFile();
            loadedConfigPath_ = path;
            activeConfigPath_ = path;
            activeConfigType_ = "loaded-config";
            updateConfigStatus();
            pushProbeSettings();
            appendConsole("Loaded config: " + path);
        } catch (const std::exception& e) {
            loadingConfigUi_ = false;
            QMessageBox::critical(this, "Load Config Failed", qstr(e.what()));
        }
    }

    void saveConfigFile() {
        if (running_) return;
        QString suggested = loadedConfigPath_.isEmpty() ? QString("sonocontrol_config.config") : loadedConfigPath_;
        const QString path = QFileDialog::getSaveFileName(this, "Save SonoControl config", suggested, "SonoControl Config (*.config);;All Files (*)");
        if (path.isEmpty()) return;
        try {
            auto cfg = buildConfig();
            validate_config(cfg);
            cfg.config_source_type = "saved-config";
            cfg.config_file_path = path.toStdString();
            sonocontrol::save_config_file(path.toStdString(), cfg, true);
            deleteTemporaryConfigFile();
            loadedConfigPath_ = path;
            activeConfigPath_ = path;
            activeConfigType_ = "saved-config";
            updateConfigStatus();
            appendConsole("Saved config: " + path);
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Save Config Failed", qstr(e.what()));
        }
    }

    void writeConfigTemplate() {
        const QString path = QFileDialog::getSaveFileName(this, "Write template .config", "sonocontrol_config_template.config", "SonoControl Config (*.config);;All Files (*)");
        if (path.isEmpty()) return;
        try {
            sonocontrol::write_config_template(path.toStdString());
            appendConsole("Wrote config template: " + path);
            QMessageBox::information(this, "Template Written", "Template written to:\n" + path);
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Template Failed", qstr(e.what()));
        }
    }


    void onFinished(int code) {
        refreshSessionTimerUi();
        sessionUiTimer_.stop();
        watchdogTimer_.stop();
        workerStallNotified_ = false;
        running_ = false;
        worker_ = nullptr;
        workerThread_ = nullptr;
        btnStart_->setEnabled(true);
        btnStop_->setEnabled(false);
        // Restore the EMERGENCY STOP label for the next run, in case a
        // force-stop escalation re-labelled it during this one.
        btnStop_->setText(" ■  EMERGENCY STOP ");
        // Resume idle probing now that the worker has released the ports.
        // setPaused(false) also kicks off a probe immediately so the pills
        // reflect the post-run state without waiting for the next tick.
        if (probe_ && !shutdownInProgress_) probe_->setPaused(false);
        lblCycle_->setText(code == 0 ? "COMPLETE" : (code == 2 ? "CUTOFF" : (code == 3 ? "STOPPED" : "ERROR")));
        lblCycle_->setStyleSheet(valueStyle(code == 0 ? "#00897b" : (code == 3 ? "#b54708" : (code == 2 ? "#d93025" : "#b42318"))));
        if (code == 0) {
            appendConsole("=== Experiment complete ===");
            statusBar()->showMessage("Experiment complete");
        } else if (code == 2) {
            appendConsole("=== Experiment stopped by safety cutoff ===");
            statusBar()->showMessage("Safety cutoff");
        } else if (code == 3) {
            appendConsole("=== Experiment stopped manually ===");
            statusBar()->showMessage("Manual emergency stop");
        } else {
            appendConsole("=== Experiment aborted because of an error ===");
            statusBar()->showMessage("Experiment aborted: error");
        }
        lastAutoSaveDir_.clear();
        if (code == 0) {
            lastAutoSaveDir_ = autosaveExperimentArtifacts();
        }
        showExperimentSummary(code);
        // idleTimer_ no longer used
    }

    void onTemperature(double t1, double t2) {
        touchWorkerSignal();
        const bool has_t1 = (t1 > 0);
        const bool has_t2 = (t2 > 0);
        lblT1_->setText(has_t1 ? QString::number(t1, 'f', 1) + QString::fromUtf8("°C") : "N/C");
        lblT2_->setText(has_t2 ? QString::number(t2, 'f', 1) + QString::fromUtf8("°C") : "N/C");

        // Compute the "PID reference" value based on user's channel choice.
        // This is what gets shown as AVG and fed to the plot/PID.
        double ref = 0.0;
        bool ref_valid = false;
        const int ch = cmbTempChannel_ ? cmbTempChannel_->currentIndex() : 2;  // 0=T1, 1=T2, 2=Avg
        if (ch == 0) {
            ref_valid = has_t1;
            ref = has_t1 ? t1 : 0.0;
        } else if (ch == 1) {
            ref_valid = has_t2;
            ref = has_t2 ? t2 : 0.0;
        } else {
            // Average — but only if BOTH channels have readings.
            // If only one is connected, AVG falls back to that single value
            // so the chart and PID still get usable data.
            if (has_t1 && has_t2) { ref = (t1 + t2) / 2.0; ref_valid = true; }
            else if (has_t1)      { ref = t1;             ref_valid = true; }
            else if (has_t2)      { ref = t2;             ref_valid = true; }
        }

        if (ref_valid) {
            lblTavg_->setText(QString::number(ref, 'f', 2) + QString::fromUtf8("°C"));
            if (running_ && startWall_.isValid()) {
                tempPlot_->append(startWall_.elapsed() / 1000.0,
                                  has_t1 ? t1 : std::numeric_limits<double>::quiet_NaN(),
                                  has_t2 ? t2 : std::numeric_limits<double>::quiet_NaN(),
                                  ref);
            }
        } else {
            lblTavg_->setText("N/C");
        }
    }

    void onParams(sonocontrol::ActiveParams p) {
        touchWorkerSignal();
        lblCurAmp_->setText(QString::number(p.amplitude, 'f', 3));
        lblCurCfreq_->setText(QString::number(p.cfreq_hz / 1000.0, 'f', 1) + " kHz");
        lblCurPrf_->setText(QString::number(p.prf_hz, 'f', 0) + " Hz");
        lblCurDuty_->setText(QString::number(p.duty_cycle * 100.0, 'f', 1) + "%");
        lblCurDur_->setText(QString::number(p.duration_ms) + " ms");
        lblCurIntv_->setText(QString::number(p.interval_time_s, 'f', 1) + " s");
        lblUsStatus_->setStyleSheet(statusStyle("#1e8e3e"));
        lblCom3Status_->setStyleSheet(statusStyle("#1e8e3e"));
    }

    void onTimeUpdate(double elapsed, double remaining) {
        touchWorkerSignal();
        // Worker-thread time updates are kept for CLI/log compatibility,
        // but the GUI display is driven by sessionUiTimer_ to avoid visible stalls
        // when hardware transmission or retries briefly block the experiment thread.
        if (!sessionUiTimer_.isActive()) {
            lblElapsed_->setText(fmt_time(elapsed));
            lblRemaining_->setText(fmt_time(remaining));
        }
    }

    void refreshSessionTimerUi() {
        if (!startWall_.isValid()) return;
        const double elapsed = startWall_.elapsed() / 1000.0;
        double remaining = std::max(0.0, plannedTotalS_ - elapsed);
        if (targetHoldMode_) {
            if (targetHoldStarted_) remaining = std::max(0.0, targetHoldTotalS_ - (elapsed - targetHoldStartWallS_));
            else remaining = targetHoldTotalS_;
        }
        lblElapsed_->setText(fmt_time(elapsed));
        lblRemaining_->setText(targetHoldMode_ && !targetHoldStarted_ ? QString("wait target") : fmt_time(remaining));
    }

    void onCycle(const QString& phase) {
        touchWorkerSignal();
        if (targetHoldMode_ && phase.contains("TARGET HOLD") && !targetHoldStarted_) {
            targetHoldStarted_ = true;
            targetHoldStartWallS_ = startWall_.isValid() ? startWall_.elapsed() / 1000.0 : 0.0;
        }
        lblCycle_->setText(phase);
        lblCycle_->setStyleSheet(valueStyle(phase.contains("HEAT") || phase.contains("TARGET") ? "#0a9e8a" : "#5a6070"));
    }

    void onWaveform(std::vector<float> w) { touchWorkerSignal(); waveformPlot_->setWaveform(std::move(w)); }

    void onCutoff(double temp) {
        fatalErrorShown_ = true;
        QMessageBox::critical(this, "Safety Cutoff",
                              QString("Temperature %1°C exceeded cutoff %2°C.\nUltrasound stopped immediately and the experiment was terminated.")
                                  .arg(temp, 0, 'f', 2).arg(spnCutoff_->value(), 0, 'f', 1));
    }

    void onError(const QString& msg) {
        appendConsole("[ERROR] " + msg);
        statusBar()->showMessage("Error: " + msg.left(120));
        lblCycle_->setText("ERROR");
        lblCycle_->setStyleSheet(valueStyle("#b42318"));
        lblCom3Status_->setStyleSheet(statusStyle("#d93025"));
        lblUsStatus_->setStyleSheet(statusStyle("#d93025"));
        if (config_.temperature_enabled) lblTempStatus_->setStyleSheet(statusStyle("#d93025"));
        else lblTempStatus_->setStyleSheet(statusStyle("#667085"));
        if (worker_) worker_->stop();
        if (!fatalErrorShown_) {
            fatalErrorShown_ = true;
            QMessageBox::critical(this, "Experiment Error",
                                  "The experiment was stopped.\n\n" + msg +
                                  "\n\nCheck serial ports, sensor power, UDP host/port, and device connections before restarting.");
        }
    }

    void appendConsole(const QString& msg) {
        // Console messages count as worker progress even when the console
        // panel is hidden: a still-talking worker is a still-alive worker.
        touchWorkerSignal();
        if (!consoleFrame_->isVisible()) return;   // skip when hidden — major perf win
        console_->appendPlainText("[" + QTime::currentTime().toString("HH:mm:ss") + "] " + msg);
    }

    void updateRepeatingFromDuration() {
        const double interval = spnInterval_->value();
        if (interval <= 0.0 || !spnRepeating_) return;
        const int reps = std::max(1, static_cast<int>((spnTotalDur_->value() * 60.0) / interval));
        QSignalBlocker b(spnRepeating_);
        spnRepeating_->setValue(reps);
        updateLengthModeUi();
    }

    void updateDurationFromRepeating() {
        if (!rbRepeating_ || !rbRepeating_->isChecked()) return;
        QSignalBlocker b(spnTotalDur_);
        spnTotalDur_->setValue((spnRepeating_->value() * spnInterval_->value()) / 60.0);
    }

    void updateLengthModeUi() {
        const bool pid = chkPid_ && chkPid_->isChecked();
        if (pid && rbRepeating_ && rbRepeating_->isChecked() && rbTotalDur_) {
            QSignalBlocker b(rbTotalDur_);
            rbTotalDur_->setChecked(true);
        }
        const bool totalMode = rbTotalDur_ && rbTotalDur_->isChecked();
        const bool targetMode = rbTargetHold_ && rbTargetHold_->isChecked();
        if (rbRepeating_) rbRepeating_->setEnabled(!pid);
        if (spnRepeating_) spnRepeating_->setEnabled(!pid && rbRepeating_->isChecked());
        if (spnTotalDur_) spnTotalDur_->setEnabled(totalMode);
        if (spnTargetHoldMin_) spnTargetHoldMin_->setEnabled(targetMode);
        if (spnTargetTol_) spnTargetTol_->setEnabled(targetMode);
        if (chkCycling_) {
            chkCycling_->setEnabled(totalMode);
            if (!totalMode && chkCycling_->isChecked()) {
                QSignalBlocker b(chkCycling_);
                chkCycling_->setChecked(false);
            }
        }
        updateTemperatureSensorUi();
    }

    void onPidChanged(bool enabled) {
        for (auto* w : pidWidgets_) if (w) w->setEnabled(enabled);
        if (enabled && chkTempMonitor_ && !chkTempMonitor_->isChecked()) {
            QSignalBlocker b(chkTempMonitor_);
            chkTempMonitor_->setChecked(true);
        }
        updateLengthModeUi();
    }

    void onTempMonitoringChanged(bool) {
        updateTemperatureSensorUi();
    }

    void updateTemperatureSensorUi() {
        const bool pid = chkPid_ && chkPid_->isChecked();
        const bool targetMode = rbTargetHold_ && rbTargetHold_->isChecked();
        const bool required = pid || targetMode;
        const bool monitor = required || (chkTempMonitor_ && chkTempMonitor_->isChecked());
        if (chkTempMonitor_) {
            chkTempMonitor_->setEnabled(!required);
            if (required && !chkTempMonitor_->isChecked()) {
                QSignalBlocker b(chkTempMonitor_);
                chkTempMonitor_->setChecked(true);
            }
        }
        for (auto* w : tempWidgets_) if (w) w->setEnabled(monitor);
        if (chkSimTemp_) chkSimTemp_->setEnabled(monitor && debug_sim_build());
        if (grpSafety_) grpSafety_->setEnabled(monitor);
        if (lblTempRequirement_) {
            if (pid) {
                lblTempRequirement_->setText("HH806AU is required because PID feedback is enabled. Preflight must read a valid temperature before start.");
            } else if (targetMode) {
                lblTempRequirement_->setText("HH806AU is required because hold-after-target mode needs a valid temperature to start its hold timer.");
            } else if (monitor) {
                lblTempRequirement_->setText("Temperature monitoring is enabled. The sensor will be checked before start, but PID is inactive.");
            } else {
                lblTempRequirement_->setText("Temperature monitoring is off. HH806AU, PID feedback, and software cutoff are inactive.");
            }
        }
        if (lblTempStatus_) {
            lblTempStatus_->setText(monitor ? (required ? "Temp REQ" : "Temp MON") : "Temp OFF");
            lblTempStatus_->setStyleSheet(statusStyle(monitor ? (required ? "#e6930a" : "#667085") : "#667085"));
        }
    }

    void refreshPorts() {
        const QStringList ports = scanPorts();
        auto fill = [&](QComboBox* c, const QString& prefer) {
            QString prev = c->currentData().toString();
            c->clear();
            if (ports.empty()) {
                c->addItem("— no ports detected —", prefer);
                c->setEnabled(true);
                return;
            }
            int best = 0;
            for (int i = 0; i < ports.size(); ++i) {
                c->addItem(ports[i], ports[i]);
                if (ports[i] == prev || ports[i] == prefer) best = i;
            }
            c->setCurrentIndex(best);
        };
        fill(cmbCom3_, "COM3");
        fill(cmbCom11_, "COM5");
        lblPortInfo_->setText(QString("%1 port(s) found: %2").arg(ports.size()).arg(ports.join(", ")));
        // After the user clicks Scan Ports the selected port may have changed,
        // so the probe needs to re-target. Cheap; no I/O on this thread.
        pushProbeSettings();
    }

    void testTempConnection() {
        const bool active = (chkPid_ && chkPid_->isChecked()) || (rbTargetHold_ && rbTargetHold_->isChecked()) || (chkTempMonitor_ && chkTempMonitor_->isChecked());
        if (!active) {
            QMessageBox::information(this, "HH806AU Not Used", "Temperature monitoring is off, so HH806AU / COM5 is not used for this run.");
            return;
        }
        const bool sim = debugSimChecked(chkSimTemp_);
        try {
            std::unique_ptr<sonocontrol::ITemperatureSensor> sensor;
            if (sim) sensor = std::make_unique<sonocontrol::TemperatureSimulator>();
            else sensor = std::make_unique<sonocontrol::HH806AUSensor>(cmbCom11_->currentData().toString().toStdString(), 10.0, 80.0, 15.0);
            // Retry up to 3 times — HH806AU often ignores the first request
            // after a fresh port open. Give the device 150ms to warm up.
            std::optional<double> t1, t2;
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (attempt > 0) QThread::msleep(150);
                auto pair = sensor->read();
                t1 = pair.first;
                t2 = pair.second;
                if (t1 || t2) break;
            }
            if (t1 || t2) {
                lblTempStatus_->setStyleSheet(statusStyle("#1e8e3e"));
                QMessageBox::information(this, "HH806AU Connected",
                    QString("T1 = %1\nT2 = %2")
                    .arg(t1 ? QString::number(*t1, 'f', 2) + QString::fromUtf8("°C") : "N/C")
                    .arg(t2 ? QString::number(*t2, 'f', 2) + QString::fromUtf8("°C") : "N/C"));
            } else {
                lblTempStatus_->setStyleSheet(statusStyle("#d93025"));
                QMessageBox::warning(this, "Connection Failed", "Cannot read HH806AU after 3 attempts. Check port, cable, and power.");
            }
        } catch (const std::exception& e) {
            QMessageBox::warning(this, "Connection Failed", qstr(e.what()));
        }
    }

    void exportCsv() {
        const QString path = QFileDialog::getSaveFileName(this, "Export Temperature CSV", "sonocontrol_temp.csv", "CSV Files (*.csv)");
        if (path.isEmpty()) return;
        try {
            logger_.flush();
            logger_.export_csv_file(path.toStdString());
            QMessageBox::information(this, "Exported", "Saved to:\n" + path);
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Export Error", qstr(e.what()));
        }
    }

    void exportGraph() {
        const QString path = QFileDialog::getSaveFileName(this, "Export Temperature Graph", "sonocontrol_temp.png", "Images (*.png)");
        if (path.isEmpty()) return;
        QPixmap pix(tempPlot_->size());
        tempPlot_->render(&pix);
        if (!pix.save(path)) QMessageBox::critical(this, "Export Error", "Could not save image.");
        else QMessageBox::information(this, "Exported", "Saved to:\n" + path);
    }

    void openLogs() {
        QDesktopServices::openUrl(QUrl::fromLocalFile(qstr(logger_.log_dir().string())));
    }

    void importLogFile() {
        const QString path = QFileDialog::getOpenFileName(this, "Import Temperature Log", QString(), "CSV Files (*.csv);;All Files (*)");
        if (path.isEmpty()) return;
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Import Error", "Cannot open file:\n" + path);
            return;
        }
        const QString text = QString::fromUtf8(f.readAll());
        const QStringList lines = text.split(QRegularExpression("\\r?\\n"), Qt::SkipEmptyParts);
        if (lines.size() < 2) {
            QMessageBox::warning(this, "Import Error", "CSV file has no data rows.");
            return;
        }
        const QStringList header = lines.first().split(',');
        auto idx = [&](const QString& name) { return header.indexOf(name); };
        const int iTime = idx("time_s");
        const int iT1 = idx("T1_C");
        const int iT2 = idx("T2_C");
        const int iTemp = idx("temp_C");
        if (iTime < 0 || iTemp < 0) {
            QMessageBox::warning(this, "Import Error", "CSV must contain at least time_s and temp_C columns.");
            return;
        }
        tempPlot_->clear();
        tempPlot_->setLines(spnSetpoint_->value(), spnCutoff_->value());
        int count = 0;
        for (int r = 1; r < lines.size(); ++r) {
            const QStringList cols = lines[r].split(',');
            if (cols.size() <= std::max(iTime, iTemp)) continue;
            bool okTime = false, okTemp = false, okT1 = false, okT2 = false;
            const double ts = cols[iTime].trimmed().toDouble(&okTime);
            const double temp = cols[iTemp].trimmed().toDouble(&okTemp);
            double t1 = 0.0, t2 = 0.0;
            if (iT1 >= 0 && cols.size() > iT1) t1 = cols[iT1].trimmed().toDouble(&okT1);
            if (iT2 >= 0 && cols.size() > iT2) t2 = cols[iT2].trimmed().toDouble(&okT2);
            if (!okTime || !okTemp) continue;
            tempPlot_->append(ts, okT1 ? t1 : 0.0, okT2 ? t2 : 0.0, temp);
            ++count;
        }
        appendConsole(QString("Imported log for plotting: %1 (%2 samples)").arg(path).arg(count));
        statusBar()->showMessage(QString("Imported %1 samples from log").arg(count));
    }

    void setAutoSaveDirectory() {
        const QString start = autoSaveDir_.isEmpty() ? QDir::currentPath() : autoSaveDir_;
        const QString dir = QFileDialog::getExistingDirectory(this, "Set Auto-save Directory", start);
        if (dir.isEmpty()) return;
        autoSaveDir_ = dir;
        markConfigChanged();
        appendConsole("Auto-save directory set to: " + dir);
        QMessageBox::information(this, "Auto-save Directory", "Completed experiment artifacts will be saved under:\n" + dir);
    }

    void showReadme() {
        QMessageBox::information(this, "SonoControl Readme",
            "PID parameters\n\n"
            "Kp: proportional gain. Higher Kp reacts more strongly to temperature error. Increase if the system is too slow; decrease if it overshoots or oscillates.\n\n"
            "Ki: integral gain. It removes long-term offset, but too much Ki causes delayed overshoot. For 1 mL wells, start very small or zero.\n\n"
            "Kd: derivative gain. It suppresses rapid temperature rise. Too much Kd can make output noisy or overly conservative.\n\n"
            "Tau: liquid thermal time constant used by the prediction model T_future = T + tau*dT/dt*(1-exp(-t1/tau)). Larger tau brakes earlier; smaller tau behaves closer to ordinary PID. Tau=0 disables prediction.\n\n"
            "CONNECT page\n\n"
            "Scan Ports refreshes available serial ports. COM3 is the ultrasound controller port. UDP Host and UDP Port are the waveform target. The HH806AU section connects the temperature meter; select T1, T2, or Average. Use Test before running PID. Temperature monitoring is required for PID and after-target hold mode.\n\n"
            "Configuration\n\n"
            "Use File > Load Configuration and File > Save Configuration for .config files. Edit > Set Auto-save Directory stores the default completed-experiment output folder in the .config file. File > Import Log File loads a CSV log and redraws the temperature plot.");
    }

    void toggleConsole(bool v) { consoleFrame_->setVisible(v); }

    void idleTempRead() {
        // Idle reading disabled — caused GUI thread lag when PID was on.
        // Temperature only updates when an experiment is running.
        // Use the "Test" button in the CONNECT tab to verify sensor manually.
    }

    // ── Theme management ─────────────────────────────────────────────────
    static bool isNightTime() {
        const int h = QTime::currentTime().hour();
        return (h >= 17) || (h < 8);   // dark: 17:00–07:59 inclusive
    }

    void applyTheme(bool dark) {
        if (Theme::isDark() == dark && stylesheetApplied_) return;
        Theme::setDark(dark);
        setStyleSheet(styleSheetText());
        if (tempPlot_) tempPlot_->update();
        if (waveformPlot_) waveformPlot_->update();
        if (chkAutoTheme_) chkAutoTheme_->blockSignals(true);
        if (chkDarkMode_)  chkDarkMode_->blockSignals(true);
        if (chkDarkMode_)  chkDarkMode_->setChecked(dark);
        if (chkAutoTheme_) chkAutoTheme_->blockSignals(false);
        if (chkDarkMode_)  chkDarkMode_->blockSignals(false);
        stylesheetApplied_ = true;
    }

    void onAutoThemeToggled(bool auto_on) {
        if (auto_on) {
            applyTheme(isNightTime());
            themeTimer_.start();
        } else {
            themeTimer_.stop();
        }
    }

    void onDarkToggled(bool dark) {
        // Manual toggle disables auto-mode so the user's choice sticks.
        if (chkAutoTheme_) {
            chkAutoTheme_->blockSignals(true);
            chkAutoTheme_->setChecked(false);
            chkAutoTheme_->blockSignals(false);
        }
        themeTimer_.stop();
        applyTheme(dark);
    }

    void checkThemeByTime() {
        if (chkAutoTheme_ && chkAutoTheme_->isChecked()) {
            applyTheme(isNightTime());
        }
    }

private:
    QString styleSheetText() const {
        const QString BG       = Theme::bg();
        const QString PANEL    = Theme::panel();
        const QString PANEL2   = Theme::panel2();
        const QString BORDER   = Theme::border();
        const QString BORDER2  = Theme::border2();
        const QString TEXT     = Theme::text();
        const QString TEXT2    = Theme::text2();
        const QString TEXT3    = Theme::text3();
        const QString ACCENT   = Theme::accent();
        const QString ACCENT2  = Theme::accent2();
        const QString HOVER_BG = Theme::isDark() ? "#334155" : "#e8f5f3";
        const QString INPUT_BG = Theme::isDark() ? "#0f172a" : "#ffffff";
        const QString CONSOLE_BG = Theme::isDark() ? "#020617" : "#1a1a2e";
        const QString CONSOLE_FG = Theme::isDark() ? "#cbd5e1" : "#d0d5dd";

        return QString(R"QSS(
QMainWindow, QWidget {
    background-color: %1;
    color: %2;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
}

QFrame#topBar {
    background: %3;
    border-bottom: 1px solid %4;
}

QGroupBox {
    background: %3;
    border: 1px solid %4;
    border-radius: 10px;
    margin-top: 16px;
    padding: 12px 14px 10px 14px;
    font-size: 11px;
    font-weight: 600;
    color: %5;
    letter-spacing: 0.5px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    top: -1px;
    padding: 0 6px;
    color: %6;
    background: %1;
    font-size: 11px;
    text-transform: uppercase;
}

QLabel {
    color: %5;
    background: transparent;
    font-size: 13px;
}

QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
    background: %7;
    border: 1px solid %8;
    border-radius: 6px;
    color: %2;
    padding: 4px 8px;
    font-family: Consolas, monospace;
    font-size: 13px;
    min-height: 26px;
}
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus {
    border: 1.5px solid %6;
}

QComboBox QAbstractItemView {
    background: %3;
    color: %2;
    border: 1px solid %8;
    selection-background-color: %6;
    selection-color: white;
    padding: 2px;
    outline: 0;
}

QPushButton {
    background: %3;
    border: 1px solid %8;
    border-radius: 6px;
    color: %2;
    padding: 5px 14px;
    font-size: 13px;
    min-height: 28px;
}
QPushButton:hover {
    border-color: %6;
    color: %9;
    background: %10;
}

QPushButton#start {
    background: %6;
    color: white;
    font-weight: 700;
    font-size: 15px;
    border: none;
    padding: 8px 22px;
    min-height: 36px;
}
QPushButton#start:hover { background: %9; }
QPushButton#start:disabled { background: %4; color: %11; }

QPushButton#stop {
    background: rgba(239, 68, 68, 0.15);
    color: #ef4444;
    font-weight: 700;
    font-size: 15px;
    border: 1px solid rgba(239, 68, 68, 0.4);
    padding: 8px 22px;
    min-height: 36px;
}
QPushButton#stop:hover { background: #ef4444; color: white; }
QPushButton#stop:disabled { background: %1; color: %11; border: none; }

QCheckBox, QRadioButton {
    color: %5;
    background: transparent;
    spacing: 6px;
    font-size: 13px;
}
QCheckBox::indicator {
    width: 16px; height: 16px;
    border: 1.5px solid %11;
    border-radius: 3px;
    background: %7;
}
QCheckBox::indicator:checked {
    border-color: %6;
    background: %6;
}
QRadioButton::indicator {
    width: 16px; height: 16px;
    border: 1.5px solid %11;
    border-radius: 9px;
    background: %7;
}
QRadioButton::indicator:checked {
    border-color: %6;
    background: %6;
}

QTabWidget::pane {
    border: none;
    background: transparent;
    border-top: 1px solid %4;
}
QTabBar::tab {
    background: transparent;
    border: none;
    padding: 9px 16px;
    color: %11;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.5px;
    min-width: 60px;
}
QTabBar::tab:hover { color: %5; }
QTabBar::tab:selected {
    color: %6;
    border-bottom: 2px solid %6;
}

QPlainTextEdit#console {
    background: %12;
    color: %13;
    font-family: Consolas, monospace;
    font-size: 12px;
    border: 1px solid %4;
    border-radius: 6px;
    padding: 6px;
}

QStatusBar {
    background: %3;
    color: %5;
    border-top: 1px solid %4;
    font-size: 12px;
}

QScrollArea { border: none; background: %1; }

QScrollBar:vertical {
    background: %1;
    width: 10px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: %8;
    min-height: 28px;
    border-radius: 5px;
}
QScrollBar::handle:vertical:hover { background: %11; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QScrollBar:horizontal {
    background: %1;
    height: 10px;
    border-radius: 5px;
}
QScrollBar::handle:horizontal {
    background: %8;
    min-width: 28px;
    border-radius: 5px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
)QSS")
        .arg(BG).arg(TEXT).arg(PANEL).arg(BORDER).arg(TEXT2).arg(ACCENT)
        .arg(INPUT_BG).arg(BORDER2).arg(ACCENT2).arg(HOVER_BG).arg(TEXT3)
        .arg(CONSOLE_BG).arg(CONSOLE_FG);
    }

    QString valueStyle(const QString& color = "#00897b") const {
        return QString("color:%1; font-family:Consolas,'Courier New',monospace; font-size:16px; font-weight:700;").arg(color);
    }

    QLabel* statLabel(const QString& text, const QString& color = "#00897b") {
        auto* l = new QLabel(text);
        l->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        l->setStyleSheet(QString("color:%1; font-family:Consolas,monospace; font-size:14px; font-weight:600;").arg(color));
        return l;
    }

    QDoubleSpinBox* dspin(double mn, double mx, double val, int dec, double step) {
        auto* s = new QDoubleSpinBox;
        s->setRange(mn, mx); s->setValue(val); s->setDecimals(dec); s->setSingleStep(step); s->setAccelerated(true); s->setMinimumWidth(80); s->setMaximumWidth(140);
        return s;
    }
    QSpinBox* ispin(int mn, int mx, int val) {
        auto* s = new QSpinBox;
        s->setRange(mn, mx); s->setValue(val); s->setAccelerated(true); s->setMinimumWidth(80); s->setMaximumWidth(140);
        return s;
    }

    bool debugSimChecked(QCheckBox* c) const { return debug_sim_build() && c && c->isChecked(); }

    void buildMenuBar() {
        auto* file = menuBar()->addMenu("File");
        auto* actExportCsv = file->addAction("Export CSV...");
        auto* actExportImage = file->addAction("Export Picture...");
        file->addSeparator();
        auto* actSaveConfig = file->addAction("Save Configuration...");
        auto* actLoadConfig = file->addAction("Load Configuration...");
        auto* actTemplate = file->addAction("Write Configuration Template...");
        file->addSeparator();
        auto* actImportLog = file->addAction("Import Log File...");
        connect(actExportCsv, &QAction::triggered, this, &SonoControlWindow::exportCsv);
        connect(actExportImage, &QAction::triggered, this, &SonoControlWindow::exportGraph);
        connect(actSaveConfig, &QAction::triggered, this, &SonoControlWindow::saveConfigFile);
        connect(actLoadConfig, &QAction::triggered, this, &SonoControlWindow::loadConfigFile);
        connect(actTemplate, &QAction::triggered, this, &SonoControlWindow::writeConfigTemplate);
        connect(actImportLog, &QAction::triggered, this, &SonoControlWindow::importLogFile);

        auto* edit = menuBar()->addMenu("Edit");
        auto* actAutoSaveDir = edit->addAction("Set Auto-save Directory...");
        connect(actAutoSaveDir, &QAction::triggered, this, &SonoControlWindow::setAutoSaveDirectory);

        auto* about = menuBar()->addMenu("About");
        auto* actReadme = about->addAction("Readme");
        connect(actReadme, &QAction::triggered, this, &SonoControlWindow::showReadme);
    }

    void buildUi() {
        buildMenuBar();
        auto* central = new QWidget;
        setCentralWidget(central);
        auto* root = new QVBoxLayout(central);
        root->setContentsMargins(0,0,0,0); root->setSpacing(0);
        root->addWidget(buildHeader());
        auto* body = new QHBoxLayout; body->setContentsMargins(8,8,8,8); body->setSpacing(8);
        auto* left = new QWidget; left->setFixedWidth(420); auto* ll = new QVBoxLayout(left); ll->setContentsMargins(0,0,0,0); ll->addWidget(buildLeftPanel());
        auto* right = new QWidget; right->setFixedWidth(220); auto* rl = new QVBoxLayout(right); rl->setContentsMargins(0,0,0,0); rl->addWidget(buildRightPanel());
        body->addWidget(left); body->addWidget(buildCenterPanel(), 1); body->addWidget(right);
        auto* bodyW = new QWidget; bodyW->setLayout(body); root->addWidget(bodyW, 1);
        root->addWidget(buildConsolePanel());
        consoleFrame_->setVisible(false);
    }

    QWidget* buildHeader() {
        auto* h = new QFrame; h->setObjectName("topBar"); h->setFixedHeight(76);
        auto* l = new QHBoxLayout(h); l->setContentsMargins(24,0,24,0);
        // Brand mark — small teal block to anchor the title visually
        auto* brand = new QLabel("●");
        brand->setStyleSheet("color:#00897b; font-size:22px; font-weight:900; margin-right:4px;");
        auto* title = new QLabel("SonoControl");
        title->setStyleSheet("color:#1a1a2e; font-size:20px; font-weight:700; letter-spacing:0.5px;");
        auto* subtitle = new QLabel("Ultrasound Thermal Controller");
        subtitle->setStyleSheet("font-size:12px; color:#98a2b3; margin-left:10px; margin-top:4px;");
        l->addWidget(brand);
        l->addWidget(title);
        l->addWidget(subtitle);
        l->addStretch();
        if (debug_sim_build()) {
            auto* dbg = new QLabel("DEBUG MODE"); dbg->setStyleSheet("color:#b54708; background:#fff4e5; border:1px solid #fed7aa; border-radius:8px; padding:6px 10px; font-weight:700; font-size:12px; margin-right:8px;"); l->addWidget(dbg);
            chkSimTemp_ = new QCheckBox("Sim Temp"); chkSimUs_ = new QCheckBox("Sim US"); chkSimTemp_->setChecked(true); chkSimUs_->setChecked(true);
            l->addWidget(chkSimTemp_); l->addWidget(chkSimUs_);
        }
        // Status pills — compact background-colored labels for at-a-glance state
        lblCom3Status_ = new QLabel("COM3");
        lblUsStatus_ = new QLabel("UDP");
        lblTempStatus_ = new QLabel("Temp");
        for (auto* x : {lblCom3Status_, lblUsStatus_, lblTempStatus_}) {
            x->setStyleSheet(statusStyle("#b54708"));
            x->setMinimumHeight(22);
            x->setMaximumHeight(24);
            l->addWidget(x);
            l->addSpacing(4);
        }
        btnStart_ = new QPushButton(" ▶  START "); btnStart_->setObjectName("start"); btnStart_->setMinimumWidth(120); btnStart_->setCursor(Qt::PointingHandCursor); connect(btnStart_, &QPushButton::clicked, this, &SonoControlWindow::onStart);
        btnStop_ = new QPushButton(" ■  EMERGENCY STOP "); btnStop_->setObjectName("stop"); btnStop_->setMinimumWidth(170); btnStop_->setEnabled(false); btnStop_->setCursor(Qt::PointingHandCursor);
        // Dispatch click depending on which phase we're in: first click sends
        // a graceful request_stop; if the worker doesn't exit in 2 s, the
        // button is re-labelled "FORCE STOP" and the next click cancels I/O.
        connect(btnStop_, &QPushButton::clicked, this, [this]() {
            if (btnStop_->text().contains("FORCE", Qt::CaseInsensitive)) onForceStop();
            else onStop();
        });
        l->addSpacing(6);
        l->addWidget(btnStart_); l->addSpacing(6); l->addWidget(btnStop_);
        return h;
    }

    QWidget* buildLeftPanel() {
        auto* scroll = new QScrollArea; scroll->setWidgetResizable(true); scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* cont = new QWidget; cont->setMaximumWidth(410);
        auto* v = new QVBoxLayout(cont); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* tabs = new QTabWidget; tabs->addTab(buildParamsTab(), "PARAMS"); tabs->addTab(buildPidTab(), "PID"); tabs->addTab(buildCycleTab(), "CYCLE"); tabs->addTab(buildConnTab(), "CONNECT");
        v->addWidget(tabs);
        lblConfigStatus_ = new QLabel("Config: GUI defaults");
        lblConfigStatus_->setWordWrap(true);
        lblConfigStatus_->setStyleSheet("color:#667085; font-size:12px; padding:4px;");
        v->addWidget(lblConfigStatus_);
        v->addStretch(); scroll->setWidget(cont); return scroll;
    }

    QWidget* buildParamsTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* us = new QGroupBox("Ultrasound"); auto* g = new QGridLayout(us);
        spnAmp_ = dspin(0.0,1.0,0.5,3,0.01); spnCfreq_ = dspin(1.0,10000.0,500.0,1,10.0); spnPrf_ = dspin(1.0,100000.0,1000.0,0,100.0); spnDuty_ = dspin(0.0,100.0,50.0,2,0.1);
        cmbWave_ = new QComboBox; cmbWave_->addItems({"Sine","Square","Triangle"});
        addRow(g,0,"Amplitude",spnAmp_,"(0–1)"); addRow(g,1,"CFreq",spnCfreq_,"kHz"); addRow(g,2,"PRF",spnPrf_,"Hz"); addRow(g,3,"Duty Cycle",spnDuty_,"%"); addRow(g,4,"Waveform",cmbWave_,""); v->addWidget(us);
        auto* timing = new QGroupBox("Timing"); auto* gt = new QGridLayout(timing);
        spnDuration_ = dspin(10.0,60000.0,1000.0,0,100.0); spnInterval_ = dspin(0.01,3600.0,5.0,2,0.5); spnSampleRate_ = dspin(0.1,20.0,2.0,1,0.5);
        addRow(gt,0,"Duration",spnDuration_,"ms"); addRow(gt,1,"Interval",spnInterval_,"s"); addRow(gt,2,"Sample Rate",spnSampleRate_,"Hz"); v->addWidget(timing);
        auto* total = new QGroupBox("Experiment Length"); auto* tv = new QVBoxLayout(total);
        rbTotalDur_ = new QRadioButton("Total Duration (mins)");
        rbRepeating_ = new QRadioButton("Repeating (cycles)");
        rbTargetHold_ = new QRadioButton("After target reached, hold for");
        rbTotalDur_->setChecked(true);
        auto* bg = new QButtonGroup(this); bg->addButton(rbTotalDur_); bg->addButton(rbRepeating_); bg->addButton(rbTargetHold_);
        spnTotalDur_ = dspin(0.1,72*60,60.0,1,1.0);
        spnRepeating_ = ispin(1,1000000,720);
        spnTargetHoldMin_ = dspin(0.1,72*60,10.0,1,1.0);
        spnTargetTol_ = dspin(0.05,5.0,0.3,2,0.05);
        auto* row1 = new QHBoxLayout; row1->addWidget(rbTotalDur_); row1->addWidget(spnTotalDur_); row1->addWidget(new QLabel("min"));
        auto* row2 = new QHBoxLayout; row2->addWidget(rbRepeating_); row2->addWidget(spnRepeating_); row2->addWidget(new QLabel("×"));
        auto* row3 = new QHBoxLayout; row3->addWidget(rbTargetHold_); row3->addWidget(spnTargetHoldMin_); row3->addWidget(new QLabel("min"));
        auto* row4 = new QHBoxLayout; row4->addSpacing(26); row4->addWidget(new QLabel("Target tolerance ±")); row4->addWidget(spnTargetTol_); row4->addWidget(new QLabel(QString::fromUtf8("°C")));
        tv->addLayout(row1); tv->addLayout(row2); tv->addLayout(row3); tv->addLayout(row4);
        auto* note = new QLabel("PID mode allows Total Duration or After-target Hold only. Cycle mode is available only with Total Duration.");
        note->setWordWrap(true); note->setStyleSheet("color:#667085; font-size:12px;");
        tv->addWidget(note);
        v->addWidget(total);
        connect(rbTotalDur_, &QRadioButton::toggled, this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(rbRepeating_, &QRadioButton::toggled, this, &SonoControlWindow::updateLengthModeUi);
        connect(rbTargetHold_, &QRadioButton::toggled, this, &SonoControlWindow::updateLengthModeUi);
        connect(spnTotalDur_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(spnInterval_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(spnRepeating_, qOverload<int>(&QSpinBox::valueChanged), this, &SonoControlWindow::updateDurationFromRepeating);
        grpSafety_ = new QGroupBox("Safety"); auto* sh = new QHBoxLayout(grpSafety_); spnCutoff_ = dspin(35.0,60.0,45.0,1,0.5); sh->addWidget(new QLabel("Cutoff Temp")); sh->addWidget(spnCutoff_); sh->addWidget(new QLabel(QString::fromUtf8("°C"))); v->addWidget(grpSafety_); v->addStretch(); return w;
    }

    QWidget* buildPidTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* en = new QGroupBox("PID Enable"); auto* ev = new QVBoxLayout(en); chkPid_ = new QCheckBox("Enable PID Temperature Control"); ev->addWidget(chkPid_); connect(chkPid_, &QCheckBox::toggled, this, [this](bool v){ idleSensor_.reset(); onPidChanged(v); }); v->addWidget(en);
        auto* sp = new QGroupBox("Setpoint"); auto* sph = new QHBoxLayout(sp); spnSetpoint_ = dspin(20.0,50.0,40.0,1,0.5); sph->addWidget(new QLabel("Target Temp")); sph->addWidget(spnSetpoint_); sph->addWidget(new QLabel(QString::fromUtf8("°C"))); v->addWidget(sp);
        auto* sel = new QGroupBox("Controlled Parameters"); auto* sv = new QVBoxLayout(sel); chkPidAmp_ = new QCheckBox("Amplitude"); chkPidDuration_ = new QCheckBox("Duration"); chkPidInterval_ = new QCheckBox("Interval Time"); chkPidDuty_ = new QCheckBox("Duty Cycle"); chkPidAmp_->setChecked(true); for (auto* c : {chkPidAmp_, chkPidDuration_, chkPidInterval_, chkPidDuty_}) sv->addWidget(c); sv->addWidget(new QLabel("Interval ≥ Duration enforced")); v->addWidget(sel);
        auto* gains = new QGroupBox("PID Gains");
        auto* gg = new QGridLayout(gains);
        spnKp_ = dspin(0.0,10.0,0.8,3,0.05);
        spnKi_ = dspin(0.0,5.0,0.05,3,0.01);
        spnKd_ = dspin(0.0,5.0,0.2,3,0.05);
        spnTau_ = dspin(0.0,3600.0,25.0,1,1.0);
        spnHorizon_ = dspin(0.0,300.0,0.0,1,1.0);
        addRow(gg,0,"Kp",spnKp_,"");
        addRow(gg,1,"Ki",spnKi_,"");
        addRow(gg,2,"Kd",spnKd_,"");
        addRow(gg,3,"Thermal const",spnTau_,"s");
        addRow(gg,4,"Prediction horizon",spnHorizon_,"s");
        auto* tauNote = new QLabel("Model: T_future = T + thermal_const × dT/dt × (1 - exp(-horizon/thermal_const)). Set thermal_const=0 to disable prediction. Set horizon=0 to use the current hardware interval.");
        tauNote->setWordWrap(true);
        tauNote->setStyleSheet("color:#667085; font-size:12px;");
        gg->addWidget(tauNote,5,0,1,3);
        v->addWidget(gains);

        pidWidgets_ = {sp, sel, gains}; v->addStretch(); return w;
    }

    QWidget* buildCycleTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* en = new QGroupBox("Temperature Cycling"); auto* ev = new QVBoxLayout(en); chkCycling_ = new QCheckBox("Enable Heating / Cooling Cycles"); ev->addWidget(chkCycling_); auto* cycleNote = new QLabel("Available only in Total Duration mode. Phase durations are fixed wall-clock durations for long-run stability."); cycleNote->setWordWrap(true); cycleNote->setStyleSheet("color:#667085; font-size:12px;"); ev->addWidget(cycleNote); v->addWidget(en);
        auto* heat = new QGroupBox("Heating Phase"); auto* hh = new QHBoxLayout(heat); spnHeatS_ = dspin(1.0,86400.0,60.0,0,5.0); hh->addWidget(new QLabel("Duration")); hh->addWidget(spnHeatS_); hh->addWidget(new QLabel("s")); v->addWidget(heat);
        auto* cool = new QGroupBox("Cooling Phase"); auto* cv = new QVBoxLayout(cool); auto* cr = new QHBoxLayout; spnCoolS_ = dspin(1.0,86400.0,30.0,0,5.0); cr->addWidget(new QLabel("Duration")); cr->addWidget(spnCoolS_); cr->addWidget(new QLabel("s")); cv->addLayout(cr);
        rbCoolStop_ = new QRadioButton("Stop ultrasound"); rbCoolLow_ = new QRadioButton("Hold at temperature (PID)"); rbCoolStop_->setChecked(true); auto* bg = new QButtonGroup(this); bg->addButton(rbCoolStop_); bg->addButton(rbCoolLow_); cv->addWidget(rbCoolStop_); cv->addWidget(rbCoolLow_);
        auto* hold = new QGroupBox("Hold Temperature"); auto* hv = new QHBoxLayout(hold); spnCoolHoldTemp_ = dspin(20.0,45.0,37.0,1,0.5); hv->addWidget(new QLabel("Target")); hv->addWidget(spnCoolHoldTemp_); hv->addWidget(new QLabel(QString::fromUtf8("°C"))); cv->addWidget(hold); v->addWidget(cool); v->addStretch(); return w;
    }

    QWidget* buildConnTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* scan = new QPushButton("⟳  Scan Ports"); connect(scan, &QPushButton::clicked, this, &SonoControlWindow::refreshPorts); v->addWidget(scan);
        auto* us = new QGroupBox("Ultrasound Device"); auto* gu = new QGridLayout(us); cmbCom3_ = new QComboBox; txtUdpHost_ = new QLineEdit(sonocontrol::protocol::UDP_HOST_DEFAULT); spnUdpPort_ = ispin(1,65535,sonocontrol::protocol::UDP_PORT_DEFAULT); addRow(gu,0,"COM3 Port",cmbCom3_,""); addRow(gu,1,"UDP Host",txtUdpHost_,""); addRow(gu,2,"UDP Port",spnUdpPort_,""); v->addWidget(us);
        grpTempSensor_ = new QGroupBox("Temperature Sensor (HH806AU)"); auto* tg = new QGridLayout(grpTempSensor_); chkTempMonitor_ = new QCheckBox("Connect / monitor temperature sensor"); connect(chkTempMonitor_, &QCheckBox::toggled, this, [this](bool v){ idleSensor_.reset(); onTempMonitoringChanged(v); }); cmbCom11_ = new QComboBox; btnTestTemp_ = new QPushButton("Test"); btnTestTemp_->setFixedWidth(72); connect(btnTestTemp_, &QPushButton::clicked, this, &SonoControlWindow::testTempConnection); cmbTempChannel_ = new QComboBox; cmbTempChannel_->addItems({"T1 only","T2 only","Average T1+T2"}); cmbTempChannel_->setCurrentIndex(0); chkTempFallback_ = new QCheckBox("Allow fallback to the other probe if selected channel is unavailable"); chkTempFallback_->setToolTip("Enable only when both T1 and T2 probes are physically installed. Fallback usage is logged."); spnTempMin_ = dspin(-20.0, 80.0, 10.0, 1, 1.0); spnTempMax_ = dspin(20.0, 150.0, 80.0, 1, 1.0); spnTempRate_ = dspin(1.0, 100.0, 15.0, 1, 1.0); lblPortInfo_ = new QLabel; lblPortInfo_->setWordWrap(true); lblTempRequirement_ = new QLabel; lblTempRequirement_->setWordWrap(true); lblTempRequirement_->setStyleSheet("color:#667085; font-size:13px;"); tg->addWidget(chkTempMonitor_,0,0,1,3); tg->addWidget(new QLabel("COM Port"),1,0); tg->addWidget(cmbCom11_,1,1); tg->addWidget(btnTestTemp_,1,2); tg->addWidget(new QLabel("Channel"),2,0); tg->addWidget(cmbTempChannel_,2,1,1,2); tg->addWidget(chkTempFallback_,3,0,1,3); tg->addWidget(new QLabel("Valid range"),4,0); auto* rangeRow = new QWidget; auto* rr = new QHBoxLayout(rangeRow); rr->setContentsMargins(0,0,0,0); rr->addWidget(spnTempMin_); rr->addWidget(new QLabel("to")); rr->addWidget(spnTempMax_); rr->addWidget(new QLabel(QString::fromUtf8("°C"))); tg->addWidget(rangeRow,4,1,1,2); tg->addWidget(new QLabel("Max rate"),5,0); auto* rateRow = new QWidget; auto* rt = new QHBoxLayout(rateRow); rt->setContentsMargins(0,0,0,0); rt->addWidget(spnTempRate_); rt->addWidget(new QLabel(QString::fromUtf8("°C/s"))); tg->addWidget(rateRow,5,1,1,2); tg->addWidget(lblTempRequirement_,6,0,1,3); tg->addWidget(lblPortInfo_,7,0,1,3); v->addWidget(grpTempSensor_);
        tempWidgets_ = {cmbCom11_, cmbTempChannel_, btnTestTemp_, chkTempFallback_, spnTempMin_, spnTempMax_, spnTempRate_};
        auto* misc = new QGroupBox("Console"); auto* mv = new QVBoxLayout(misc); chkConsole_ = new QCheckBox("Enable Console Output"); connect(chkConsole_, &QCheckBox::toggled, this, &SonoControlWindow::toggleConsole); mv->addWidget(chkConsole_); v->addWidget(misc);

        // Theme controls
        auto* themeGrp = new QGroupBox("Appearance");
        auto* tv = new QVBoxLayout(themeGrp);
        chkDarkMode_  = new QCheckBox("Dark Mode");
        chkAutoTheme_ = new QCheckBox("Auto (dark from 17:00 to 08:00)");
        chkDarkMode_->setChecked(Theme::isDark());
        chkAutoTheme_->setChecked(true);
        connect(chkDarkMode_,  &QCheckBox::toggled, this, &SonoControlWindow::onDarkToggled);
        connect(chkAutoTheme_, &QCheckBox::toggled, this, &SonoControlWindow::onAutoThemeToggled);
        tv->addWidget(chkDarkMode_);
        tv->addWidget(chkAutoTheme_);
        v->addWidget(themeGrp);

        v->addStretch(); return w;
    }

    QWidget* buildExportPanel() {
        auto* grp = new QGroupBox("Config / Export / Logs");
        auto* g = new QGridLayout(grp);
        auto* load = new QPushButton("Load .config");
        auto* save = new QPushButton("Save .config");
        auto* templ = new QPushButton("Template");
        auto* csv = new QPushButton("CSV");
        auto* graph = new QPushButton("Graph");
        auto* logs = new QPushButton("Open Logs");
        connect(load, &QPushButton::clicked, this, &SonoControlWindow::loadConfigFile);
        connect(save, &QPushButton::clicked, this, &SonoControlWindow::saveConfigFile);
        connect(templ, &QPushButton::clicked, this, &SonoControlWindow::writeConfigTemplate);
        connect(csv, &QPushButton::clicked, this, &SonoControlWindow::exportCsv);
        connect(graph, &QPushButton::clicked, this, &SonoControlWindow::exportGraph);
        connect(logs, &QPushButton::clicked, this, &SonoControlWindow::openLogs);
        lblConfigStatus_ = new QLabel("Config: GUI defaults");
        lblConfigStatus_->setWordWrap(true);
        lblConfigStatus_->setStyleSheet("color:#667085; font-size:12px;");
        g->addWidget(load,0,0); g->addWidget(save,0,1);
        g->addWidget(templ,1,0); g->addWidget(logs,1,1);
        g->addWidget(csv,2,0); g->addWidget(graph,2,1);
        g->addWidget(lblConfigStatus_,3,0,1,2);
        return grp;
    }

    QWidget* buildCenterPanel() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* temp = new QGroupBox("Temperature"); auto* tv = new QVBoxLayout(temp); tempPlot_ = new TemperaturePlot; tv->addWidget(tempPlot_); v->addWidget(temp, 2);
        auto* wf = new QGroupBox("Waveform (current UDP burst)"); auto* wv = new QVBoxLayout(wf); waveformPlot_ = new WaveformPlot; waveformPlot_->setWaveform(sonocontrol::protocol::waveform_float_array(0.5,0.5,sonocontrol::WaveShape::Sine)); wv->addWidget(waveformPlot_); v->addWidget(wf, 1);
        return w;
    }

    QWidget* buildRightPanel() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* timer = new QGroupBox("Session Timer"); auto* tg = new QGridLayout(timer); lblElapsed_ = statLabel("00:00:00"); lblRemaining_ = statLabel("--:--:--"); lblCycle_ = statLabel("IDLE", "#5a6070"); addStat(tg,0,"Elapsed",lblElapsed_); addStat(tg,1,"Remaining",lblRemaining_); addStat(tg,2,"Phase",lblCycle_); v->addWidget(timer);
        auto* cur = new QGroupBox("Temperature"); auto* cg = new QGridLayout(cur); lblT1_ = statLabel("--.-°C"); lblT2_ = statLabel("--.-°C"); lblTavg_ = statLabel("--.-°C"); addStat(cg,0,"T1",lblT1_); addStat(cg,1,"T2",lblT2_); addStat(cg,2,"Avg",lblTavg_); v->addWidget(cur);
        auto* active = new QGroupBox("Active Parameters"); auto* ag = new QGridLayout(active); lblCurAmp_=statLabel("--"); lblCurCfreq_=statLabel("-- kHz"); lblCurPrf_=statLabel("-- Hz"); lblCurDuty_=statLabel("--%"); lblCurDur_=statLabel("-- ms"); lblCurIntv_=statLabel("-- s"); addStat(ag,0,"Amplitude",lblCurAmp_); addStat(ag,1,"CFreq",lblCurCfreq_); addStat(ag,2,"PRF",lblCurPrf_); addStat(ag,3,"Duty",lblCurDuty_); addStat(ag,4,"Duration",lblCurDur_); addStat(ag,5,"Interval",lblCurIntv_); v->addWidget(active); v->addStretch(); return w;
    }

    QWidget* buildConsolePanel() {
        consoleFrame_ = new QFrame; auto* cv = new QVBoxLayout(consoleFrame_); cv->setContentsMargins(8,0,8,6); cv->setSpacing(2); auto* hdr = new QHBoxLayout; auto* lbl = new QLabel("CONSOLE"); lbl->setStyleSheet("color:#475467; font-size:12px; font-weight:600; letter-spacing:1px;"); hdr->addWidget(lbl); hdr->addStretch(); auto* clear = new QPushButton("Clear"); clear->setFixedSize(50,20); connect(clear, &QPushButton::clicked, [this]{ console_->clear(); }); hdr->addWidget(clear); cv->addLayout(hdr); console_ = new QPlainTextEdit; console_->setObjectName("console"); console_->setReadOnly(true); console_->setMaximumBlockCount(2000); console_->setFixedHeight(180); cv->addWidget(console_); consoleFrame_->setVisible(false); return consoleFrame_;
    }

    void addRow(QGridLayout* g, int row, const QString& label, QWidget* w, const QString& unit) {
        g->addWidget(new QLabel(label), row, 0); g->addWidget(w, row, 1); if (!unit.isEmpty()) g->addWidget(new QLabel(unit), row, 2);
    }
    void addStat(QGridLayout* g, int row, const QString& label, QLabel* value) {
        g->addWidget(new QLabel(label), row, 0); g->addWidget(value, row, 1);
    }

    QStringList scanPorts() const {
        QStringList out;
#ifdef _WIN32
        // Method 1: HKLM\HARDWARE\DEVICEMAP\SERIALCOMM — the canonical
        // location every Windows serial driver writes its port name to.
        // This catches USB-to-Serial adapters (FTDI, CH340, Prolific, etc.)
        // that QueryDosDevice() sometimes misses.
        HKEY hKey = nullptr;
        if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                          L"HARDWARE\\DEVICEMAP\\SERIALCOMM",
                          0, KEY_READ, &hKey) == ERROR_SUCCESS) {
            DWORD index = 0;
            wchar_t valueName[256];
            wchar_t valueData[256];
            while (true) {
                DWORD nameLen = 256;
                DWORD dataLen = sizeof(valueData);
                DWORD type = 0;
                LONG rc = RegEnumValueW(hKey, index, valueName, &nameLen,
                                        nullptr, &type,
                                        reinterpret_cast<LPBYTE>(valueData), &dataLen);
                if (rc != ERROR_SUCCESS) break;
                if (type == REG_SZ) {
                    QString port = QString::fromWCharArray(valueData);
                    if (!port.isEmpty() && !out.contains(port)) out << port;
                }
                ++index;
            }
            RegCloseKey(hKey);
        }
        // Method 2: fall back to QueryDosDevice for virtual ports that
        // don't register under SERIALCOMM (e.g. some null-modem emulators).
        for (int i = 1; i <= 64; ++i) {
            QString port = QString("COM%1").arg(i);
            wchar_t target[512];
            if (QueryDosDeviceW(reinterpret_cast<LPCWSTR>(port.utf16()), target, 512) != 0) {
                if (!out.contains(port)) out << port;
            }
        }
        // Method 3: brute-force CreateFile probe. Some drivers (certain
        // FTDI / silabs revisions) register elsewhere; the only reliable
        // existence test is to try to open the port. Use \\.\COMx syntax
        // which works for any COM number (regular COM1..COM9 form fails for
        // COM10+). Open with no access (third param = 0) to avoid locking.
        for (int i = 1; i <= 64; ++i) {
            const QString port = QString("COM%1").arg(i);
            if (out.contains(port)) continue;
            const std::wstring path = (L"\\\\.\\COM" + std::to_wstring(i));
            HANDLE h = CreateFileW(path.c_str(), 0, 0, nullptr,
                                   OPEN_EXISTING, 0, nullptr);
            if (h != INVALID_HANDLE_VALUE) {
                CloseHandle(h);
                out << port;
            } else {
                const DWORD err = GetLastError();
                // ERROR_ACCESS_DENIED (5) means port exists but is in use.
                // ERROR_SHARING_VIOLATION (32) same. Both indicate the port
                // exists, so add it.
                if (err == ERROR_ACCESS_DENIED || err == ERROR_SHARING_VIOLATION) {
                    out << port;
                }
            }
        }
        // Sort numerically by COM number
        std::sort(out.begin(), out.end(), [](const QString& a, const QString& b) {
            return a.mid(3).toInt() < b.mid(3).toInt();
        });
#else
        const QStringList patterns = {"ttyUSB*", "ttyACM*", "ttyS*", "cu.*"};
        const QStringList roots = {"/dev"};
        for (const QString& root : roots) {
            QDir d(root);
            for (const QString& pat : patterns) {
                const auto entries = d.entryList({pat}, QDir::System | QDir::Files | QDir::Readable, QDir::Name);
                for (const QString& e : entries) out << d.absoluteFilePath(e);
            }
        }
#endif
        out.removeDuplicates();
        out.sort();
        return out;
    }


    bool preflightCheck(sonocontrol::Config& cfg) {
        const double duration_s = spnDuration_->value() / 1000.0;
        if (spnInterval_->value() < duration_s) {
            QMessageBox::warning(this, "Invalid Timing",
                                 "Interval must be greater than or equal to Duration. The experiment was not started.");
            return false;
        }
        if (cfg.cutoff_temp <= cfg.pid_setpoint && (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget)) {
            QMessageBox::warning(this, "Invalid Safety Limit",
                                 "Cutoff temperature must be higher than the target temperature. The experiment was not started.");
            return false;
        }
        if (cfg.pid_enabled && cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles) {
            QMessageBox::warning(this, "Invalid Length Mode",
                                 "Repeating cycles cannot be used when PID is enabled. Select Total Duration or After-target Hold.");
            return false;
        }
        if (cfg.use_cycling && cfg.length_mode != sonocontrol::LengthMode::TotalDuration) {
            QMessageBox::warning(this, "Invalid Cycle Mode",
                                 "Temperature cycling is only available in Total Duration mode.");
            return false;
        }
        if (cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget && !cfg.temperature_enabled) {
            QMessageBox::warning(this, "Temperature Required",
                                 "After-target Hold mode requires temperature monitoring, because the hold timer starts only after the target temperature is reached.");
            return false;
        }
        if (!cfg.simulate_us && (QString::fromStdString(cfg.com3_port).trimmed().isEmpty() || cmbCom3_->currentText().contains("no ports", Qt::CaseInsensitive))) {
            QMessageBox::critical(this, "Missing Ultrasound Port", "Select a valid COM port for the ultrasound device.");
            return false;
        }
        if (cfg.temperature_enabled && !cfg.simulate_temp && (QString::fromStdString(cfg.com11_port).trimmed().isEmpty() || cmbCom11_->currentText().contains("no ports", Qt::CaseInsensitive))) {
            const bool tempRequired = cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget;
            if (tempRequired) {
                QMessageBox::critical(this, "Missing Temperature Port", "This mode requires temperature feedback. Select a valid COM port for the HH806AU temperature sensor.");
                return false;
            }
            const auto choice = QMessageBox::warning(this, "Temperature Sensor Not Available",
                "Temperature monitoring is enabled, but no valid HH806AU COM port is selected.\n\nContinue without temperature monitoring and software cutoff for this run?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (choice != QMessageBox::Yes) return false;
            cfg.temperature_enabled = false;
            cfg.simulate_temp = false;
        }
        if (txtUdpHost_->text().trimmed().isEmpty()) {
            QMessageBox::critical(this, "Missing UDP Host", "Enter the ultrasound device UDP host address.");
            return false;
        }
        if (!runHardwarePreflight(cfg)) return false;
        return showPreflightConfirmation(cfg);
    }

    bool runHardwarePreflight(sonocontrol::Config& cfg) {
        QStringList lines;
        lines << "Preflight hardware check:";
        if (!cfg.simulate_us) {
            sonocontrol::SerialPort port;
            if (!port.open(cfg.com3_port, 9600, 8, 'N', 1, 1000)) {
                QMessageBox::critical(this, "Preflight Failed",
                                      "Cannot open ultrasound serial port " + qstr(cfg.com3_port) +
                                      ".\nThe experiment was not started.");
                return false;
            }
            port.close();
            lines << "✓ Ultrasound COM opened: " + qstr(cfg.com3_port);

            sonocontrol::UdpSender udp;
            if (!udp.open(4561)) {
                QMessageBox::critical(this, "Preflight Failed",
                                      "Cannot bind UDP source port 4561. Another process may be using it.\nThe experiment was not started.");
                return false;
            }
            udp.close();
            lines << QString("✓ UDP source port bind OK; target %1:%2").arg(qstr(cfg.udp_host)).arg(cfg.udp_port);
        } else {
            lines << "✓ Ultrasound transport simulated in Debug build";
        }

        if (cfg.temperature_enabled && !cfg.simulate_temp) {
            try {
                sonocontrol::HH806AUSensor sensor(cfg.com11_port, cfg.min_plausible_temp_c, cfg.max_plausible_temp_c, cfg.max_temp_rate_c_per_s);
                std::optional<double> t1, t2;
                for (int attempt = 0; attempt < 3; ++attempt) {
                    if (attempt > 0) QThread::msleep(150);
                    auto pair = sensor.read();
                    t1 = pair.first;
                    t2 = pair.second;
                    if (t1 || t2) break;
                }
                if (!(t1 || t2)) {
                    if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
                        QMessageBox::critical(this, "Preflight Failed",
                                              "This mode requires temperature feedback, but HH806AU returned no valid data after 3 attempts.\nThe experiment was not started.");
                        return false;
                    }
                    const auto choice = QMessageBox::warning(this, "Temperature Monitor Not Ready",
                        "Temperature monitoring is enabled, but HH806AU returned no valid data after 3 attempts.\n\nContinue without temperature monitoring and software cutoff for this run?",
                        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
                    if (choice != QMessageBox::Yes) return false;
                    cfg.temperature_enabled = false;
                    cfg.simulate_temp = false;
                    lines << "• HH806AU unavailable; continuing without temperature monitoring for this run";
                } else {
                    lines << QString("✓ HH806AU read OK: T1=%1, T2=%2")
                             .arg(t1 ? QString::number(*t1, 'f', 2) + QString::fromUtf8("°C") : "N/C")
                             .arg(t2 ? QString::number(*t2, 'f', 2) + QString::fromUtf8("°C") : "N/C");
                }
            } catch (const std::exception& e) {
                if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
                    QMessageBox::critical(this, "Preflight Failed", qstr(e.what()));
                    return false;
                }
                const auto choice = QMessageBox::warning(this, "Temperature Monitor Not Ready",
                    QString("Temperature monitoring is enabled, but HH806AU could not be opened/read:\n\n") + qstr(e.what()) +
                    "\n\nContinue without temperature monitoring and software cutoff for this run?",
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
                if (choice != QMessageBox::Yes) return false;
                cfg.temperature_enabled = false;
                cfg.simulate_temp = false;
                lines << "• HH806AU unavailable; continuing without temperature monitoring for this run";
            }
        } else if (cfg.temperature_enabled && cfg.simulate_temp) {
            lines << "✓ Temperature sensor simulated in Debug build";
        } else {
            lines << "• Temperature monitoring disabled; HH806AU, PID feedback, and software cutoff are not used";
        }

        appendConsole(lines.join("\n"));
        return true;
    }

    QString configSummary(const sonocontrol::Config& cfg) const {
        const double total_s = (cfg.length_mode == sonocontrol::LengthMode::TotalDuration)
            ? cfg.total_duration_mins * 60.0
            : ((cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles)
                ? static_cast<double>(cfg.repeating) * cfg.interval_time_s
                : cfg.hold_after_target_mins * 60.0);
        QString modeText;
        if (cfg.length_mode == sonocontrol::LengthMode::TotalDuration) modeText = "Total duration";
        else if (cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles) modeText = "Repeating cycles";
        else modeText = "After target reached, hold";

        QString s;
        s += "Ultrasound output\n";
        s += QString("  Amplitude: %1\n").arg(cfg.amplitude, 0, 'f', 3);
        s += QString("  CFreq: %1 kHz\n").arg(cfg.cfreq_hz / 1000.0, 0, 'f', 1);
        s += QString("  PRF: %1 Hz\n").arg(cfg.prf_hz, 0, 'f', 0);
        s += QString("  Duty: %1 %\n").arg(cfg.duty_cycle * 100.0, 0, 'f', 2);
        s += QString("  Waveform: %1\n").arg(qstr(sonocontrol::to_string(cfg.wave_shape)));
        s += QString("  Duration: %1 ms, Interval: %2 s\n\n").arg(cfg.duration_ms).arg(cfg.interval_time_s, 0, 'f', 2);
        s += "Experiment length\n";
        s += QString("  Mode: %1\n").arg(modeText);
        if (cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
            s += QString("  Hold time after first target reach: %1 min (%2 s)\n").arg(cfg.hold_after_target_mins, 0, 'f', 1).arg(total_s, 0, 'f', 1);
            s += QString("  Target reach criterion: setpoint ± %1 °C\n").arg(cfg.target_tolerance_c, 0, 'f', 2);
            s += "  Total duration is not used in this mode.\n";
        } else if (cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles) {
            s += QString("  Repeating cycles: %1\n").arg(cfg.repeating);
            s += QString("  Estimated total: %1 s (%2 h)\n").arg(total_s, 0, 'f', 1).arg(total_s / 3600.0, 0, 'f', 2);
        } else {
            s += QString("  Planned total: %1 s (%2 h)\n").arg(total_s, 0, 'f', 1).arg(total_s / 3600.0, 0, 'f', 2);
            s += QString("  Cycling: %1\n").arg(cfg.use_cycling ? "enabled" : "disabled");
        }
        s += "\nConnections\n";
        s += QString("  COM3: %1%2\n").arg(qstr(cfg.com3_port)).arg(cfg.simulate_us ? " [SIM]" : "");
        s += QString("  UDP target: %1:%2\n").arg(qstr(cfg.udp_host)).arg(cfg.udp_port);
        s += QString("  HH806AU: %1\n\n").arg(cfg.temperature_enabled ? (qstr(cfg.com11_port) + (cfg.simulate_temp ? " [SIM]" : "")) : "not used");
        s += "Temperature / safety\n";
        s += QString("  Temperature monitoring: %1\n").arg(cfg.temperature_enabled ? "enabled" : "disabled");
        s += QString("  Temperature channel: %1%2\n")
                 .arg(cfg.temp_channel == sonocontrol::TempChannel::T1 ? "T1" : (cfg.temp_channel == sonocontrol::TempChannel::T2 ? "T2" : "Average T1+T2"))
                 .arg(cfg.temp_channel_fallback ? " [fallback enabled]" : "");
        s += QString("  Plausible range: %1–%2 °C, max rate: %3 °C/s\n")
                 .arg(cfg.min_plausible_temp_c, 0, 'f', 1)
                 .arg(cfg.max_plausible_temp_c, 0, 'f', 1)
                 .arg(cfg.max_temp_rate_c_per_s, 0, 'f', 1);
        s += QString("  PID enabled: %1%2\n").arg(cfg.pid_enabled ? "yes" : "no").arg(cfg.pid_enabled ? " [sensor required]" : "");
        s += QString("  Target setpoint: %1 °C\n").arg(cfg.pid_setpoint, 0, 'f', 1);
        s += QString("  Thermal const: %1 s%2\n")
                 .arg(cfg.pid_prediction_tau_s, 0, 'f', 1)
                 .arg(cfg.pid_prediction_tau_s <= 0.0 ? " [disabled]" : "");
        s += QString("  Prediction horizon: %1 s%2\n")
                 .arg(cfg.pid_prediction_horizon_s, 0, 'f', 1)
                 .arg(cfg.pid_prediction_horizon_s <= 0.0 ? " [current interval]" : "");
        s += "  Prediction model: T_future = T + thermal_const × dT/dt × (1 - exp(-horizon/thermal_const))\n";
        s += QString("  Config source: %1%2\n")
                 .arg(QString::fromStdString(cfg.config_source_type))
                 .arg(cfg.config_file_path.empty() ? "" : (" [" + QString::fromStdString(cfg.config_file_path) + "]"));
        s += QString("  Cutoff: %1 °C%2 (%3 samples, ≥%4 ms apart)\n\n")
                 .arg(cfg.cutoff_temp, 0, 'f', 1)
                 .arg(cfg.temperature_enabled ? "" : " [inactive]")
                 .arg(cfg.cutoff_confirm_samples)
                 .arg(cfg.cutoff_confirm_min_spacing_ms);
        s += "Long-run protection\n";
        s += QString("  Streaming CSV log: enabled, flush every 5 s\n");
        s += QString("  Persistent COM3: %1\n").arg(cfg.persistent_com3 ? "enabled" : "disabled");
        s += QString("  Communication retry: %1 attempt(s), backoff starts at %2 ms\n").arg(cfg.communication_retry_attempts).arg(cfg.communication_retry_initial_backoff_ms);
        s += QString("  Emergency STOP repeats: %1\n").arg(cfg.emergency_stop_repeats);
        return s;
    }

    bool showPreflightConfirmation(const sonocontrol::Config& cfg) {
        QDialog dlg(this);
        dlg.setWindowTitle("Preflight Confirmation");
        dlg.setModal(true);
        dlg.resize(840, 720);
        auto* layout = new QVBoxLayout(&dlg);
        auto* title = new QLabel("Review settings before starting");
        title->setStyleSheet("font-size:18px; font-weight:700;");
        layout->addWidget(title);

        auto* summary = new QPlainTextEdit;
        summary->setReadOnly(true);
        summary->setPlainText(configSummary(cfg));
        summary->setStyleSheet("font-family:Consolas,monospace; font-size:12px;");
        layout->addWidget(summary, 1);

        auto* confirmRow = new QWidget;
        auto* confirmLayout = new QHBoxLayout(confirmRow);
        confirmLayout->setContentsMargins(0, 0, 0, 0);
        confirmLayout->setSpacing(8);
        auto* confirm = new QCheckBox;
        auto* confirmText = new QLabel("I have reviewed the settings and confirmed hardware emergency stop / timeout protection is available.");
        confirmText->setWordWrap(true);
        confirmText->setStyleSheet("font-size:13px; color:#344054;");
        confirmLayout->addWidget(confirm, 0, Qt::AlignTop);
        confirmLayout->addWidget(confirmText, 1);
        layout->addWidget(confirmRow);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        buttons->button(QDialogButtonBox::Ok)->setText("Start Experiment");
        buttons->button(QDialogButtonBox::Ok)->setEnabled(false);
        connect(confirm, &QCheckBox::toggled, buttons->button(QDialogButtonBox::Ok), &QWidget::setEnabled);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        layout->addWidget(buttons);
        return dlg.exec() == QDialog::Accepted;
    }

    QString autosaveExperimentArtifacts() {
        try {
            logger_.flush();
            std::filesystem::path csv = logger_.csv_path();
            std::string session = csv.stem().string();
            const std::string suffix = "_log";
            if (session.size() > suffix.size() && session.substr(session.size() - suffix.size()) == suffix) {
                session.erase(session.size() - suffix.size());
            }
            if (session.empty()) session = "session";
            const std::filesystem::path baseDir = config_.auto_save_dir.empty()
                ? (std::filesystem::current_path() / "experiments")
                : std::filesystem::path(config_.auto_save_dir);
            const std::filesystem::path outDir = baseDir / session;
            std::filesystem::create_directories(outDir);

            const std::filesystem::path outCsv = outDir / "experiment_log.csv";
            logger_.export_csv_file(outCsv);
            const std::filesystem::path meta = logger_.meta_path();
            if (!meta.empty() && std::filesystem::exists(meta)) {
                std::filesystem::copy_file(meta, outDir / "experiment_meta.json", std::filesystem::copy_options::overwrite_existing);
            }

            if (tempPlot_) {
                QPixmap tempPix(tempPlot_->size());
                tempPlot_->render(&tempPix);
                tempPix.save(qstr((outDir / "temperature_plot.png").string()));
            }
            if (waveformPlot_) {
                QPixmap wavePix(waveformPlot_->size());
                waveformPlot_->render(&wavePix);
                wavePix.save(qstr((outDir / "waveform_plot.png").string()));
            }
            appendConsole("Auto-saved completed experiment artifacts to: " + qstr(outDir.string()));
            return qstr(outDir.string());
        } catch (const std::exception& e) {
            QMessageBox::warning(this, "Auto-save Warning",
                                 "The experiment completed, but automatic artifact saving failed:\n\n" + qstr(e.what()));
            return QString();
        }
    }

    void showExperimentSummary(int code) {
        logger_.flush();
        const QString status = (code == 0) ? "completed" : (code == 2 ? "stopped by safety cutoff" : (code == 3 ? "stopped manually" : "aborted"));
        QString msg;
        msg += QString("Experiment %1.\n\n").arg(status);
        msg += qstr(logger_.error_summary_text());
        msg += "\nLog file:\n" + qstr(logger_.csv_path().string());
        msg += "\n\nMetadata file:\n" + qstr(logger_.meta_path().string());
        if (!lastAutoSaveDir_.isEmpty()) {
            msg += "\n\nCompleted experiment folder:\n" + lastAutoSaveDir_;
            msg += "\n\nSaved files: experiment_log.csv, experiment_meta.json, temperature_plot.png, waveform_plot.png";
        }
        if (logger_.error_count() == 0 && code == 0) {
            QMessageBox::information(this, "Experiment Summary", msg);
        } else {
            QMessageBox::warning(this, "Experiment Summary", msg);
        }
    }


    void setComboData(QComboBox* combo, const QString& text) {
        if (!combo) return;
        int idx = combo->findData(text);
        if (idx < 0) {
            idx = combo->findText(text);
        }
        if (idx < 0) {
            combo->addItem(text, text);
            idx = combo->count() - 1;
        }
        combo->setCurrentIndex(idx);
    }

    void applyConfigToUi(const sonocontrol::Config& cfg) {
        spnAmp_->setValue(cfg.amplitude);
        spnCfreq_->setValue(cfg.cfreq_hz / 1000.0);
        spnPrf_->setValue(cfg.prf_hz);
        spnDuty_->setValue(cfg.duty_cycle * 100.0);
        cmbWave_->setCurrentIndex(cfg.wave_shape == sonocontrol::WaveShape::Square ? 1 : (cfg.wave_shape == sonocontrol::WaveShape::Triangle ? 2 : 0));
        spnDuration_->setValue(cfg.duration_ms);
        spnInterval_->setValue(cfg.interval_time_s);
        if (cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) rbTargetHold_->setChecked(true);
        else if (cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles) rbRepeating_->setChecked(true);
        else rbTotalDur_->setChecked(true);
        spnTotalDur_->setValue(cfg.total_duration_mins);
        spnRepeating_->setValue(cfg.repeating);
        spnTargetHoldMin_->setValue(cfg.hold_after_target_mins);
        spnTargetTol_->setValue(cfg.target_tolerance_c);
        spnSampleRate_->setValue(cfg.sampling_rate_hz);
        spnCutoff_->setValue(cfg.cutoff_temp);
        if (spnTempMin_) spnTempMin_->setValue(cfg.min_plausible_temp_c);
        if (spnTempMax_) spnTempMax_->setValue(cfg.max_plausible_temp_c);
        if (spnTempRate_) spnTempRate_->setValue(cfg.max_temp_rate_c_per_s);
        if (chkTempFallback_) chkTempFallback_->setChecked(cfg.temp_channel_fallback);
        if (chkTempMonitor_) chkTempMonitor_->setChecked(cfg.temperature_enabled && !cfg.pid_enabled && cfg.length_mode != sonocontrol::LengthMode::HoldAfterTarget);
        chkPid_->setChecked(cfg.pid_enabled);
        spnSetpoint_->setValue(cfg.pid_setpoint);
        chkPidAmp_->setChecked(cfg.pid_amplitude);
        chkPidDuration_->setChecked(cfg.pid_duration);
        chkPidDuty_->setChecked(cfg.pid_duty);
        chkPidInterval_->setChecked(cfg.pid_interval);
        spnKp_->setValue(cfg.pid_kp);
        spnKi_->setValue(cfg.pid_ki);
        spnKd_->setValue(cfg.pid_kd);
        spnTau_->setValue(cfg.pid_prediction_tau_s);
        if (spnHorizon_) spnHorizon_->setValue(cfg.pid_prediction_horizon_s);
        autoSaveDir_ = qstr(cfg.auto_save_dir);
        chkCycling_->setChecked(cfg.use_cycling);
        spnHeatS_->setValue(cfg.heating_s);
        spnCoolS_->setValue(cfg.cooling_s);
        if (cfg.cooling_mode == sonocontrol::CoolingMode::Low) rbCoolLow_->setChecked(true); else rbCoolStop_->setChecked(true);
        spnCoolHoldTemp_->setValue(cfg.cooling_hold_temp);
        setComboData(cmbCom3_, qstr(cfg.com3_port));
        setComboData(cmbCom11_, qstr(cfg.com11_port));
        cmbTempChannel_->setCurrentIndex(static_cast<int>(cfg.temp_channel));
        txtUdpHost_->setText(qstr(cfg.udp_host));
        spnUdpPort_->setValue(static_cast<int>(cfg.udp_port));
#if SONOCONTROL_DEBUG_SIM
        if (chkSimTemp_) chkSimTemp_->setChecked(cfg.simulate_temp);
        if (chkSimUs_) chkSimUs_->setChecked(cfg.simulate_us);
#endif
        updateRepeatingFromDuration();
        updateLengthModeUi();
        onPidChanged(chkPid_->isChecked());
        onTempMonitoringChanged(chkTempMonitor_ && chkTempMonitor_->isChecked());
    }

    void attachActiveConfigProvenance(sonocontrol::Config& cfg) const {
        cfg.config_source_type = activeConfigType_.isEmpty() ? QString("gui-default").toStdString() : activeConfigType_.toStdString();
        cfg.config_file_path = activeConfigPath_.toStdString();
    }

    void updateConfigStatus() {
        if (!lblConfigStatus_) return;
        QString label = "Config: " + (activeConfigType_.isEmpty() ? QString("gui-default") : activeConfigType_);
        if (!activeConfigPath_.isEmpty()) label += "\n" + activeConfigPath_;
        lblConfigStatus_->setText(label);
    }

    void deleteTemporaryConfigFile() {
        if (!tempConfigPath_.isEmpty()) {
            QFile::remove(tempConfigPath_);
            tempConfigPath_.clear();
            if (activeConfigType_ == "temporary-modified-config") {
                activeConfigType_ = loadedConfigPath_.isEmpty() ? "gui-default" : "loaded-config";
                activeConfigPath_ = loadedConfigPath_;
            }
        }
    }

    QString ensureTemporaryConfigPath() {
        if (!tempConfigPath_.isEmpty()) return tempConfigPath_;
        QDir dir(QDir::current().filePath("temp_configs"));
        if (!dir.exists()) dir.mkpath(".");
        const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
        tempConfigPath_ = dir.filePath("temporary_modified_" + stamp + ".config");
        return tempConfigPath_;
    }

    void markConfigChanged() {
        if (loadingConfigUi_ || running_) return;
        // Keep the idle status probe in sync with whatever the user just
        // edited. Done before the try/catch so the probe sees changes even
        // if the temp .config write below fails.
        pushProbeSettings();
        if (loadedConfigPath_.isEmpty()) {
            activeConfigType_ = "gui-default";
            activeConfigPath_.clear();
            updateConfigStatus();
            return;
        }
        try {
            auto cfg = buildConfig();
            validate_config(cfg);
            cfg.config_source_type = "temporary-modified-config";
            cfg.config_file_path = ensureTemporaryConfigPath().toStdString();
            sonocontrol::save_config_file(tempConfigPath_.toStdString(), cfg, true);
            activeConfigType_ = "temporary-modified-config";
            activeConfigPath_ = tempConfigPath_;
            updateConfigStatus();
        } catch (...) {
            // Avoid disrupting interactive editing; preflight will validate again.
        }
    }

    void installConfigChangeTracking() {
        for (auto* w : findChildren<QDoubleSpinBox*>()) {
            connect(w, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ markConfigChanged(); });
        }
        for (auto* w : findChildren<QSpinBox*>()) {
            connect(w, qOverload<int>(&QSpinBox::valueChanged), this, [this](int){ markConfigChanged(); });
        }
        for (auto* w : findChildren<QCheckBox*>()) {
            connect(w, &QCheckBox::toggled, this, [this](bool){ markConfigChanged(); });
        }
        for (auto* w : findChildren<QRadioButton*>()) {
            connect(w, &QRadioButton::toggled, this, [this](bool){ markConfigChanged(); });
        }
        for (auto* w : findChildren<QComboBox*>()) {
            connect(w, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){ markConfigChanged(); });
        }
        for (auto* w : findChildren<QLineEdit*>()) {
            connect(w, &QLineEdit::textEdited, this, [this](const QString&){ markConfigChanged(); });
        }
    }

    sonocontrol::Config buildConfig() const {
        sonocontrol::Config c;
        c.amplitude = spnAmp_->value(); c.cfreq_hz = spnCfreq_->value() * 1000.0; c.prf_hz = spnPrf_->value(); c.duty_cycle = spnDuty_->value() / 100.0;
        c.wave_shape = cmbWave_->currentIndex() == 1 ? sonocontrol::WaveShape::Square : (cmbWave_->currentIndex() == 2 ? sonocontrol::WaveShape::Triangle : sonocontrol::WaveShape::Sine);
        c.duration_ms = static_cast<int>(spnDuration_->value()); c.interval_time_s = spnInterval_->value();
        if (rbTargetHold_ && rbTargetHold_->isChecked()) c.length_mode = sonocontrol::LengthMode::HoldAfterTarget;
        else if (rbRepeating_ && rbRepeating_->isChecked()) c.length_mode = sonocontrol::LengthMode::RepeatingCycles;
        else c.length_mode = sonocontrol::LengthMode::TotalDuration;
        c.use_total_duration = (c.length_mode != sonocontrol::LengthMode::RepeatingCycles);
        c.total_duration_mins = spnTotalDur_->value(); c.repeating = spnRepeating_->value();
        c.hold_after_target_mins = spnTargetHoldMin_ ? spnTargetHoldMin_->value() : 10.0;
        c.target_tolerance_c = spnTargetTol_ ? spnTargetTol_->value() : 0.3;
        c.sampling_rate_hz = spnSampleRate_->value(); c.cutoff_temp = spnCutoff_->value(); c.min_plausible_temp_c = spnTempMin_ ? spnTempMin_->value() : 10.0; c.max_plausible_temp_c = spnTempMax_ ? spnTempMax_->value() : 80.0; c.max_temp_rate_c_per_s = spnTempRate_ ? spnTempRate_->value() : 15.0;
        c.pid_enabled = chkPid_->isChecked();
        c.temperature_enabled = c.pid_enabled || (c.length_mode == sonocontrol::LengthMode::HoldAfterTarget) || (chkTempMonitor_ && chkTempMonitor_->isChecked());
        c.pid_setpoint = spnSetpoint_->value(); c.pid_amplitude = chkPidAmp_->isChecked(); c.pid_duration = chkPidDuration_->isChecked(); c.pid_duty = chkPidDuty_->isChecked(); c.pid_interval = chkPidInterval_->isChecked(); c.pid_kp = spnKp_->value(); c.pid_ki = spnKi_->value(); c.pid_kd = spnKd_->value(); c.pid_prediction_tau_s = spnTau_ ? spnTau_->value() : 25.0; c.pid_prediction_horizon_s = spnHorizon_ ? spnHorizon_->value() : 0.0; c.auto_save_dir = autoSaveDir_.toStdString();
        c.use_cycling = chkCycling_->isChecked(); c.heating_s = spnHeatS_->value(); c.cooling_s = spnCoolS_->value(); c.cooling_mode = rbCoolStop_->isChecked() ? sonocontrol::CoolingMode::Stop : sonocontrol::CoolingMode::Low; c.cooling_hold_temp = spnCoolHoldTemp_->value();
        c.com3_port = cmbCom3_->currentData().toString().toStdString(); c.com11_port = cmbCom11_->currentData().toString().toStdString(); c.temp_channel = static_cast<sonocontrol::TempChannel>(cmbTempChannel_->currentIndex()); c.temp_channel_fallback = chkTempFallback_ && chkTempFallback_->isChecked(); c.udp_host = txtUdpHost_->text().trimmed().toStdString(); c.udp_port = static_cast<uint16_t>(spnUdpPort_->value());
        c.simulate_temp = c.temperature_enabled && debugSimChecked(chkSimTemp_); c.simulate_us = debugSimChecked(chkSimUs_);
        return c;
    }

    void setupStatusProbe() {
        probeThread_ = new QThread(this);
        probe_ = new StatusProbe();
        probe_->moveToThread(probeThread_);
        connect(probeThread_, &QThread::finished, probe_, &QObject::deleteLater);
        probeThread_->start();

        probeTimer_ = new QTimer(this);
        probeTimer_->setInterval(3000);
        connect(probeTimer_, &QTimer::timeout, this, [this]() {
            if (!probe_ || running_ || shutdownInProgress_) return;
            QMetaObject::invokeMethod(probe_, "runOnce", Qt::QueuedConnection);
        });
        // Use QueuedConnection so handlers run on the GUI thread.
        connect(probe_, &StatusProbe::com3Status, this, &SonoControlWindow::onProbeCom3, Qt::QueuedConnection);
        connect(probe_, &StatusProbe::udpStatus, this, &SonoControlWindow::onProbeUdp, Qt::QueuedConnection);
        connect(probe_, &StatusProbe::tempStatus, this, &SonoControlWindow::onProbeTemp, Qt::QueuedConnection);

        pushProbeSettings();
        probeTimer_->start();
        // Kick off an initial probe immediately so the pills update on launch
        // instead of waiting 3 s for the first periodic tick.
        QMetaObject::invokeMethod(probe_, "runOnce", Qt::QueuedConnection);
    }

    void teardownStatusProbe() {
        if (probeTimer_) probeTimer_->stop();
        if (probeThread_) {
            probeThread_->quit();
            // The probe runs short operations only (each step bounded to
            // <1 s), so a 2 s wait is generous. If it does time out we still
            // proceed — the thread will be terminated at process exit since
            // the probe doesn't hold any I/O handle across calls.
            probeThread_->wait(2000);
            probeThread_ = nullptr;
            probe_ = nullptr;
        }
    }

    void pushProbeSettings() {
        if (!probe_) return;
        StatusProbe::Settings s;
        s.com3_port = cmbCom3_ ? cmbCom3_->currentData().toString() : QString();
        s.com11_port = cmbCom11_ ? cmbCom11_->currentData().toString() : QString();
        s.udp_host = txtUdpHost_ ? txtUdpHost_->text().trimmed() : QString();
        s.udp_source_port = 4561;
        // Only probe the temperature sensor when the user actually wants
        // monitoring (or PID/target-hold force it). Otherwise the pill
        // displays "Temp OFF" and we don't poll the HH806AU.
        const bool pid = chkPid_ && chkPid_->isChecked();
        const bool targetHold = rbTargetHold_ && rbTargetHold_->isChecked();
        const bool monitor = chkTempMonitor_ && chkTempMonitor_->isChecked();
        s.check_temperature = pid || targetHold || monitor;
        s.temp_min_c = spnTempMin_ ? spnTempMin_->value() : 10.0;
        s.temp_max_c = spnTempMax_ ? spnTempMax_->value() : 80.0;
        s.temp_max_rate = spnTempRate_ ? spnTempRate_->value() : 15.0;
        probe_->updateSettings(s);
    }

    void onProbeCom3(bool ok) {
        if (running_ || !lblCom3Status_) return;
        lblCom3Status_->setStyleSheet(statusStyle(ok ? "#1e8e3e" : "#d93025"));
    }

    void onProbeUdp(bool ok) {
        if (running_ || !lblUsStatus_) return;
        // Note: ok here means "we can bind the source port". If false, the
        // most likely cause is another SonoControl process still alive — see
        // the long-run notes in the change log.
        lblUsStatus_->setStyleSheet(statusStyle(ok ? "#1e8e3e" : "#d93025"));
    }

    void onProbeTemp(bool ok, double t1, double t2) {
        if (running_) return;
        // Don't touch the pill text — that's owned by updateTemperatureSensorUi.
        // We only change the color: green when reads succeed, red when the
        // user wants monitoring but the sensor isn't responding, gray when
        // monitoring is off.
        const bool pid = chkPid_ && chkPid_->isChecked();
        const bool targetHold = rbTargetHold_ && rbTargetHold_->isChecked();
        const bool monitor = chkTempMonitor_ && chkTempMonitor_->isChecked();
        const bool wants = pid || targetHold || monitor;
        if (!wants) {
            lblTempStatus_->setStyleSheet(statusStyle("#667085"));
            return;
        }
        lblTempStatus_->setStyleSheet(statusStyle(ok ? "#1e8e3e" : (pid || targetHold ? "#d93025" : "#e6930a")));
        // Mirror the latest reading into the side panel so the user sees the
        // sensor is alive before pressing Start. This was the original
        // intent of the now-removed idleTimer_, but here we do it off the
        // GUI thread so it can't lag the UI.
        if (ok && lblT1_ && lblT2_ && lblTavg_) {
            const bool has_t1 = !std::isnan(t1);
            const bool has_t2 = !std::isnan(t2);
            lblT1_->setText(has_t1 ? QString::number(t1, 'f', 1) + QString::fromUtf8("°C") : "N/C");
            lblT2_->setText(has_t2 ? QString::number(t2, 'f', 1) + QString::fromUtf8("°C") : "N/C");
            double ref = 0.0;
            int n = 0;
            if (has_t1) { ref += t1; ++n; }
            if (has_t2) { ref += t2; ++n; }
            if (n > 0) lblTavg_->setText(QString::number(ref / n, 'f', 2) + QString::fromUtf8("°C"));
        }
    }

    void checkWorkerProgress() {
        // Fires every second while an experiment is running. If we haven't
        // received any signal from the worker in `watchdog_timeout_ms`, the
        // worker is likely wedged in a synchronous serial/UDP call. Surface a
        // visible warning and auto-escalate to force_stop so the run aborts
        // cleanly instead of leaving a zombie process holding the ports.
        if (!running_ || !worker_ || !startWall_.isValid()) return;
        const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
        if (lastWorkerSignalMs_ == 0) return;
        const qint64 since = now_ms - lastWorkerSignalMs_;
        const int timeout = std::max(1000, config_.watchdog_timeout_ms);
        if (since < timeout) return;
        if (workerStallNotified_) return;
        workerStallNotified_ = true;
        appendConsole("[WARN] Worker has not reported progress for " +
                      QString::number(since / 1000.0, 'f', 1) +
                      " s. Cancelling pending I/O to recover.");
        statusBar()->showMessage("Watchdog: worker stalled, cancelling pending I/O");
        lblCom3Status_->setStyleSheet(statusStyle("#d93025"));
        lblTempStatus_->setStyleSheet(statusStyle("#d93025"));
        worker_->forceStop();
        btnStop_->setText(" ■  FORCE STOP ");
        btnStop_->setEnabled(true);
    }

    void touchWorkerSignal() {
        lastWorkerSignalMs_ = QDateTime::currentMSecsSinceEpoch();
        workerStallNotified_ = false;
    }

    void stopWorker(bool blocking) {
        idleTimer_.stop();
        sessionUiTimer_.stop();
        watchdogTimer_.stop();
        if (worker_) worker_->stop();
        if (blocking && workerThread_) {
            // Phase 1: give the worker 2 s to exit gracefully (it will send
            // the hardware STOP packets on the way out).
            if (!workerThread_->wait(2000)) {
                // Phase 2: cancel any pending serial/UDP I/O so a wedged
                // WriteFile/sendto on a USB-serial driver returns immediately.
                if (worker_) worker_->forceStop();
                // Then wait another 3 s for the now-unblocked worker to exit
                // and emit `finished`, which fires `deleteLater` on both the
                // worker and the thread. If we skip this wait, the QThread
                // would be destroyed while still running and the underlying
                // OS thread would keep the process alive with COM3/UDP
                // handles open — which is exactly the "zombie process holds
                // the COM port" symptom the user reported.
                if (!workerThread_->wait(3000)) {
                    // Last-ditch: tell Qt to terminate the OS thread. This is
                    // unsafe in general but the alternative is a process that
                    // never exits and prevents new SonoControl instances from
                    // binding the ports. Log it loudly.
                    if (workerThread_) {
                        workerThread_->terminate();
                        workerThread_->wait(1000);
                    }
                }
            }
        }
    }

    sonocontrol::ExperimentLogger logger_;
    sonocontrol::Config config_;
    QElapsedTimer startWall_;
    QTimer idleTimer_;
    QTimer sessionUiTimer_;
    double plannedTotalS_ = 0.0;
    bool targetHoldMode_ = false;
    bool targetHoldStarted_ = false;
    double targetHoldStartWallS_ = 0.0;
    double targetHoldTotalS_ = 0.0;
    std::unique_ptr<sonocontrol::ITemperatureSensor> idleSensor_;
    QCheckBox *chkDarkMode_{}, *chkAutoTheme_{};
    QTimer themeTimer_;
    bool stylesheetApplied_ = false;
    bool idle_reading_ = false;
    bool running_ = false;
    bool fatalErrorShown_ = false;
    QString lastAutoSaveDir_;
    QString loadedConfigPath_;
    QString activeConfigPath_;
    QString activeConfigType_ = "gui-default";
    QString tempConfigPath_;
    QString autoSaveDir_;
    bool loadingConfigUi_ = false;
    QThread* workerThread_ = nullptr;
    RunnerWorker* worker_ = nullptr;

    // Background status probe — runs on its own QThread. See StatusProbe and
    // setupStatusProbe(). Pointer ownership: probeThread_ is parented to
    // `this`; probe_ is deleted via deleteLater on the thread's `finished`
    // signal during teardownStatusProbe().
    QThread* probeThread_ = nullptr;
    StatusProbe* probe_ = nullptr;
    QTimer* probeTimer_ = nullptr;

    // Stall watchdog — detects when the worker thread has not reported any
    // progress signal for `config_.watchdog_timeout_ms` and triggers a
    // force-stop to recover. lastWorkerSignalMs_ is updated by every
    // worker-side slot via touchWorkerSignal().
    QTimer watchdogTimer_;
    qint64 lastWorkerSignalMs_ = 0;
    bool workerStallNotified_ = false;

    // Set true while the window is closing so the probe doesn't try to run
    // one more iteration after we've started tearing down its thread.
    bool shutdownInProgress_ = false;

    QPushButton *btnStart_{}, *btnStop_{}, *btnTestTemp_{};
    QCheckBox *chkSimTemp_{}, *chkSimUs_{}, *chkPid_{}, *chkTempMonitor_{}, *chkTempFallback_{}, *chkPidAmp_{}, *chkPidDuration_{}, *chkPidInterval_{}, *chkPidDuty_{}, *chkCycling_{}, *chkConsole_{};
    QLabel *lblCom3Status_{}, *lblUsStatus_{}, *lblTempStatus_{}, *lblPortInfo_{}, *lblTempRequirement_{}, *lblConfigStatus_{};
    QDoubleSpinBox *spnAmp_{}, *spnCfreq_{}, *spnPrf_{}, *spnDuty_{}, *spnDuration_{}, *spnInterval_{}, *spnSampleRate_{}, *spnTotalDur_{}, *spnTargetHoldMin_{}, *spnTargetTol_{}, *spnCutoff_{}, *spnSetpoint_{}, *spnKp_{}, *spnKi_{}, *spnKd_{}, *spnTau_{}, *spnHorizon_{}, *spnHeatS_{}, *spnCoolS_{}, *spnCoolHoldTemp_{}, *spnTempMin_{}, *spnTempMax_{}, *spnTempRate_{};
    QSpinBox *spnRepeating_{}, *spnUdpPort_{};
    QComboBox *cmbWave_{}, *cmbCom3_{}, *cmbCom11_{}, *cmbTempChannel_{};
    QLineEdit *txtUdpHost_{};
    QRadioButton *rbTotalDur_{}, *rbRepeating_{}, *rbTargetHold_{}, *rbCoolStop_{}, *rbCoolLow_{};
    QLabel *lblElapsed_{}, *lblRemaining_{}, *lblCycle_{}, *lblT1_{}, *lblT2_{}, *lblTavg_{}, *lblCurAmp_{}, *lblCurCfreq_{}, *lblCurPrf_{}, *lblCurDuty_{}, *lblCurDur_{}, *lblCurIntv_{};
    TemperaturePlot* tempPlot_{};
    WaveformPlot* waveformPlot_{};
    QGroupBox *grpTempSensor_{}, *grpSafety_{};
    QFrame* consoleFrame_{};
    QPlainTextEdit* console_{};
    QVector<QWidget*> pidWidgets_;
    QVector<QWidget*> tempWidgets_;
};

} // namespace

int main(int argc, char** argv) {
    qRegisterMetaType<sonocontrol::ActiveParams>("sonocontrol::ActiveParams");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");
    app.setApplicationName("SonoControl");
    app.setApplicationVersion("1.6.0-fixed-tau-config-menu");
    SonoControlWindow w;
    w.show();
    return app.exec();
}

#include "main_gui.moc"
