#include "config.hpp"
#include "config_io.hpp"
#include "experiment.hpp"
#include "logger.hpp"
#include "protocol.hpp"
#include "temperature.hpp"
#include "serial_port.hpp"
#include "udp_socket.hpp"
#include "version.hpp"

#include "hyus_protocol.hpp"
#include "hyus_net.hpp"
#include "pid.hpp"
#include "device_dialog.hpp"

#include <chrono>
#include <thread>

#if SONOCONTROL_WEB_SERVER
#include "web_server.hpp"
#endif

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QIODevice>
#include <QtCore/QMetaType>
#include <QtCore/QMimeData>
#include <QtCore/QMutex>
#include <QtCore/QSignalBlocker>
#include <QtCore/QTime>
#include <QtCore/QFileInfo>
#include <QtCore/QUrl>
#include <QtCore/QStandardPaths>
#include <QtCore/QThread>
#include <QtCore/QRegularExpression>
#include <QtCore/QTimer>
#include <QtGui/QClipboard>
#include <QtGui/QCloseEvent>
#include <QtGui/QDesktopServices>
#include <QtGui/QDragEnterEvent>
#include <QtGui/QDragMoveEvent>
#include <QtGui/QDropEvent>
#include <QtGui/QIntValidator>
#include <QtGui/QKeyEvent>
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
#include <QtWidgets/QListWidget>
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

// Set by the window's "Switch Device" action; read by main() to re-show the
// device-selection dialog after the current window closes.
bool g_switchDeviceRequested = false;

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
    // Interval floor — see sonocontrol::kMinIntervalTimeS in config.hpp.
    // The CLI's validate_config applies the same clamp to config-file and
    // --interval-s values, so the GUI spinbox limit and the headless paths
    // can't drift.
    c.interval_time_s = std::max({c.interval_time_s,
                                  static_cast<double>(c.duration_ms) / 1000.0,
                                  sonocontrol::kMinIntervalTimeS});
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
    // sampling_rate_hz was floored at >= 0.1 above, so the window floor is finite.
    c.temp_rate_window_s = std::clamp(c.temp_rate_window_s,
                                      sonocontrol::rate_window_floor_s(c.sampling_rate_hz),
                                      sonocontrol::kMaxRateWindowS);
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
        // Keep every sample for the full experiment so long runs do not lose
        // history. The painter already downsamples to ~1200 points for drawing
        // (see `step` in paintEvent), so memory — not draw time — is the only
        // cost of holding the full series. At 2 Hz that is ~173 k samples per
        // 24 h, ≈ 5.5 MB across the four vectors. Importing a CSV log relies
        // on this too: previously the buffer cap silently dropped older rows
        // and only the tail of the file was plotted.
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

// Pulse-train schematic shared by both devices. A *not-to-scale* diagram of one
// burst: pulse 1 and pulse 2 sit close together (one pulse period, solid divider
// between them); pulse 3 is the start of the next sequence, drawn after a larger
// gap with dashed dividers (an ellipsis for the rest of the burst + the off
// time). Labelled dimensions top-to-bottom: pulse amplitude, pulse length, pulse
// period, sequence length, sequence period, and (total-duration mode) repeating
// cycles. For the Zhuhai device the values are derived from PRF/duty/Duration/
// Interval; for Hyus they are the native pulse/sequence parameters.
class PulseSequencePlot final : public QWidget {
public:
    explicit PulseSequencePlot(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(300);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }
    void setParams(double ampPct, double pulseLenUs, double pulsePeriodUs,
                   double seqLenMs, double seqPeriodMs, const QString& spanLabel, bool showSpan) {
        ampPct_ = ampPct; pulseLenUs_ = pulseLenUs; pulsePeriodUs_ = pulsePeriodUs;
        seqLenMs_ = seqLenMs; seqPeriodMs_ = seqPeriodMs;
        spanLabel_ = spanLabel; showSpan_ = showSpan;
        update();
    }
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setRenderHint(QPainter::TextAntialiasing, true);
        p.fillRect(rect(), QColor(Theme::chartBg()));

        {
            const QRectF area = rect().adjusted(8, 8, -8, -8);
            constexpr double barW = 28.0;
            constexpr double pulsePeriodPx = 50.0;
            constexpr double seqLenPx = pulsePeriodPx * 2.0 + barW;
            constexpr double seqPeriodPx = 225.0;
            constexpr double finalSeqStartPx = 530.0;
            constexpr double totalPx = finalSeqStartPx + seqLenPx;

            const double sx = area.width() / totalPx;
            const double x0 = area.left();
            auto X = [&](double px) { return x0 + px * sx; };

            const double pulseTop = area.top() + 2.0;
            const double dimReserve = showSpan_ ? 118.0 : 98.0;
            const double pulseBottom = std::max(pulseTop + 92.0, area.bottom() - dimReserve);
            const double centerY = pulseTop + (pulseBottom - pulseTop) * 0.50;
            const double bw = std::max(7.0, barW * sx);

            const QColor pulseColor(Theme::isDark() ? "#60a5fa" : "#5b8fe3");
            const QColor lineColor(Theme::isDark() ? "#93c5fd" : "#4d86e8");
            const QColor labelColor(Theme::isDark() ? "#bfdbfe" : "#15599b");
            QPen guidePen(lineColor, 1.0, Qt::DashLine);
            guidePen.setDashPattern({3.0, 3.0});

            p.setPen(QPen(lineColor, 1.0));
            p.drawLine(QPointF(area.left(), centerY), QPointF(area.right(), centerY));

            p.setPen(Qt::NoPen);
            p.setBrush(pulseColor);
            const double starts[] = {0.0, seqPeriodPx, finalSeqStartPx};
            for (double seqStart : starts) {
                for (int i = 0; i < 3; ++i) {
                    const double x = X(seqStart + pulsePeriodPx * i);
                    p.drawRect(QRectF(x, pulseTop, bw, pulseBottom - pulseTop));
                }
            }

            QFont f = p.font();
            f.setPointSize(10);
            f.setBold(false);
            p.setFont(f);

            auto arrowHeadH = [&](const QPointF& tip, int dir) {
                QPolygonF tri;
                tri << tip
                    << QPointF(tip.x() + dir * 9.0, tip.y() - 5.0)
                    << QPointF(tip.x() + dir * 9.0, tip.y() + 5.0);
                p.drawPolygon(tri);
            };
            auto arrowHeadV = [&](const QPointF& tip, int dir) {
                QPolygonF tri;
                tri << tip
                    << QPointF(tip.x() - 5.0, tip.y() + dir * 9.0)
                    << QPointF(tip.x() + 5.0, tip.y() + dir * 9.0);
                p.drawPolygon(tri);
            };
            auto drawGuide = [&](double x, double y1, double y2) {
                p.setPen(guidePen);
                p.drawLine(QPointF(x, y1), QPointF(x, y2));
            };
            auto drawDimH = [&](double xa, double xb, double y, const QString& text) {
                if (xb < xa) std::swap(xa, xb);
                p.setPen(QPen(lineColor, 1.4));
                p.setBrush(lineColor);
                p.drawLine(QPointF(xa, y), QPointF(xb, y));
                arrowHeadH(QPointF(xa, y), 1);
                arrowHeadH(QPointF(xb, y), -1);
                p.setPen(labelColor);
                const double w = std::max(28.0, xb - xa);
                p.drawText(QRectF((xa + xb) / 2.0 - w / 2.0, y - 20.0, w, 18.0),
                           Qt::AlignCenter, text);
            };

            const double p0 = X(0.0);
            const double p1 = X(pulsePeriodPx);
            const double p2End = X(seqLenPx);
            const double nextSeq = X(seqPeriodPx);
            const double finalEnd = X(finalSeqStartPx + seqLenPx);
            const double yBase = pulseBottom + 10.0;
            const double rowGap = 28.0;

            drawGuide(p0, pulseBottom, area.bottom());
            drawGuide(X(barW), pulseBottom, yBase + 4.0);
            drawDimH(p0, X(barW), yBase, fmtNum(pulseLenUs_));

            drawGuide(p1, pulseBottom, yBase + rowGap + 4.0);
            drawDimH(p0, p1, yBase + rowGap, fmtNum(pulsePeriodUs_));

            drawGuide(p2End, pulseBottom, yBase + rowGap * 2.0 + 4.0);
            drawDimH(p0, p2End, yBase + rowGap * 2.0, fmtNum(seqLenMs_));

            drawGuide(nextSeq, pulseBottom, yBase + rowGap * 3.0 + 4.0);
            drawDimH(p0, nextSeq, yBase + rowGap * 3.0, fmtNum(seqPeriodMs_));

            if (showSpan_) {
                drawGuide(finalEnd, pulseBottom, yBase + rowGap * 4.0 + 4.0);
                drawDimH(p0, finalEnd, yBase + rowGap * 4.0,
                         spanLabel_.isEmpty() ? QStringLiteral("-") : spanLabel_);
            }

            const double ampX = X(seqPeriodPx + seqLenPx) + 20.0;
            p.setPen(guidePen);
            p.drawLine(QPointF(X(seqPeriodPx + seqLenPx), pulseTop), QPointF(ampX + 24.0, pulseTop));
            p.drawLine(QPointF(ampX + 24.0, centerY), QPointF(area.right(), centerY));
            p.setPen(QPen(lineColor, 1.4));
            p.setBrush(lineColor);
            p.drawLine(QPointF(ampX, pulseTop + 6.0), QPointF(ampX, centerY - 6.0));
            arrowHeadV(QPointF(ampX, pulseTop + 6.0), 1);
            arrowHeadV(QPointF(ampX, centerY - 6.0), -1);
            p.setPen(labelColor);
            p.drawText(QRectF(ampX + 12.0, (pulseTop + centerY) / 2.0 - 10.0, 64.0, 20.0),
                       Qt::AlignLeft | Qt::AlignVCenter, fmtNum(ampPct_) + "%");
        }
    }
private:
    static QString fmtNum(double v) {
        if (std::abs(v - std::round(v)) < 1e-9) return QString::number(static_cast<long long>(std::llround(v)));
        return QString::number(v, 'g', 4);
    }
    double ampPct_ = 50.0, pulseLenUs_ = 160.0, pulsePeriodUs_ = 400.0;
    double seqLenMs_ = 1.0, seqPeriodMs_ = 1000.0;
    QString spanLabel_ = QStringLiteral("60");
    bool showSpan_ = true;
};

// AC mains status: 1 = power cord plugged in (AC), 0 = running on battery,
// -1 = unknown (call failed, or a desktop without a battery).
inline int acLineStatus() {
#ifdef _WIN32
    SYSTEM_POWER_STATUS sps{};
    if (GetSystemPowerStatus(&sps)) {
        if (sps.ACLineStatus == 0) return 0;  // offline -> on battery
        if (sps.ACLineStatus == 1) return 1;  // online  -> AC
    }
    return -1;  // 255 = unknown
#else
    return -1;
#endif
}

// Large battery icon for the preflight dialog: green when on AC mains, yellow
// when on battery (so the operator notices the PC could die mid-run), grey when
// unknown.
class BatteryIndicator final : public QWidget {
public:
    explicit BatteryIndicator(QWidget* parent = nullptr) : QWidget(parent) {
        setFixedSize(150, 66);
    }
    void setStatus(int s) { if (s != status_) { status_ = s; update(); } }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        const bool ac = (status_ == 1);
        const bool unknown = (status_ < 0);
        const QColor fill = unknown ? QColor("#98a2b3") : (ac ? QColor("#1e8e3e") : QColor("#f5b301"));

        const QRectF body(6, 8, 104, 38);
        const QRectF cap(110, 20, 10, 14);
        p.setPen(QPen(QColor("#344054"), 3));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(body, 6, 6);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor("#344054"));
        p.drawRoundedRect(cap, 2, 2);
        p.setBrush(fill);
        p.drawRoundedRect(body.adjusted(5, 5, -5, -5), 4, 4);

        p.setPen(QColor("#ffffff"));
        QFont f = p.font(); f.setBold(true); f.setPointSize(12); p.setFont(f);
        p.drawText(body, Qt::AlignCenter, ac ? "AC" : (unknown ? "?" : "BATT"));

        p.setPen(QColor(unknown ? "#667085" : (ac ? "#1e8e3e" : "#b54708")));
        QFont cf = p.font(); cf.setBold(true); cf.setPointSize(8); p.setFont(cf);
        p.drawText(QRectF(0, 48, width(), 16), Qt::AlignCenter,
                   ac ? "Power plugged in" : (unknown ? "Power: unknown" : "ON BATTERY"));
    }

private:
    int status_ = -1;
};

// Apple-style PIN code entry: a row of N separate digit boxes (visible
// digits, no echo masking). Each box accepts one digit; entering one auto-
// advances focus to the next; Backspace on an empty box jumps back and
// clears the previous digit; arrow keys navigate; paste fills as many
// boxes as the clipboard provides. Two sizes — compact for inline forms
// (preflight) and "large" for the stop confirmation dialog.
class PinEntry final : public QWidget {
    Q_OBJECT
public:
    explicit PinEntry(int digits, bool large, QWidget* parent = nullptr)
        : QWidget(parent) {
        auto* h = new QHBoxLayout(this);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(large ? 12 : 8);
        h->setAlignment(Qt::AlignCenter);
        boxes_.reserve(static_cast<size_t>(digits));
        const int side = large ? 64 : 40;
        const int radius = large ? 14 : 10;
        const int fontPx = large ? 32 : 22;
        const QString style = QString(
            "QLineEdit { background:#ffffff; color:#101828; border:1px solid #d0d5dd; "
            "border-radius:%1px; padding:0; min-width:%2px; max-width:%2px; "
            "min-height:%2px; max-height:%2px; font-size:%3px; font-weight:600; }"
            "QLineEdit:focus { border:2px solid #00897b; }"
            "QLineEdit:disabled { background:#f3f5f8; color:#98a2b3; }"
        ).arg(radius).arg(side).arg(fontPx);
        for (int i = 0; i < digits; ++i) {
            auto* e = new QLineEdit;
            e->setMaxLength(1);
            e->setValidator(new QIntValidator(0, 9, e));
            e->setAlignment(Qt::AlignCenter);
            e->setStyleSheet(style);
            // Visible digits — explicitly NOT password mode. Operators
            // and the lab partner standing next to them both need to see
            // the digits when shouting the PIN across the bench.
            e->setEchoMode(QLineEdit::Normal);
            e->installEventFilter(this);
            connect(e, &QLineEdit::textChanged, this, [this, i](const QString& s) { onTextChanged(i, s); });
            h->addWidget(e);
            boxes_.push_back(e);
        }
    }

    QString pin() const {
        QString s;
        for (auto* b : boxes_) s += b->text();
        return s;
    }

    void clearPin() {
        for (auto* b : boxes_) b->clear();
        focusFirst();
    }

    void setFromString(const QString& src) {
        for (auto* b : boxes_) b->clear();
        int wrote = 0;
        for (int i = 0; i < src.size() && wrote < static_cast<int>(boxes_.size()); ++i) {
            if (src[i].isDigit()) {
                boxes_[static_cast<size_t>(wrote)]->setText(QString(src[i]));
                ++wrote;
            }
        }
        if (wrote < static_cast<int>(boxes_.size())) {
            boxes_[static_cast<size_t>(wrote)]->setFocus();
        } else if (!boxes_.empty()) {
            boxes_.back()->setFocus();
        }
    }

    void focusFirst() {
        for (auto* b : boxes_) {
            if (b->text().isEmpty()) { b->setFocus(); return; }
        }
        if (!boxes_.empty()) boxes_.back()->setFocus();
    }

    void setEntryEnabled(bool on) {
        for (auto* b : boxes_) b->setEnabled(on);
    }

signals:
    void changed();
    // Emitted once when the final digit completes the code. Useful for
    // auto-submit, but the dialog still requires an explicit OK click to
    // commit so a mistyped digit can be corrected before submission.
    void completed(const QString& pin);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override {
        if (event->type() == QEvent::KeyPress) {
            auto* ke = static_cast<QKeyEvent*>(event);
            for (size_t i = 0; i < boxes_.size(); ++i) {
                if (watched != boxes_[i]) continue;
                if (ke->key() == Qt::Key_Backspace && boxes_[i]->text().isEmpty() && i > 0) {
                    boxes_[i - 1]->setFocus();
                    boxes_[i - 1]->clear();
                    return true;
                }
                if (ke->key() == Qt::Key_Left && i > 0) {
                    boxes_[i - 1]->setFocus();
                    return true;
                }
                if (ke->key() == Qt::Key_Right && i + 1 < boxes_.size()) {
                    boxes_[i + 1]->setFocus();
                    return true;
                }
                if (ke->matches(QKeySequence::Paste)) {
                    setFromString(QApplication::clipboard()->text());
                    return true;
                }
                break;
            }
        }
        return QWidget::eventFilter(watched, event);
    }

private:
    void onTextChanged(int i, const QString& s) {
        if (!s.isEmpty() && i + 1 < static_cast<int>(boxes_.size())) {
            boxes_[static_cast<size_t>(i) + 1]->setFocus();
            boxes_[static_cast<size_t>(i) + 1]->selectAll();
        }
        emit changed();
        const QString full = pin();
        if (full.size() == static_cast<int>(boxes_.size())) {
            emit completed(full);
        }
    }

    std::vector<QLineEdit*> boxes_;
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
    // after a graceful stop request. Latches an operator (manual) stop.
    void forceStop() {
        if (auto* r = runnerRaw_.load(std::memory_order_acquire)) r->force_stop();
    }

    // Watchdog variant: same I/O cancellation, but latches an automatic
    // comms-stall stop (code 4) instead of a manual one. Called only by the
    // stall watchdog so its recovery is not mis-reported as an operator stop.
    void watchdogStop() {
        if (auto* r = runnerRaw_.load(std::memory_order_acquire)) r->watchdog_stop();
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

// Active control loop for a Hyus run: temperature sampling, safety cutoff, PID,
// and heating/cooling cycling. Unlike the FPGA ExperimentRunner it does NOT
// re-transmit a waveform — the device self-clocks, so PID/cycle act by sending
// single live parameter/run frames over the held TCP connection while the run
// continues uninterrupted. Runs on its own QThread; sends via HyusDiscovery
// (owned by the window, which stops this worker before tearing the discovery
// down). Reuses PIDController and the shared temperature sensors.
class HyusRunWorker final : public QObject {
    Q_OBJECT
public:
    HyusRunWorker(sonocontrol::Config cfg, std::string ip, sonocontrol::hyus::HyusDiscovery* disc)
        : cfg_(std::move(cfg)), ip_(std::move(ip)), disc_(disc) {}
    void requestStop() { stop_.store(true); }

public slots:
    void run() {
        using namespace sonocontrol;
        namespace hp = sonocontrol::hyus;

        std::unique_ptr<ITemperatureSensor> sensor;
        if (!cfg_.temperature_enabled) sensor = std::make_unique<NullTemperatureSensor>();
        else if (cfg_.simulate_temp) sensor = std::make_unique<TemperatureSimulator>();
        else sensor = std::make_unique<HH806AUSensor>(cfg_.com11_port, cfg_.min_plausible_temp_c,
                                                      cfg_.max_plausible_temp_c, cfg_.max_temp_rate_c_per_s);

        auto sendParam = [&](hp::Cmd cmd, uint32_t v) {
            const auto f = hp::write_param(cmd, v);
            if (disc_) disc_->send(ip_, f.data(), f.size());
        };

        PIDController pid(cfg_.pid_kp, cfg_.pid_ki, cfg_.pid_kd);

        const double seq_period_s = cfg_.hyus_seq_period_ms / 1000.0;
        const double total_s = (cfg_.length_mode == LengthMode::TotalDuration)
            ? cfg_.total_duration_mins * 60.0
            : (cfg_.length_mode == LengthMode::RepeatingCycles
                ? static_cast<double>(cfg_.repeating) * seq_period_s : 0.0);
        const double hold_after_target_s = std::max(0.0, cfg_.hold_after_target_mins * 60.0);
        const double target_tolerance = std::max(0.0, cfg_.target_tolerance_c);
        bool target_hold_started = false;
        auto target_hold_start = std::chrono::steady_clock::now();
        const double sampling_period = 1.0 / std::max(0.1, cfg_.sampling_rate_hz);
        const bool cycling = cfg_.use_cycling && cfg_.length_mode == LengthMode::TotalDuration;

        // Upper limits for the selected PID knob (configured value = full scale).
        const double ampUpperPct = std::clamp(cfg_.amplitude * 100.0, 0.0, 100.0);
        const double pulseDutyUpper = (cfg_.hyus_pulse_period_us > 0.0)
            ? std::clamp(cfg_.hyus_pulse_len_us / cfg_.hyus_pulse_period_us, 0.0, 1.0) : 0.0;
        const double seqDutyUpper = (cfg_.hyus_seq_period_ms > 0.0)
            ? std::clamp(cfg_.hyus_seq_len_ms / cfg_.hyus_seq_period_ms, 0.0, 1.0) : 0.0;

        auto applyDemand = [&](double demand) {
            demand = std::clamp(demand, 0.0, 1.0);
            switch (cfg_.hyus_pid_var) {
                case 0: sendParam(hp::Cmd::Amplitude, hp::encode_amplitude_percent(demand * ampUpperPct)); break;
                case 2: sendParam(hp::Cmd::SeqLen, hp::encode_seq_len_ms(demand * seqDutyUpper * cfg_.hyus_seq_period_ms)); break;
                default: sendParam(hp::Cmd::PulseLen, hp::encode_pulse_len_us(demand * pulseDutyUpper * cfg_.hyus_pulse_period_us)); break;
            }
        };

        const auto start = std::chrono::steady_clock::now();
        auto nextSample = start;
        auto phaseStart = start;
        enum class Phase { Heating, Cooling } phase = Phase::Heating;
        int overCutoff = 0;
        emit cycle(cfg_.length_mode == LengthMode::HoldAfterTarget ? QStringLiteral("WAIT TARGET")
                                                                    : QStringLiteral("HEATING"));

        while (!stop_.load()) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - start).count();
            double remaining = std::max(0.0, total_s - elapsed);
            if (cfg_.length_mode == LengthMode::HoldAfterTarget) {
                if (target_hold_started) {
                    const double held = std::chrono::duration<double>(now - target_hold_start).count();
                    remaining = std::max(0.0, hold_after_target_s - held);
                    if (held >= hold_after_target_s) break;
                } else {
                    remaining = hold_after_target_s;
                }
            } else if (total_s > 0.0 && elapsed >= total_s) {
                break;
            }

            if (cfg_.temperature_enabled && now >= nextSample) {
                nextSample = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(sampling_period));
                const auto [t1, t2] = sensor->read();
                std::optional<double> ctrl;
                if (cfg_.temp_channel == TempChannel::T1) ctrl = t1 ? t1 : (cfg_.temp_channel_fallback ? t2 : std::nullopt);
                else if (cfg_.temp_channel == TempChannel::T2) ctrl = t2 ? t2 : (cfg_.temp_channel_fallback ? t1 : std::nullopt);
                else { if (t1 && t2) ctrl = (*t1 + *t2) / 2.0; else ctrl = t1 ? t1 : t2; }

                emit temperature(t1 ? *t1 : std::numeric_limits<double>::quiet_NaN(),
                                 t2 ? *t2 : std::numeric_limits<double>::quiet_NaN());

                if (ctrl) {
                    if (*ctrl >= cfg_.cutoff_temp) {
                        if (++overCutoff >= std::max(1, cfg_.cutoff_confirm_samples)) {
                            sendParam(hp::Cmd::Run, 0u);
                            emit console(QString("[SAFETY] Cutoff at %1 C — stopping.").arg(*ctrl, 0, 'f', 2));
                            emit cutoff(*ctrl);
                            emit finished(2);
                            return;
                        }
                    } else {
                        overCutoff = 0;
                    }
                    if (cfg_.length_mode == LengthMode::HoldAfterTarget && !target_hold_started) {
                        if (std::abs(*ctrl - cfg_.pid_setpoint) <= target_tolerance) {
                            target_hold_started = true;
                            target_hold_start = now;
                            emit console(QString("Target reached: %1 C is within +/- %2 C of setpoint %3 C. Hold timer started for %4 s.")
                                             .arg(*ctrl, 0, 'f', 2)
                                             .arg(target_tolerance, 0, 'f', 2)
                                             .arg(cfg_.pid_setpoint, 0, 'f', 2)
                                             .arg(hold_after_target_s, 0, 'f', 1));
                            emit cycle(QStringLiteral("TARGET HOLD"));
                        }
                    }
                    const bool heatingPid = cfg_.pid_enabled && phase == Phase::Heating;
                    const bool coolHoldPid = cfg_.pid_enabled && cycling && phase == Phase::Cooling
                                             && cfg_.cooling_mode == CoolingMode::Low;
                    if (heatingPid || coolHoldPid) {
                        const double sp = (phase == Phase::Cooling) ? cfg_.cooling_hold_temp : cfg_.pid_setpoint;
                        applyDemand(pid.compute(sp, *ctrl));
                    }
                }
            }

            if (cycling) {
                const double pe = std::chrono::duration<double>(now - phaseStart).count();
                if (phase == Phase::Heating && pe >= cfg_.heating_s) {
                    phase = Phase::Cooling; phaseStart = now; pid.reset();
                    if (cfg_.cooling_mode == CoolingMode::Stop) { sendParam(hp::Cmd::Run, 0u); emit cycle(QStringLiteral("COOLING")); }
                    else emit cycle(QStringLiteral("COOLING HOLD"));
                } else if (phase == Phase::Cooling && pe >= cfg_.cooling_s) {
                    phase = Phase::Heating; phaseStart = now; pid.reset();
                    if (cfg_.cooling_mode == CoolingMode::Stop) sendParam(hp::Cmd::Run, 1u);  // resume output
                    emit cycle(QStringLiteral("HEATING"));
                }
            }

            emit timeUpdate(elapsed, remaining);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        sendParam(hp::Cmd::Run, 0u);  // stop on normal end or user stop
        emit finished(stop_.load() ? 3 : 0);
    }

signals:
    void console(QString);
    void temperature(double, double);
    void timeUpdate(double, double);
    void cycle(QString);
    void cutoff(double);
    void finished(int);

private:
    sonocontrol::Config cfg_;
    std::string ip_;
    sonocontrol::hyus::HyusDiscovery* disc_ = nullptr;
    std::atomic<bool> stop_{false};
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

// One entry in the experiment sequence: a parsed Config plus the file path it
// was loaded from. The path is kept only for display; once the sequence is
// armed it operates on the parsed Config and is not re-read at run time, so a
// user editing the file mid-sequence cannot change a queued experiment.
struct SequenceItem {
    QString path;
    sonocontrol::Config cfg;
};

// Scrollable container that accepts external file drops (drag-and-drop of
// .config files from Explorer). Internal row reordering is handled by per-row
// up/down buttons instead of QListWidget DnD, because the sequence layout
// interleaves intervals between configs and dragging a row would otherwise
// have ambiguous semantics with respect to the surrounding interval boxes.
//
// Important Qt subtlety: QScrollArea's `viewport()` is a plain QWidget that
// does NOT have acceptDrops set by default, and Qt only delivers drag/drop
// events to widgets that explicitly accept them. Setting setAcceptDrops(true)
// on the QScrollArea alone is therefore *insufficient* — drops on the visible
// scroll area would silently be rejected because they're targeted at the
// viewport. We work around this by enabling drops on the viewport too and
// installing an event filter that hoists the drag/drop events back up to the
// QScrollArea-level overrides.
class SequenceDropArea final : public QScrollArea {
    Q_OBJECT
public:
    explicit SequenceDropArea(QWidget* parent = nullptr) : QScrollArea(parent) {
        setAcceptDrops(true);
        setWidgetResizable(true);
        viewport()->setAcceptDrops(true);
        viewport()->installEventFilter(this);
    }
signals:
    void filesDropped(const QStringList& paths);
protected:
    bool eventFilter(QObject* obj, QEvent* ev) override {
        if (obj == viewport()) {
            switch (ev->type()) {
                case QEvent::DragEnter:
                case QEvent::DragMove: {
                    auto* de = static_cast<QDragMoveEvent*>(ev);  // QDragEnter inherits from QDragMove
                    if (de->mimeData() && de->mimeData()->hasUrls()) {
                        de->acceptProposedAction();
                        return true;
                    }
                    break;
                }
                case QEvent::Drop: {
                    auto* de = static_cast<QDropEvent*>(ev);
                    if (de->mimeData() && de->mimeData()->hasUrls()) {
                        QStringList paths;
                        for (const QUrl& url : de->mimeData()->urls()) {
                            const QString local = url.toLocalFile();
                            if (!local.isEmpty()) paths << local;
                        }
                        if (!paths.isEmpty()) {
                            de->acceptProposedAction();
                            emit filesDropped(paths);
                            return true;
                        }
                    }
                    break;
                }
                default: break;
            }
        }
        return QScrollArea::eventFilter(obj, ev);
    }
    void dragEnterEvent(QDragEnterEvent* e) override {
        if (e->mimeData() && e->mimeData()->hasUrls()) { e->acceptProposedAction(); return; }
        QScrollArea::dragEnterEvent(e);
    }
    void dragMoveEvent(QDragMoveEvent* e) override {
        if (e->mimeData() && e->mimeData()->hasUrls()) { e->acceptProposedAction(); return; }
        QScrollArea::dragMoveEvent(e);
    }
    void dropEvent(QDropEvent* e) override {
        if (e->mimeData() && e->mimeData()->hasUrls()) {
            QStringList paths;
            for (const QUrl& url : e->mimeData()->urls()) {
                const QString local = url.toLocalFile();
                if (!local.isEmpty()) paths << local;
            }
            if (!paths.isEmpty()) { e->acceptProposedAction(); emit filesDropped(paths); return; }
        }
        QScrollArea::dropEvent(e);
    }
};

class SonoControlWindow;

// Modeless dialog for building and running an experiment sequence.
//
// State model:
//   items_           — N configurations in execution order.
//   intervalMinutes_ — N-1 entries (or 0 if N<2). intervalMinutes_[k] is the
//                      gap that runs AFTER items_[k] finishes and BEFORE
//                      items_[k+1] starts. Units are minutes; range 0..600.
//
// The list area is rebuilt from these two arrays on every structural change
// (add / remove / reorder). The interval QSpinBoxes are owned by the
// rebuilt widgets — their valueChanged signal writes back into
// intervalMinutes_ without triggering a rebuild, so typing inside an
// interval box does not lose focus.
class SequenceDialog final : public QDialog {
    Q_OBJECT
public:
    explicit SequenceDialog(QWidget* parent = nullptr);

    // Seed the UI from previously-saved sequence state on the parent window
    // (so closing+reopening the dialog preserves the queue).
    void seedState(const QList<SequenceItem>& items, const QList<int>& intervalsMin);
    QList<SequenceItem> items() const { return items_; }
    QList<int> intervalsMinutes() const { return intervalMinutes_; }
    bool pinEnabled() const { return chkPin_ && chkPin_->isChecked(); }
    QString pinUsername() const { return txtUser_ ? txtUser_->text().trimmed() : QString(); }
    QString pinValue() const { return pinEntry_ ? pinEntry_->pin() : QString(); }
    bool stopOnError() const { return chkStopOnError_ && chkStopOnError_->isChecked(); }

    // Called by SonoControlWindow as the run progresses. Purely visual.
    void setRunningState(bool active);
    void setStatusText(const QString& text);
    void setCurrentIndex(int idx);          // -1 when no item is currently active
    void setSequenceFinished(bool ok);

signals:
    void startRequested();
    void stopRequested();

private:
    static constexpr int kDefaultIntervalMin = 5;
    static constexpr int kMaxIntervalMin = 600;

    void rebuild();
    void importPaths(const QStringList& paths);
    void onAddFiles();
    void onClearAll();
    void onStartClicked();
    void onStopClicked();
    void removeItemAt(int i);
    void moveItemUp(int i);
    void moveItemDown(int i);
    void recomputeTotal();
    static QString itemDetail(const SequenceItem& it);
    static bool configIsTargetHold(const sonocontrol::Config& c) {
        return c.length_mode == sonocontrol::LengthMode::HoldAfterTarget;
    }
    static double configDurationSeconds(const sonocontrol::Config& c) {
        if (c.length_mode == sonocontrol::LengthMode::TotalDuration) return c.total_duration_mins * 60.0;
        if (c.length_mode == sonocontrol::LengthMode::RepeatingCycles) return static_cast<double>(c.repeating) * c.interval_time_s;
        return 0.0;
    }

    QList<SequenceItem> items_;
    QList<int> intervalMinutes_;
    bool running_ = false;
    int currentIdx_ = -1;

    SequenceDropArea* scroll_ = nullptr;
    QWidget* listContainer_ = nullptr;
    QVBoxLayout* listLayout_ = nullptr;
    QLabel* lblTotalTime_ = nullptr;
    QLabel* lblStatus_ = nullptr;
    QPushButton* btnAdd_ = nullptr;
    QPushButton* btnClear_ = nullptr;
    QPushButton* btnStart_ = nullptr;
    QPushButton* btnStop_ = nullptr;
    QCheckBox* chkPin_ = nullptr;
    QLineEdit* txtUser_ = nullptr;
    PinEntry* pinEntry_ = nullptr;
    QLabel* pinErr_ = nullptr;
    QCheckBox* chkStopOnError_ = nullptr;
};

SequenceDialog::SequenceDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("Experiment Sequence");
    setModal(false);
    setMinimumSize(1100, 760);
    resize(1180, 820);
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 18, 20, 16);
    root->setSpacing(12);

    auto* title = new QLabel("Experiment Sequence");
    title->setStyleSheet("font-size:20px; font-weight:700;");
    root->addWidget(title);

    auto* hint = new QLabel("Drag .config files into the area below, or click \"Add Configs…\". Set the interval (in minutes, up to 600) between each pair of configurations independently. Each entry runs as an independent experiment and produces its own log file.");
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#475467; font-size:12px;");
    root->addWidget(hint);

    auto* addRow = new QHBoxLayout;
    btnAdd_ = new QPushButton("Add Configs…");
    btnClear_ = new QPushButton("Clear");
    btnAdd_->setMinimumWidth(140);
    btnClear_->setMinimumWidth(120);
    addRow->addWidget(btnAdd_);
    addRow->addWidget(btnClear_);
    addRow->addStretch();
    root->addLayout(addRow);

    scroll_ = new SequenceDropArea;
    scroll_->setMinimumHeight(300);
    listContainer_ = new QWidget;
    listLayout_ = new QVBoxLayout(listContainer_);
    listLayout_->setContentsMargins(8, 8, 8, 8);
    listLayout_->setSpacing(6);
    scroll_->setWidget(listContainer_);
    root->addWidget(scroll_, 1);

    lblTotalTime_ = new QLabel("Estimated total: —");
    lblTotalTime_->setStyleSheet("color:#475467; font-size:12px;");
    root->addWidget(lblTotalTime_);

    chkStopOnError_ = new QCheckBox("Abort the rest of the sequence if a configuration fails (safety cutoff, hardware error, or watchdog stop)");
    chkStopOnError_->setToolTip("When checked, the sequence stops after any non-clean exit. When unchecked (the default), the queue keeps advancing — useful for fire-and-forget unattended runs where a single bad config shouldn't kill the batch.");
    root->addWidget(chkStopOnError_);

    auto* pinGrp = new QGroupBox("Manual-stop PIN protection (sequence)");
    auto* pinGrid = new QGridLayout(pinGrp);
    pinGrid->setHorizontalSpacing(14);
    pinGrid->setVerticalSpacing(10);
    chkPin_ = new QCheckBox("Require username + 4-digit PIN to stop this sequence manually");
    auto* lblUser = new QLabel("Username");
    txtUser_ = new QLineEdit;
    txtUser_->setMaxLength(48);
    txtUser_->setPlaceholderText("e.g. Lab Operator");
    auto* lblPin = new QLabel("4-digit PIN");
    pinEntry_ = new PinEntry(4, /*large=*/false);
    pinErr_ = new QLabel(" ");
    pinErr_->setStyleSheet("color:#d93025; font-size:12px;");
    auto* pinNote = new QLabel("If enabled, the sequence STOP button requires the PIN. There is no preflight confirmation window for sequences.");
    pinNote->setWordWrap(true);
    pinNote->setStyleSheet("color:#667085; font-size:12px;");
    pinGrid->addWidget(chkPin_, 0, 0, 1, 2);
    pinGrid->addWidget(lblUser, 1, 0);
    pinGrid->addWidget(txtUser_, 1, 1);
    pinGrid->addWidget(lblPin, 2, 0);
    pinGrid->addWidget(pinEntry_, 2, 1, Qt::AlignLeft);
    pinGrid->addWidget(pinErr_, 3, 0, 1, 2);
    pinGrid->addWidget(pinNote, 4, 0, 1, 2);
    root->addWidget(pinGrp);

    auto setPinFieldsEnabled = [this, lblUser, lblPin](bool on) {
        for (auto* w : std::initializer_list<QWidget*>{lblUser, txtUser_, lblPin}) w->setEnabled(on);
        pinEntry_->setEntryEnabled(on);
    };
    setPinFieldsEnabled(false);
    connect(chkPin_, &QCheckBox::toggled, this, [setPinFieldsEnabled](bool on){ setPinFieldsEnabled(on); });
    connect(txtUser_, &QLineEdit::textEdited, this, [this](const QString&){ pinErr_->setText(" "); });
    connect(pinEntry_, &PinEntry::changed, this, [this]{ pinErr_->setText(" "); });

    lblStatus_ = new QLabel("Idle. Add at least one configuration to start.");
    lblStatus_->setStyleSheet("font-size:12px; color:#475467;");
    lblStatus_->setWordWrap(true);
    root->addWidget(lblStatus_);

    auto* btnRow = new QHBoxLayout;
    btnStart_ = new QPushButton(" ▶  Start Sequence ");
    btnStart_->setObjectName("start");
    btnStart_->setMinimumWidth(180);
    btnStop_ = new QPushButton(" ■  Stop Sequence ");
    btnStop_->setObjectName("stop");
    btnStop_->setMinimumWidth(180);
    btnStop_->setEnabled(false);
    auto* btnClose = new QPushButton("Close");
    btnRow->addStretch();
    btnRow->addWidget(btnStart_);
    btnRow->addWidget(btnStop_);
    btnRow->addWidget(btnClose);
    root->addLayout(btnRow);

    connect(btnAdd_, &QPushButton::clicked, this, &SequenceDialog::onAddFiles);
    connect(btnClear_, &QPushButton::clicked, this, &SequenceDialog::onClearAll);
    connect(scroll_, &SequenceDropArea::filesDropped, this, &SequenceDialog::importPaths);
    connect(btnStart_, &QPushButton::clicked, this, &SequenceDialog::onStartClicked);
    connect(btnStop_, &QPushButton::clicked, this, &SequenceDialog::onStopClicked);
    connect(btnClose, &QPushButton::clicked, this, &QDialog::hide);

    rebuild();
}

void SequenceDialog::seedState(const QList<SequenceItem>& items, const QList<int>& intervalsMin) {
    items_ = items;
    intervalMinutes_ = intervalsMin;
    // Normalize: intervalMinutes_ must have exactly max(0, N-1) entries.
    const int wanted = std::max(0, static_cast<int>(items_.size()) - 1);
    while (intervalMinutes_.size() < wanted) intervalMinutes_.append(kDefaultIntervalMin);
    while (intervalMinutes_.size() > wanted) intervalMinutes_.removeLast();
    for (int& v : intervalMinutes_) v = std::clamp(v, 0, kMaxIntervalMin);
    rebuild();
}

void SequenceDialog::rebuild() {
    // Clear existing rows.
    while (auto* it = listLayout_->takeAt(0)) {
        if (auto* w = it->widget()) w->deleteLater();
        delete it;
    }

    // Normalize intervalMinutes_ length to N-1.
    const int wanted = std::max(0, static_cast<int>(items_.size()) - 1);
    while (intervalMinutes_.size() < wanted) intervalMinutes_.append(kDefaultIntervalMin);
    while (intervalMinutes_.size() > wanted) intervalMinutes_.removeLast();

    if (items_.isEmpty()) {
        auto* empty = new QLabel("Drop .config files here, or click \"Add Configs…\".");
        empty->setAlignment(Qt::AlignCenter);
        empty->setStyleSheet("color:#98a2b3; font-size:13px; padding:60px; background:transparent;");
        listLayout_->addWidget(empty);
        listLayout_->addStretch();
        if (btnStart_ && !running_) btnStart_->setEnabled(false);
        recomputeTotal();
        return;
    }

    // Per-row icon-button stylesheet. The main window's global QPushButton
    // rule sets padding:5px 14px / min-height:28px, which on a small fixed-
    // size button squashes the glyph to invisibility. The local stylesheet
    // here forces the size explicitly via min/max-width/height plus zero
    // padding, so the arrows and × render at their intended pixel size.
    const QString iconBtnSs =
        "QPushButton {"
        " padding:0; margin:0;"
        " min-width:38px; max-width:38px;"
        " min-height:34px; max-height:34px;"
        " font-family:'Segoe UI Symbol','Segoe UI',Arial,sans-serif;"
        " font-size:18px; font-weight:700;"
        " background:#ffffff; color:#1a1a2e;"
        " border:1px solid #c0c8d4; border-radius:6px; }"
        "QPushButton:hover { background:#e8f5f3; border-color:#00897b; color:#00695c; }"
        "QPushButton:pressed { background:#d3eee8; }"
        "QPushButton:disabled { color:#cbd5e1; background:#f9fafb; border-color:#e4e7ec; }";
    const QString removeBtnSs = iconBtnSs +
        "QPushButton { color:#b42318; }"
        "QPushButton:hover { color:#ffffff; background:#b42318; border-color:#b42318; }"
        "QPushButton:disabled { color:#fca5a5; }";

    for (int i = 0; i < items_.size(); ++i) {
        const QString cardBg = (running_ && i == currentIdx_) ? "#e6f4ea"
                                : (running_ && i < currentIdx_) ? "#f3f5f8"
                                : "#ffffff";
        const QString cardFg = (running_ && i < currentIdx_) ? "#98a2b3" : "#1a1a2e";
        const QString detailFg = (running_ && i < currentIdx_) ? "#98a2b3" : "#475467";
        auto* card = new QFrame;
        card->setStyleSheet(QString(
            "QFrame { background:%1; color:%2; border:1px solid #d0d5dd; border-radius:8px; }"
        ).arg(cardBg, cardFg));
        auto* h = new QHBoxLayout(card);
        h->setContentsMargins(14, 8, 12, 8);
        h->setSpacing(10);

        // Text column: two lines so the file name and the parameter detail
        // don't compete for one row of horizontal space.
        auto* textCol = new QVBoxLayout;
        textCol->setSpacing(2);
        auto* titleLbl = new QLabel(QString("%1.  %2")
                                        .arg(i + 1, 2, 10, QChar(' '))
                                        .arg(QFileInfo(items_[i].path).fileName()));
        titleLbl->setStyleSheet(QString("background:transparent; border:none; color:%1; "
                                        "font-family:'Segoe UI',Arial,sans-serif; "
                                        "font-size:13px; font-weight:600;").arg(cardFg));
        titleLbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        auto* detailLbl = new QLabel(itemDetail(items_[i]));
        detailLbl->setStyleSheet(QString("background:transparent; border:none; color:%1; "
                                         "font-family:Consolas,monospace; font-size:11px;").arg(detailFg));
        detailLbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        textCol->addWidget(titleLbl);
        textCol->addWidget(detailLbl);
        h->addLayout(textCol, 1);

        auto* btnUp = new QPushButton(QString::fromUtf8("\xE2\x86\x91"));  // ↑
        btnUp->setStyleSheet(iconBtnSs);
        btnUp->setEnabled(!running_ && i > 0);
        btnUp->setToolTip("Move up");
        connect(btnUp, &QPushButton::clicked, this, [this, i]{ moveItemUp(i); });
        h->addWidget(btnUp);

        auto* btnDown = new QPushButton(QString::fromUtf8("\xE2\x86\x93"));  // ↓
        btnDown->setStyleSheet(iconBtnSs);
        btnDown->setEnabled(!running_ && i + 1 < items_.size());
        btnDown->setToolTip("Move down");
        connect(btnDown, &QPushButton::clicked, this, [this, i]{ moveItemDown(i); });
        h->addWidget(btnDown);

        auto* btnRemove = new QPushButton(QString::fromUtf8("\xC3\x97"));  // ×
        btnRemove->setStyleSheet(removeBtnSs);
        btnRemove->setEnabled(!running_);
        btnRemove->setToolTip("Remove");
        connect(btnRemove, &QPushButton::clicked, this, [this, i]{ removeItemAt(i); });
        h->addWidget(btnRemove);

        listLayout_->addWidget(card);

        // Interval row — only between two configurations.
        if (i + 1 < items_.size()) {
            auto* gap = new QWidget;
            auto* gl = new QHBoxLayout(gap);
            gl->setContentsMargins(40, 0, 0, 0);
            gl->setSpacing(8);
            auto* arrow = new QLabel("↓");
            arrow->setStyleSheet("background:transparent; color:#98a2b3; font-size:18px; font-weight:700;");
            auto* lbl = new QLabel("Interval");
            lbl->setStyleSheet("background:transparent; color:#475467; font-size:12px;");
            auto* spn = new QSpinBox;
            spn->setRange(0, kMaxIntervalMin);
            spn->setSuffix(" min");
            spn->setValue(intervalMinutes_[i]);
            spn->setEnabled(!running_);
            spn->setMinimumWidth(120);
            // Capture by value so the lambda doesn't read a stale i if the
            // list is rebuilt while the spin box is destroyed; the spin box's
            // signal is disconnected on deleteLater() before any stale fire.
            connect(spn, qOverload<int>(&QSpinBox::valueChanged), this, [this, i](int v){
                if (i >= 0 && i < intervalMinutes_.size()) intervalMinutes_[i] = std::clamp(v, 0, kMaxIntervalMin);
                recomputeTotal();
            });
            gl->addWidget(arrow);
            gl->addWidget(lbl);
            gl->addWidget(spn);
            gl->addStretch();
            listLayout_->addWidget(gap);
        }
    }
    listLayout_->addStretch();
    if (btnStart_ && !running_) btnStart_->setEnabled(!items_.isEmpty());
    recomputeTotal();
}

void SequenceDialog::importPaths(const QStringList& paths) {
    if (running_) return;
    QStringList failed;
    for (const QString& path : paths) {
        try {
            auto cfg = sonocontrol::load_config_file(path.toStdString());
            SequenceItem item;
            item.path = path;
            item.cfg = cfg;
            items_.push_back(item);
            // Add a default interval entry whenever the count crosses into N>=2.
            if (items_.size() >= 2) intervalMinutes_.push_back(kDefaultIntervalMin);
        } catch (const std::exception& e) {
            failed << QString("%1: %2").arg(QFileInfo(path).fileName()).arg(QString::fromUtf8(e.what()));
        }
    }
    if (!failed.isEmpty()) {
        QMessageBox::warning(this, "Some Configs Failed", "The following files could not be loaded:\n\n" + failed.join("\n"));
    }
    rebuild();
}

void SequenceDialog::onAddFiles() {
    const QStringList paths = QFileDialog::getOpenFileNames(this, "Add Configurations to Sequence", QString(), "SonoControl Config (*.config);;All Files (*)");
    if (paths.isEmpty()) return;
    importPaths(paths);
}

void SequenceDialog::onClearAll() {
    if (running_) return;
    items_.clear();
    intervalMinutes_.clear();
    rebuild();
}

void SequenceDialog::removeItemAt(int i) {
    if (running_) return;
    if (i < 0 || i >= items_.size()) return;
    items_.removeAt(i);
    // Drop the interval AFTER the removed item if it exists; otherwise drop
    // the last (which was BEFORE it, i.e. the only neighbour left).
    if (i < intervalMinutes_.size()) intervalMinutes_.removeAt(i);
    else if (!intervalMinutes_.isEmpty()) intervalMinutes_.removeLast();
    rebuild();
}

void SequenceDialog::moveItemUp(int i) {
    if (running_) return;
    if (i <= 0 || i >= items_.size()) return;
    items_.swapItemsAt(i, i - 1);
    if (i - 1 < intervalMinutes_.size() && i < intervalMinutes_.size()) intervalMinutes_.swapItemsAt(i - 1, i);
    rebuild();
}

void SequenceDialog::moveItemDown(int i) {
    if (running_) return;
    if (i < 0 || i + 1 >= items_.size()) return;
    items_.swapItemsAt(i, i + 1);
    if (i < intervalMinutes_.size() && i + 1 < intervalMinutes_.size()) intervalMinutes_.swapItemsAt(i, i + 1);
    rebuild();
}

void SequenceDialog::recomputeTotal() {
    if (!lblTotalTime_) return;
    if (items_.isEmpty()) { lblTotalTime_->setText("Estimated total: —"); return; }
    bool anyTargetHold = false;
    double sumS = 0.0;
    for (const auto& it : items_) {
        if (configIsTargetHold(it.cfg)) { anyTargetHold = true; break; }
        sumS += configDurationSeconds(it.cfg);
    }
    if (anyTargetHold) {
        lblTotalTime_->setText("Estimated total: unavailable (after-target hold mode is used)");
        return;
    }
    int intervalSumMin = 0;
    for (int v : intervalMinutes_) intervalSumMin += v;
    const double totalS = sumS + static_cast<double>(intervalSumMin) * 60.0;
    lblTotalTime_->setText(QString("Estimated total: %1 (%2 experiment(s), %3 gap(s) totalling %4 min)")
                              .arg(fmt_time(totalS))
                              .arg(items_.size())
                              .arg(intervalMinutes_.size())
                              .arg(intervalSumMin));
}

void SequenceDialog::onStartClicked() {
    pinErr_->setText(" ");
    if (items_.isEmpty()) { setStatusText("Add at least one configuration before starting."); return; }
    if (chkPin_->isChecked()) {
        const QString user = txtUser_->text().trimmed();
        if (user.isEmpty()) { pinErr_->setText("Enter a username (shown on the stop prompt)."); txtUser_->setFocus(); return; }
        static const QRegularExpression kPinPattern("^[0-9]{4}$");
        if (!kPinPattern.match(pinEntry_->pin()).hasMatch()) { pinErr_->setText("Enter all 4 digits of the PIN."); pinEntry_->focusFirst(); return; }
    }
    emit startRequested();
}

void SequenceDialog::onStopClicked() { emit stopRequested(); }

void SequenceDialog::setRunningState(bool active) {
    running_ = active;
    btnAdd_->setEnabled(!active);
    btnClear_->setEnabled(!active);
    chkPin_->setEnabled(!active);
    txtUser_->setEnabled(!active && chkPin_->isChecked());
    pinEntry_->setEntryEnabled(!active && chkPin_->isChecked());
    if (chkStopOnError_) chkStopOnError_->setEnabled(!active);
    btnStart_->setEnabled(!active && !items_.isEmpty());
    btnStop_->setEnabled(active);
    if (active) {
        // Hide the PIN digits the moment the sequence locks in. The value
        // has already been captured into SonoControlWindow::sequencePin_ by
        // onSequenceStartRequested *before* this call, so clearing the
        // visible boxes is purely cosmetic — but it stops the PIN from
        // sitting on screen for the duration of an hours-long sequence (or
        // staying visible when the dialog is reopened after a finished run).
        pinEntry_->clearPin();
    }
    rebuild();
}

void SequenceDialog::setStatusText(const QString& text) {
    if (lblStatus_) lblStatus_->setText(text);
}

void SequenceDialog::setCurrentIndex(int idx) {
    currentIdx_ = idx;
    rebuild();
}

void SequenceDialog::setSequenceFinished(bool ok) {
    currentIdx_ = -1;
    setRunningState(false);
    setStatusText(ok ? "Sequence finished." : "Sequence stopped.");
}

QString SequenceDialog::itemDetail(const SequenceItem& it) {
    QString modeStr;
    QString length;
    switch (it.cfg.length_mode) {
        case sonocontrol::LengthMode::TotalDuration:
            modeStr = "TotalDur";
            length = QString::number(it.cfg.total_duration_mins, 'f', 1) + " min";
            break;
        case sonocontrol::LengthMode::RepeatingCycles:
            modeStr = "Repeat";
            length = QString("%1 × %2 s").arg(it.cfg.repeating).arg(it.cfg.interval_time_s, 0, 'f', 2);
            break;
        case sonocontrol::LengthMode::HoldAfterTarget:
            modeStr = "Hold";
            length = QString::number(it.cfg.hold_after_target_mins, 'f', 1) + " min (after target)";
            break;
    }
    return QString("%1  ·  %2  ·  amp=%3  ·  dur=%4 ms  ·  intv=%5 s")
        .arg(modeStr)
        .arg(length)
        .arg(it.cfg.amplitude, 0, 'f', 3)
        .arg(it.cfg.duration_ms)
        .arg(it.cfg.interval_time_s, 0, 'f', 2);
}

class SonoControlWindow final : public QMainWindow {
    Q_OBJECT
public:
    explicit SonoControlWindow(sonocontrol::DeviceKind deviceKind = sonocontrol::DeviceKind::SonoControlFpga,
                               QWidget* parent = nullptr)
        : QMainWindow(parent), deviceKind_(deviceKind) {
        // deviceKind_ is set before buildUi() so the tab/connect layout can be
        // tailored to the selected device as it is constructed.
        setWindowTitle(QString("SonoControl  ·  %1")
                           .arg(deviceKind_ == sonocontrol::DeviceKind::Hyus
                                    ? "Hyus (LAN) Controller"
                                    : "Ultrasound Temperature Controller"));
        setMinimumSize(1600, 900);
        // Auto-detect theme by time of day: dark from 17:00 to 08:00.
        Theme::setDark(isNightTime());
        setStyleSheet(styleSheetText());
        statusBar()->showMessage("Ready");
        buildUi();
        refreshPorts();
        updateRepeatingFromDuration();
        updateSequencePreview();
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

        // Sequence interval timer: fires once after the configured gap to
        // launch the next configuration. Single-shot is set explicitly each
        // time it is armed in handleSequenceItemFinished().
        sequenceIntervalTimer_.setSingleShot(true);
        connect(&sequenceIntervalTimer_, &QTimer::timeout, this, &SonoControlWindow::onSequenceIntervalElapsed);

        // Hyus LAN discovery: poll the background service ~1 Hz to refresh the
        // device dropdown. Discovery itself runs only for the Hyus device.
        hyusScanTimer_.setInterval(1000);
        connect(&hyusScanTimer_, &QTimer::timeout, this, &SonoControlWindow::refreshHyusDevices);
        hyusStopTimer_.setSingleShot(true);
        connect(&hyusStopTimer_, &QTimer::timeout, this, &SonoControlWindow::onHyusRunFinished);
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) startHyusDiscovery();
    }

    ~SonoControlWindow() override {
        shutdownInProgress_ = true;
        sequenceActive_ = false;
        sequenceIntervalTimer_.stop();
        hyusScanTimer_.stop();
        hyusStopTimer_.stop();
        if (hyusWorker_) hyusWorker_->requestStop();
        if (hyusWorkerThread_) { hyusWorkerThread_->quit(); hyusWorkerThread_->wait(2000); }
        sendHyusSafetyStop();  // stop output before dropping the connection
        if (hyusDiscovery_) hyusDiscovery_->stop();
        teardownStatusProbe();
#if SONOCONTROL_WEB_SERVER
        if (webServer_) webServer_->stop();
#endif
        stopWorker(true);
        deleteTemporaryConfigFile();
    }

protected:
    void closeEvent(QCloseEvent* e) override {
        shutdownInProgress_ = true;
        sequenceActive_ = false;
        sequenceIntervalTimer_.stop();
        hyusScanTimer_.stop();
        hyusStopTimer_.stop();
        if (hyusWorker_) hyusWorker_->requestStop();
        if (hyusWorkerThread_) { hyusWorkerThread_->quit(); hyusWorkerThread_->wait(2000); }
        sendHyusSafetyStop();  // stop output before dropping the connection
        if (hyusDiscovery_) hyusDiscovery_->stop();
        teardownStatusProbe();
#if SONOCONTROL_WEB_SERVER
        if (webServer_) webServer_->stop();
#endif
        stopWorker(true);
        deleteTemporaryConfigFile();
        QMainWindow::closeEvent(e);
    }

private slots:
    void onStart() {
        if (running_) return;
        if (sequenceActive_) return;  // main Start is disabled while a sequence is queued, but guard regardless
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) {
            onStartHyus();
            return;
        }
        config_ = buildConfig();
        attachActiveConfigProvenance(config_);
        validate_config(config_);
        if (!preflightCheck(config_)) return;
        launchWorkerForConfig(config_, /*fromSequence=*/false);
    }

    // Shared worker setup used by both onStart (with preflight) and the
    // sequence runner (which skips the preflight confirmation window but
    // still relies on this method to wire up the QThread + signals exactly
    // the same way).
    void launchWorkerForConfig(const sonocontrol::Config& cfg, bool fromSequence) {
        config_ = cfg;
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
        // In sequence mode the main Start/Stop stay locked out for the full
        // duration of the sequence; the sequence dialog provides its own
        // buttons. Outside sequence mode this is just the usual per-run
        // disable.
        if (!sequenceActive_) {
            btnStart_->setEnabled(false);
            btnStop_->setEnabled(true);
            btnStop_->setText(" ■  EMERGENCY STOP ");
        }
        lblCycle_->setText("IDLE");
        if (fromSequence) {
            appendConsole(QString("=== [Sequence %1/%2] Experiment started ===")
                              .arg(sequenceIndex_ + 1).arg(sequenceTotal_));
            statusBar()->showMessage(QString("Sequence %1/%2 running")
                                         .arg(sequenceIndex_ + 1).arg(sequenceTotal_));
        } else {
            appendConsole("=== Experiment started ===");
            statusBar()->showMessage("Experiment running");
        }

        if (probe_) probe_->setPaused(true);

#if SONOCONTROL_WEB_SERVER
        if (webServer_) {
            // The real session id is created on the worker thread inside
            // ExperimentRunner::run() → logger.start_session(), so we can't
            // know it here. Start with an empty id; touchWorkerSignal()
            // backfills the canonical logger id as soon as the first
            // worker-side signal arrives. This keeps the web page session_id
            // strictly aligned with the on-disk CSV/JSON filename instead of
            // a GUI-side approximation that drifted by a few ms.
            webServer_->onRunStarted(QString());
        }
#endif

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
        if (!worker_) return;
        if (pendingPinEnabled_ && !promptForStopPin()) {
            appendConsole(">>> Manual stop cancelled (PIN not confirmed)");
            statusBar()->showMessage("Manual stop cancelled");
            return;
        }
        appendConsole(">>> EMERGENCY STOP requested");
        statusBar()->showMessage("Emergency stop requested");
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
        if (pendingPinEnabled_ && !promptForStopPin()) {
            appendConsole(">>> Force-stop cancelled (PIN not confirmed)");
            statusBar()->showMessage("Force-stop cancelled");
            return;
        }
        appendConsole(">>> User requested force-stop");
        worker_->forceStop();
        btnStop_->setEnabled(false);
    }

    // Opens a modal prompt that shows the current operator name and demands
    // the 4-digit PIN configured at preflight. Returns true only on an exact
    // match. The 2 s graceful->force escalation and the watchdog do not call
    // this — they bypass the PIN so safety paths cannot be locked out.
    bool promptForStopPin() {
        QDialog dlg(this);
        dlg.setWindowTitle("Confirm Manual Stop");
        dlg.setModal(true);
        dlg.setMinimumWidth(440);
        auto* v = new QVBoxLayout(&dlg);
        v->setContentsMargins(36, 28, 36, 22);
        v->setSpacing(14);

        auto* title = new QLabel("Stop experiment?");
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet("font-size:22px; font-weight:700;");
        v->addWidget(title);

        auto* who = new QLabel(QString("Current user: <b>%1</b>").arg(pendingPinUsername_.toHtmlEscaped()));
        who->setTextFormat(Qt::RichText);
        who->setAlignment(Qt::AlignCenter);
        who->setStyleSheet("font-size:14px; color:#344054;");
        v->addWidget(who);

        auto* prompt = new QLabel("Enter the 4-digit PIN to stop the experiment");
        prompt->setAlignment(Qt::AlignCenter);
        prompt->setStyleSheet("font-size:13px; color:#475467;");
        v->addWidget(prompt);

        v->addSpacing(4);
        auto* pinEntry = new PinEntry(4, /*large=*/true);
        v->addWidget(pinEntry, 0, Qt::AlignCenter);

        auto* err = new QLabel(" ");
        err->setAlignment(Qt::AlignCenter);
        err->setStyleSheet("color:#d93025; font-size:12px; min-height:18px;");
        v->addWidget(err);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        auto* okBtn = buttons->button(QDialogButtonBox::Ok);
        okBtn->setText("Stop");
        okBtn->setEnabled(false);
        v->addWidget(buttons);

        // Enable Stop only when all four digits are present.
        connect(pinEntry, &PinEntry::changed, okBtn, [pinEntry, okBtn, err]() {
            okBtn->setEnabled(pinEntry->pin().size() == 4);
            err->setText(" ");
        });
        // Pressing the last digit submits via the same path as clicking Stop.
        connect(pinEntry, &PinEntry::completed, okBtn, [okBtn]() {
            if (okBtn->isEnabled()) okBtn->animateClick();
        });

        connect(buttons, &QDialogButtonBox::accepted, &dlg, [&, pinEntry, err]() {
            if (pinEntry->pin() != pendingPin_) {
                err->setText("Incorrect PIN.");
                pinEntry->clearPin();
                return;
            }
            dlg.accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        pinEntry->focusFirst();
        return dlg.exec() == QDialog::Accepted;
    }

    void loadConfigFile() {
        if (running_) return;
        const QString path = QFileDialog::getOpenFileName(this, "Load SonoControl config", QString(), "SonoControl Config (*.config);;All Files (*)");
        if (path.isEmpty()) return;
        try {
            auto cfg = sonocontrol::load_config_file(path.toStdString());
            // Device recognition: a config saved for one device must not be
            // loaded onto a session running the other device.
            if (cfg.device_kind != deviceKind_) {
                QMessageBox::critical(this, "Device mismatch",
                    QString("This configuration is for the '%1' device, but the current "
                            "session is running the '%2' device.\n\nStart the software and "
                            "select the matching device, then load this configuration.")
                        .arg(deviceDisplayName(cfg.device_kind))
                        .arg(deviceDisplayName(deviceKind_)));
                return;
            }
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
        if (!sequenceActive_) {
            btnStart_->setEnabled(true);
            btnStop_->setEnabled(false);
            btnStop_->setText(" ■  EMERGENCY STOP ");
        }
        // Resume idle probing now that the worker has released the ports. In
        // sequence mode the next item will pause it again moments later, but
        // resuming it here keeps the pills accurate during the inter-config
        // interval.
        if (probe_ && !shutdownInProgress_) probe_->setPaused(false);
        lblCycle_->setText(code == 0 ? "COMPLETE" : (code == 2 ? "CUTOFF" : (code == 3 ? "STOPPED" : (code == 4 ? "STALLED" : "ERROR"))));
        lblCycle_->setStyleSheet(valueStyle(code == 0 ? "#00897b" : ((code == 3 || code == 4) ? "#b54708" : (code == 2 ? "#d93025" : "#b42318"))));
        if (sequenceActive_) {
            const int oneBased = sequenceIndex_ + 1;
            const QString verb = (code == 0) ? "complete" : (code == 2 ? "stopped by safety cutoff" : (code == 3 ? "stopped manually" : (code == 4 ? "auto-stopped (comms stall)" : "aborted")));
            appendConsole(QString("=== [Sequence %1/%2] Experiment %3 ===").arg(oneBased).arg(sequenceTotal_).arg(verb));
            statusBar()->showMessage(QString("Sequence %1/%2: %3").arg(oneBased).arg(sequenceTotal_).arg(verb));
        } else if (code == 0) {
            appendConsole("=== Experiment complete ===");
            statusBar()->showMessage("Experiment complete");
        } else if (code == 2) {
            appendConsole("=== Experiment stopped by safety cutoff ===");
            statusBar()->showMessage("Safety cutoff");
        } else if (code == 3) {
            appendConsole("=== Experiment stopped manually ===");
            statusBar()->showMessage("Manual emergency stop");
        } else if (code == 4) {
            appendConsole("=== Experiment auto-stopped: device stopped responding (communication stall) ===");
            statusBar()->showMessage("Auto-stopped: communication stall");
        } else {
            appendConsole("=== Experiment aborted because of an error ===");
            statusBar()->showMessage("Experiment aborted: error");
        }
        lastAutoSaveDir_.clear();
        if (code == 0) {
            lastAutoSaveDir_ = autosaveExperimentArtifacts();
        }
        // Drop the manual-stop PIN so it does not leak across runs. The
        // sequence's own PIN lives in separate members and is not affected.
        pendingPinEnabled_ = false;
        pendingPinUsername_.clear();
        pendingPin_.clear();
#if SONOCONTROL_WEB_SERVER
        if (webServer_) webServer_->onRunFinished(code);
#endif
        if (sequenceActive_) {
            // The summary popup would interrupt the inter-config gap on every
            // item, which defeats the point of an unattended sequence. The
            // dialog status label carries the equivalent information.
            handleSequenceItemFinished(code);
        } else {
            showExperimentSummary(code);
        }
        // idleTimer_ no longer used
    }

    void onTemperature(double t1, double t2) {
        touchWorkerSignal();
        // The worker sends NaN (not 0.0) when a channel is not connected, so
        // use isnan() here rather than t1 > 0 (which would wrongly treat an
        // exact 0 °C reading as "disconnected" if min_plausible_temp_c < 0).
        const bool has_t1 = !std::isnan(t1);
        const bool has_t2 = !std::isnan(t2);
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
                const double t_s = startWall_.elapsed() / 1000.0;
                tempPlot_->append(t_s,
                                  has_t1 ? t1 : std::numeric_limits<double>::quiet_NaN(),
                                  has_t2 ? t2 : std::numeric_limits<double>::quiet_NaN(),
                                  ref);
#if SONOCONTROL_WEB_SERVER
                if (webServer_) {
                    webServer_->onSampleAvailable(t_s,
                        has_t1 ? t1 : std::numeric_limits<double>::quiet_NaN(),
                        has_t2 ? t2 : std::numeric_limits<double>::quiet_NaN(),
                        ref);
                }
#endif
            }
        } else {
            lblTavg_->setText("N/C");
        }
    }

    void onParams(sonocontrol::ActiveParams p) {
        touchWorkerSignal();
        // The Hyus device repurposes the Active Parameters labels (populated by
        // updateSequencePreview); its runtime does not emit FPGA ActiveParams.
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) return;
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
#if SONOCONTROL_WEB_SERVER
        if (webServer_) webServer_->onTimesUpdated(elapsed, remaining);
#endif
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

    void onWaveform(std::vector<float> w) { touchWorkerSignal(); if (waveformPlot_) waveformPlot_->setWaveform(std::move(w)); }

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

    // Per-cycle period in seconds used by the duration <-> repeating
    // relationship. For the FPGA box this is the transmit Interval; for Hyus it
    // is the Sequence Period (total duration = repeating × sequence period).
    double cyclePeriodSeconds() const {
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus)
            return spnHyusSeqPeriodMs_ ? spnHyusSeqPeriodMs_->value() / 1000.0 : 0.0;
        return spnInterval_ ? spnInterval_->value() : 0.0;
    }

    void updateRepeatingFromDuration() {
        const double period = cyclePeriodSeconds();
        if (period <= 0.0 || !spnRepeating_) return;
        const int reps = std::max(1, static_cast<int>((spnTotalDur_->value() * 60.0) / period));
        QSignalBlocker b(spnRepeating_);
        spnRepeating_->setValue(reps);
        updateLengthModeUi();
    }

    void updateDurationFromRepeating() {
        if (!rbRepeating_ || !rbRepeating_->isChecked()) return;
        QSignalBlocker b(spnTotalDur_);
        spnTotalDur_->setValue((spnRepeating_->value() * cyclePeriodSeconds()) / 60.0);
        updateSequencePreview();
    }

    // Refresh the pulse-sequence schematic (both devices) and, for Hyus, the
    // Active Parameters panel, from the current PARAMS-tab inputs. For Zhuhai the
    // schematic values are derived: pulse period = 1/PRF, pulse length =
    // duty/PRF, sequence length = Duration, sequence period = Interval.
    void updateSequencePreview() {
        if (!seqPlot_) return;
        const bool totalMode = rbTotalDur_ && rbTotalDur_->isChecked();
        auto fmtPreviewNum = [](double v) {
            return (std::abs(v - std::round(v)) < 1e-9) ? QString::number(static_cast<long long>(std::llround(v)))
                                                        : QString::number(v, 'g', 4);
        };
        const QString spanLabel = (totalMode && spnTotalDur_) ? fmtPreviewNum(spnTotalDur_->value() * 60.0) : QString();
        const int reps = spnRepeating_ ? spnRepeating_->value() : 0;
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) {
            const double ampPct = spnHyusAmpPct_ ? spnHyusAmpPct_->value() : 50.0;
            const double pLen = spnHyusPulseLenUs_ ? spnHyusPulseLenUs_->value() : 160.0;
            const double pPer = spnHyusPulsePeriodUs_ ? spnHyusPulsePeriodUs_->value() : 400.0;
            const double sLen = spnHyusSeqLenMs_ ? spnHyusSeqLenMs_->value() : 1.0;
            const double sPer = spnHyusSeqPeriodMs_ ? spnHyusSeqPeriodMs_->value() : 1000.0;
            seqPlot_->setParams(ampPct, pLen, pPer, sLen, sPer, spanLabel, totalMode);
            auto num = [](double v) {
                return (std::abs(v - std::round(v)) < 1e-9) ? QString::number(static_cast<long long>(std::llround(v)))
                                                            : QString::number(v, 'g', 4);
            };
            if (lblCurAmp_) lblCurAmp_->setText(num(ampPct) + "%");
            if (lblCurCfreq_) lblCurCfreq_->setText(num(pLen) + " us");
            if (lblCurPrf_) lblCurPrf_->setText(num(pPer) + " us");
            if (lblCurDuty_) lblCurDuty_->setText(num(sLen) + " ms");
            if (lblCurDur_) lblCurDur_->setText(num(sPer) + " ms");
            if (lblCurIntv_) lblCurIntv_->setText(totalMode ? (QString::number(reps) + " cyc") : QString("(set)"));
        } else {
            const double prf = (spnPrf_ && spnPrf_->value() > 0.0) ? spnPrf_->value() : 1.0;
            const double ampPct = spnAmp_ ? spnAmp_->value() * 100.0 : 50.0;
            const double pPer = 1.0e6 / prf;                                   // us
            const double pLen = (spnDuty_ ? spnDuty_->value() / 100.0 : 0.5) * 1.0e6 / prf;  // us
            const double sLen = spnDuration_ ? spnDuration_->value() : 0.0;     // ms
            const double sPer = spnInterval_ ? spnInterval_->value() * 1000.0 : 0.0;  // ms
            seqPlot_->setParams(ampPct, pLen, pPer, sLen, sPer, spanLabel, totalMode);
        }
    }

    // Live timing constraints for Hyus: length <= period (pulse and sequence)
    // and pulse period <= sequence length. Enforced as dynamic spinbox ranges so
    // invalid values cannot be entered. Signals are blocked to avoid recursion.
    void enforceHyusTiming() {
        if (!spnHyusPulseLenUs_ || !spnHyusPulsePeriodUs_ || !spnHyusSeqLenMs_ || !spnHyusSeqPeriodMs_) return;
        QSignalBlocker b1(spnHyusPulseLenUs_), b2(spnHyusPulsePeriodUs_), b3(spnHyusSeqLenMs_), b4(spnHyusSeqPeriodMs_);
        const double seqLenUs = spnHyusSeqLenMs_->value() * 1000.0;
        // pulse period in [pulse length, sequence length]
        spnHyusPulsePeriodUs_->setMinimum(spnHyusPulseLenUs_->value());
        spnHyusPulsePeriodUs_->setMaximum(std::max(spnHyusPulseLenUs_->value(), seqLenUs));
        // pulse length <= pulse period
        spnHyusPulseLenUs_->setMaximum(spnHyusPulsePeriodUs_->value());
        // sequence length in [pulse period, sequence period]
        spnHyusSeqLenMs_->setMinimum(spnHyusPulsePeriodUs_->value() / 1000.0);
        spnHyusSeqLenMs_->setMaximum(spnHyusSeqPeriodMs_->value());
        // sequence period >= sequence length
        spnHyusSeqPeriodMs_->setMinimum(spnHyusSeqLenMs_->value());
    }

    // Live timing constraints for Zhuhai: sequence length (Duration) <= sequence
    // period (Interval). validate_config already keeps interval >= duration, so
    // these ranges never alter a freshly-loaded config. (pulse length <= pulse
    // period is automatic since duty <= 100%; pulse period <= sequence length is
    // checked at preflight.)
    void enforceZhuhaiTiming() {
        if (!spnDuration_ || !spnInterval_) return;
        QSignalBlocker bD(spnDuration_), bI(spnInterval_);
        spnInterval_->setMinimum(std::max(sonocontrol::kMinIntervalTimeS, spnDuration_->value() / 1000.0));
        spnDuration_->setMaximum(spnInterval_->value() * 1000.0);
    }

    // The dT/dt smoothing window cannot be narrower than one sample period, so
    // its spinbox floor tracks the sample-rate spinbox. validate_config applies
    // the same clamp; this just keeps the GUI from offering an invalid value.
    void enforceRateWindowRange() {
        if (!spnRateWindow_ || !spnSampleRate_) return;
        QSignalBlocker b(spnRateWindow_);
        spnRateWindow_->setMinimum(sonocontrol::rate_window_floor_s(spnSampleRate_->value()));
    }

    // ---- Hyus LAN discovery / connection ----
    void startHyusDiscovery() {
        if (!hyusDiscovery_) hyusDiscovery_ = std::make_unique<sonocontrol::hyus::HyusDiscovery>();
        if (!hyusDiscovery_->start(8192, 8193)) {
            if (lblHyusStatus_) lblHyusStatus_->setText("Cannot start discovery: " +
                                                        QString::fromStdString(hyusDiscovery_->lastError()));
            return;
        }
        hyusScanTimer_.start();
        refreshHyusDevices();
    }

    void rescanHyusDevices() {
        if (!hyusDiscovery_) { startHyusDiscovery(); return; }
        // A full restart clears stale entries and re-broadcasts immediately.
        hyusDiscovery_->stop();
        if (cmbHyusDevice_) cmbHyusDevice_->clear();
        hyusKnownDevices_.clear();  // re-push init when devices reconnect
        startHyusDiscovery();
    }

    void refreshHyusDevices() {
        if (!hyusDiscovery_ || !cmbHyusDevice_) return;
        const auto devs = hyusDiscovery_->devices();
        const QString prev = cmbHyusDevice_->currentText();
        QStringList items;
        for (const auto& d : devs) items << QString::fromStdString(d);
        // Init is pushed on user selection (QComboBox::activated), not here.
        hyusKnownDevices_ = items;
        // Only rebuild when the set actually changed, to keep the user's
        // selection and avoid dropdown flicker.
        QStringList current;
        for (int i = 0; i < cmbHyusDevice_->count(); ++i) current << cmbHyusDevice_->itemText(i);
        if (items != current) {
            QSignalBlocker b(cmbHyusDevice_);
            cmbHyusDevice_->clear();
            cmbHyusDevice_->addItems(items);
            const int idx = items.indexOf(prev);
            if (idx >= 0) cmbHyusDevice_->setCurrentIndex(idx);
        }
        if (lblHyusStatus_) {
            lblHyusStatus_->setText(devs.empty()
                ? "Listening on TCP 8192 — no device connected yet."
                : QString("Connected: %1 device(s).").arg(static_cast<int>(devs.size())));
        }
        // Header LAN pill: green once at least one device is detected on the LAN.
        if (lblLanStatus_) lblLanStatus_->setStyleSheet(statusStyle(devs.empty() ? "#b54708" : "#1e8e3e"));
    }

    QString selectedHyusDevice() const {
        return cmbHyusDevice_ ? cmbHyusDevice_->currentText().trimmed() : QString();
    }

    // Send a single PREFIX+PARAM frame to the selected device immediately. Used
    // for live parameter edits (the device accepts changes whether or not it is
    // running). No-op if no device is connected.
    void sendHyusParam(sonocontrol::hyus::Cmd cmd, uint32_t value) {
        if (deviceKind_ != sonocontrol::DeviceKind::Hyus || !hyusDiscovery_) return;
        if (loadingConfigUi_) return;  // don't push frames while a config is being applied
        const QString ip = selectedHyusDevice();
        if (ip.isEmpty()) return;
        const auto f = sonocontrol::hyus::write_param(cmd, value);
        hyusDiscovery_->send(ip.toStdString(), f.data(), f.size());
    }

    // Full initial-connection parameter download (matches the 353-byte batch in
    // all.pcapng). Sent ONCE when a device first connects, to push the current
    // GUI values. Mirrors the captured order; ends with the device idle (RUN=0).
    std::vector<uint8_t> buildHyusInitBatch(const sonocontrol::Config& c) const {
        using namespace sonocontrol::hyus;
        std::vector<uint8_t> b;
        const auto init = make_init();
        b.insert(b.end(), init.begin(), init.end());
        append_param(b, Cmd::Cfreq, encode_cfreq(c.cfreq_hz));
        append_param(b, Cmd::Amplitude, encode_amplitude_percent(c.amplitude * 100.0));
        append_param(b, Cmd::PulseLen, encode_pulse_len_us(c.hyus_pulse_len_us));
        append_param(b, Cmd::PulsePeriod, encode_pulse_period_us(c.hyus_pulse_period_us));
        append_param(b, Cmd::SeqLen, encode_seq_len_ms(c.hyus_seq_len_ms));
        append_param(b, Cmd::SeqPeriod, encode_seq_period_ms(c.hyus_seq_period_ms));
        append_param(b, Cmd::TotalDuration, encode_total_duration_s(c.total_duration_mins * 60.0));
        append_param(b, Cmd::Run, 0u);
        append_param(b, Cmd::RunMode, (c.hyus_run_mode == 1) ? 1u : 0u);
        append_param(b, Cmd::Run, 0u);
        append_param(b, Cmd::Run, 0u);
        append_param(b, Cmd::TriggerSource, 0u);   // internal
        append_param(b, Cmd::Run, 0u);
        return b;
    }

    // The "Start" message — the short start sequence, NOT the init batch.
    // Internal repeat (matches the Length=170 / 104-byte frame in the captures):
    //   RUN=0, RUN=0, TRIGSRC=0, RUN=1
    // Internal total-duration: RUN=0, TRIGSRC=0, RUN=1 (run mode + total duration
    // were already pushed as live edits).
    std::vector<uint8_t> buildHyusStartSeq(const sonocontrol::Config& c) const {
        using namespace sonocontrol::hyus;
        std::vector<uint8_t> b;
        append_param(b, Cmd::Run, 0u);
        if (c.hyus_run_mode == 0) append_param(b, Cmd::Run, 0u);  // repeat mode clears twice
        append_param(b, Cmd::TriggerSource, 0u);                  // internal
        append_param(b, Cmd::Run, 1u);                            // run enable
        return b;
    }

    // Safety stop: tell every connected device to stop output. Sent when the
    // app is closing so a run cannot continue unattended after the controller
    // exits (the device otherwise keeps emitting after RUN=1 until told to stop).
    void sendHyusSafetyStop() {
        if (deviceKind_ != sonocontrol::DeviceKind::Hyus || !hyusDiscovery_) return;
        const auto stop = sonocontrol::hyus::write_param(sonocontrol::hyus::Cmd::Run, 0u);
        for (const auto& ip : hyusDiscovery_->devices())
            hyusDiscovery_->send(ip, stop.data(), stop.size());
    }

    void sendHyusInit(const QString& ip) {
        if (!hyusDiscovery_ || ip.isEmpty()) return;
        config_ = buildConfig();
        const auto batch = buildHyusInitBatch(config_);
        hyusDiscovery_->send(ip.toStdString(), batch.data(), batch.size());
        appendConsole("Hyus: pushed initial parameters to " + ip + " (" +
                      QString::number(batch.size()) + " bytes).");
    }

    void onStartHyus() {
        const QString ip = selectedHyusDevice();
        if (ip.isEmpty() || !hyusDiscovery_) {
            QMessageBox::warning(this, "No device",
                "No Hyus device detected. Check the RJ45 cable and the PC network adapter "
                "(192.168.0.x), then press Scan.");
            return;
        }
        config_ = buildConfig();
        attachActiveConfigProvenance(config_);
        validate_config(config_);
        if (!preflightCheckHyus(config_)) return;
        const auto seq = buildHyusStartSeq(config_);
        if (!hyusDiscovery_->send(ip.toStdString(), seq.data(), seq.size())) {
            QMessageBox::critical(this, "Send failed",
                "Could not send to " + ip + ". The device may have disconnected.");
            return;
        }
        running_ = true;
        btnStart_->setEnabled(false);
        btnStop_->setEnabled(true);
        btnStop_->setText(" ■  STOP ");
        appendConsole("=== Hyus run started on " + ip + " ===");
        appendConsole("Sent start sequence (" + QString::number(seq.size()) + " bytes: clear + internal trigger + run).");
        statusBar()->showMessage("Hyus running on " + ip);
        lblCycle_->setText("RUNNING");
        startWall_.start();
        targetHoldMode_ = (config_.length_mode == sonocontrol::LengthMode::HoldAfterTarget);
        targetHoldStarted_ = false;
        targetHoldStartWallS_ = 0.0;
        targetHoldTotalS_ = std::max(0.0, config_.hold_after_target_mins * 60.0);
        plannedTotalS_ = (config_.length_mode == sonocontrol::LengthMode::TotalDuration)
            ? config_.total_duration_mins * 60.0
            : ((config_.length_mode == sonocontrol::LengthMode::RepeatingCycles)
                ? static_cast<double>(config_.repeating) * (config_.hyus_seq_period_ms / 1000.0)
                : targetHoldTotalS_);
        refreshSessionTimerUi();
        sessionUiTimer_.start();
        tempPlot_->clear();
        tempPlot_->setLines(spnSetpoint_ ? spnSetpoint_->value() : 0.0, spnCutoff_ ? spnCutoff_->value() : 0.0);

        // Launch the active control loop only when there is something to control:
        // temperature monitoring/cutoff, PID, or cycling. Otherwise the device
        // self-clocks and a simple timer handles total-duration auto-stop.
        const bool needControl = config_.temperature_enabled || config_.pid_enabled || config_.use_cycling;
        if (needControl) {
            launchHyusWorker(config_, ip);
        } else if (config_.length_mode == sonocontrol::LengthMode::TotalDuration && plannedTotalS_ > 0.0) {
            hyusStopTimer_.start(static_cast<int>(plannedTotalS_ * 1000.0));
        }
    }

    // Create + start the Hyus control-loop worker on its own thread. Shared by
    // single runs and sequence items.
    void launchHyusWorker(const sonocontrol::Config& cfg, const QString& ip) {
        if (probe_) probe_->setPaused(true);  // the worker owns the HH806AU while running
        hyusWorkerThread_ = new QThread(this);
        hyusWorker_ = new HyusRunWorker(cfg, ip.toStdString(), hyusDiscovery_.get());
        hyusWorker_->moveToThread(hyusWorkerThread_);
        connect(hyusWorkerThread_, &QThread::started, hyusWorker_, &HyusRunWorker::run);
        connect(hyusWorker_, &HyusRunWorker::console, this, &SonoControlWindow::appendConsole);
        connect(hyusWorker_, &HyusRunWorker::temperature, this, &SonoControlWindow::onTemperature);
        connect(hyusWorker_, &HyusRunWorker::timeUpdate, this, &SonoControlWindow::onTimeUpdate);
        connect(hyusWorker_, &HyusRunWorker::cycle, this, &SonoControlWindow::onCycle);
        connect(hyusWorker_, &HyusRunWorker::cutoff, this, &SonoControlWindow::onCutoff);
        connect(hyusWorker_, &HyusRunWorker::finished, this, &SonoControlWindow::onHyusWorkerFinished);
        connect(hyusWorker_, &HyusRunWorker::finished, hyusWorkerThread_, &QThread::quit);
        connect(hyusWorkerThread_, &QThread::finished, hyusWorker_, &QObject::deleteLater);
        connect(hyusWorkerThread_, &QThread::finished, hyusWorkerThread_, &QObject::deleteLater);
        hyusWorkerThread_->start();
    }

    // Run one sequence item on the Hyus device: push that config's parameters,
    // start, and always run the worker (so it finishes deterministically and
    // sends RUN=0 at the end). The inter-config interval is handled by the shared
    // sequence machinery via onHyusWorkerFinished -> handleSequenceItemFinished.
    void launchHyusSequenceItem(const sonocontrol::Config& cfg) {
        const QString ip = selectedHyusDevice();
        if (ip.isEmpty() || !hyusDiscovery_) {
            appendConsole("[Sequence] No Hyus device connected; aborting sequence.");
            finalizeSequence(/*ok=*/false);
            return;
        }
        config_ = cfg;
        validate_config(config_);
        const auto initBatch = buildHyusInitBatch(config_);
        hyusDiscovery_->send(ip.toStdString(), initBatch.data(), initBatch.size());
        const auto seq = buildHyusStartSeq(config_);
        hyusDiscovery_->send(ip.toStdString(), seq.data(), seq.size());
        running_ = true;
        targetHoldMode_ = (config_.length_mode == sonocontrol::LengthMode::HoldAfterTarget);
        targetHoldStarted_ = false;
        targetHoldStartWallS_ = 0.0;
        targetHoldTotalS_ = std::max(0.0, config_.hold_after_target_mins * 60.0);
        plannedTotalS_ = (config_.length_mode == sonocontrol::LengthMode::TotalDuration)
            ? config_.total_duration_mins * 60.0
            : ((config_.length_mode == sonocontrol::LengthMode::RepeatingCycles)
                ? static_cast<double>(config_.repeating) * (config_.hyus_seq_period_ms / 1000.0)
                : targetHoldTotalS_);
        startWall_.start();
        refreshSessionTimerUi();
        sessionUiTimer_.start();
        tempPlot_->clear();
        tempPlot_->setLines(config_.pid_setpoint, config_.cutoff_temp);
        lblCycle_->setText("RUNNING");
        appendConsole(QString("=== [Sequence %1/%2] Hyus run started on %3 ===")
                          .arg(sequenceIndex_ + 1).arg(sequenceTotal_).arg(ip));
        launchHyusWorker(config_, ip);  // always (finishes -> RUN=0 -> next after interval)
    }

    void onStopHyus() {
        // Manual-stop PIN gate (restored for Hyus): the operator must confirm
        // the PIN configured at preflight before the run can be stopped by hand.
        if (pendingPinEnabled_ && !promptForStopPin()) {
            appendConsole(">>> Manual stop cancelled (PIN not confirmed)");
            statusBar()->showMessage("Manual stop cancelled");
            return;
        }
        hyusStopTimer_.stop();
        if (hyusWorker_) {
            // The worker sends RUN=0 then emits finished → onHyusWorkerFinished
            // resets the UI. Don't double-send here.
            hyusWorker_->requestStop();
            appendConsole("Hyus STOP requested.");
            return;
        }
        const QString ip = selectedHyusDevice();
        if (!ip.isEmpty() && hyusDiscovery_) {
            const auto stop = sonocontrol::hyus::write_param(sonocontrol::hyus::Cmd::Run, 0u);
            hyusDiscovery_->send(ip.toStdString(), stop.data(), stop.size());
            appendConsole("Hyus STOP sent to " + ip + ".");
        }
        finishHyusRunUi("IDLE");
    }

    // No-monitoring total-duration auto-stop (simple timer path).
    void onHyusRunFinished() {
        const QString ip = selectedHyusDevice();
        if (!ip.isEmpty() && hyusDiscovery_) {
            const auto stop = sonocontrol::hyus::write_param(sonocontrol::hyus::Cmd::Run, 0u);
            hyusDiscovery_->send(ip.toStdString(), stop.data(), stop.size());
        }
        appendConsole("=== Hyus run complete ===");
        finishHyusRunUi("DONE");
    }

    // Control-loop worker finished (normal end, user stop, or cutoff).
    void onHyusWorkerFinished(int code) {
        hyusWorker_ = nullptr;        // deleteLater'd via thread finished
        hyusWorkerThread_ = nullptr;
        if (probe_ && !shutdownInProgress_) probe_->setPaused(false);  // resume temp-pill probing
        if (sequenceActive_) {
            // The worker already transmitted RUN=0; the sequence machinery waits
            // the configured interval, then launches the next configuration.
            running_ = false;
            sessionUiTimer_.stop();
            const int oneBased = sequenceIndex_ + 1;
            const QString verb = (code == 2) ? "stopped by safety cutoff"
                               : (code == 3) ? "stopped" : "complete";
            appendConsole(QString("=== [Sequence %1/%2] Hyus run %3 ===").arg(oneBased).arg(sequenceTotal_).arg(verb));
            lblCycle_->setText(code == 2 ? "CUTOFF" : (code == 3 ? "STOPPED" : "COMPLETE"));
            handleSequenceItemFinished(code);
            return;
        }
        if (code == 2) {              // cutoff — onCutoff already fired the dialog
            finishHyusRunUi("CUTOFF");
        } else {
            appendConsole(code == 3 ? "=== Hyus run stopped ===" : "=== Hyus run complete ===");
            finishHyusRunUi(code == 3 ? "STOPPED" : "DONE");
        }
    }

    // Edit -> Switch Device. Closes the current session and asks main() to
    // re-show the device-selection window (the program effectively returns to
    // the device picker). Refused while a run/sequence is active.
    static QString deviceDisplayName(sonocontrol::DeviceKind k) {
        return k == sonocontrol::DeviceKind::Hyus ? QStringLiteral("Hyus (LAN)")
                                                  : QStringLiteral("Zhuhai (COM+UDP)");
    }

    void onSwitchDevice() {
        if (running_ || sequenceActive_) {
            QMessageBox::warning(this, "Switch Device", "Stop the current run before switching devices.");
            return;
        }
        const auto choice = QMessageBox::question(this, "Switch Device",
            "Switch to a different device?\n\nThe current session will close and the device "
            "selection window will appear.",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (choice != QMessageBox::Yes) return;
        g_switchDeviceRequested = true;
        close();  // closeEvent tears down discovery/connection; main() loops
    }

    void finishHyusRunUi(const QString& phase) {
        running_ = false;
        sessionUiTimer_.stop();
        btnStart_->setEnabled(true);
        btnStop_->setEnabled(false);
        btnStop_->setText(" ■  EMERGENCY STOP ");
        lblCycle_->setText(phase);
        statusBar()->showMessage("Ready");
        // Drop the manual-stop PIN so it does not leak across runs.
        pendingPinEnabled_ = false;
        pendingPinUsername_.clear();
        pendingPin_.clear();
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
        updateSequencePreview();
    }

    void onPidChanged(bool enabled) {
        for (auto* w : pidWidgets_) if (w) w->setEnabled(enabled);
        if (enabled && chkTempMonitor_ && !chkTempMonitor_->isChecked()) {
            QSignalBlocker b(chkTempMonitor_);
            chkTempMonitor_->setChecked(true);
        }
        updateLengthModeUi();
        updateCoolingModeUi();
    }

    // "Hold at temperature (PID)" cooling needs PID feedback. Disable it (and
    // revert to "Stop ultrasound") whenever PID is off. Applies to both devices.
    void updateCoolingModeUi() {
        const bool pid = chkPid_ && chkPid_->isChecked();
        if (!rbCoolLow_) return;
        rbCoolLow_->setEnabled(pid);
        rbCoolLow_->setToolTip(pid ? QString()
            : "Hold-at-temperature cooling requires PID temperature control to be enabled.");
        if (!pid && rbCoolLow_->isChecked() && rbCoolStop_) {
            QSignalBlocker b(rbCoolStop_);
            rbCoolStop_->setChecked(true);
        }
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
            // after a fresh port open. Give the device 150 ms to warm up.
            // Note: this blocks the GUI thread for up to ~1.2 s total
            // (3 × ~360 ms sensor poll + 2 × 150 ms sleep). Acceptable for a
            // user-triggered manual test but should not be called in a loop.
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
        // Header is read by name (not by position) so both the legacy schema
        // and the extended schema (with phase/setpoint_C/pid_demand_pct/
        // us_active appended at the end) are accepted unchanged: extra
        // columns are simply ignored, and a setpoint column is honoured for
        // the dashed reference line if it exists.
        const QStringList header = lines.first().split(',');
        auto idx = [&](const QString& name) { return header.indexOf(name); };
        const int iTime = idx("time_s");
        const int iT1 = idx("T1_C");
        const int iT2 = idx("T2_C");
        const int iTemp = idx("temp_C");
        const int iSetpoint = idx("setpoint_C");  // -1 on legacy CSVs
        if (iTime < 0 || iTemp < 0) {
            QMessageBox::warning(this, "Import Error", "CSV must contain at least time_s and temp_C columns.");
            return;
        }
        tempPlot_->clear();
        // Pick up the recorded setpoint for the dashed reference line so the
        // plot reflects the imported run instead of whatever the GUI happens
        // to have configured right now. The cutoff temperature is not a
        // per-sample column in either schema — keep the live GUI value for it.
        double importedSetpoint = std::numeric_limits<double>::quiet_NaN();
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
            if (iSetpoint >= 0 && cols.size() > iSetpoint && std::isnan(importedSetpoint)) {
                bool okSp = false;
                const double sp = cols[iSetpoint].trimmed().toDouble(&okSp);
                if (okSp) importedSetpoint = sp;
            }
            tempPlot_->append(ts, okT1 ? t1 : 0.0, okT2 ? t2 : 0.0, temp);
            ++count;
        }
        // Apply the reference line AFTER the loop so the imported setpoint
        // (if any) replaces the GUI value; otherwise fall back to GUI.
        tempPlot_->setLines(std::isnan(importedSetpoint) ? spnSetpoint_->value() : importedSetpoint,
                            spnCutoff_->value());
        appendConsole(QString("Imported log for plotting: %1 (%2 samples%3)")
                          .arg(path)
                          .arg(count)
                          .arg(std::isnan(importedSetpoint) ? QString() : QString(", setpoint=%1°C").arg(importedSetpoint, 0, 'f', 2)));
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
        QDialog dlg(this);
        dlg.setWindowTitle("SonoControl — Readme");
        dlg.setModal(true);
        dlg.resize(720, 640);
        auto* layout = new QVBoxLayout(&dlg);

        auto* title = new QLabel("SonoControl quick reference");
        title->setStyleSheet("font-size:18px; font-weight:700;");
        layout->addWidget(title);

        auto* body = new QPlainTextEdit;
        body->setReadOnly(true);
        body->setStyleSheet("font-family:'Segoe UI',Arial,sans-serif; font-size:13px;");
        body->setPlainText(
            "PID parameters\n"
            "──────────────\n"
            "Kp  Proportional gain. Higher = stronger reaction to error. Raise if response is too slow; lower if it overshoots or oscillates.\n"
            "Ki  Integral gain. Removes long-term offset; too much causes delayed overshoot. For small (≈1 mL) wells start at zero or near-zero.\n"
            "Kd  Derivative gain. Suppresses rapid rises; too much makes output noisy.\n"
            "Thermal const (tau)  Liquid time constant for the predictive model T_future = T + τ·dT/dt·(1 − e^(−t/τ)). Larger τ brakes earlier; τ = 0 disables prediction.\n"
            "Prediction horizon  Lookahead in seconds. 0 = use the current hardware interval.\n"
            "Temp smoothing window  Span (seconds) of the least-squares fit that estimates dT/dt from the noisy probe. Larger = smoother but slower-reacting slope; default 30 s, range 1/sample-rate to 60 s.\n"
            "\n"
            "Experiment length modes\n"
            "───────────────────────\n"
            "Total Duration    Wall-clock minutes; supports temperature cycling.\n"
            "Repeating Cycles  Fixed number of burst-interval cycles. Disabled when PID is enabled.\n"
            "After-target Hold Starts the hold timer the first time the channel temperature reaches setpoint ± tolerance. Requires temperature monitoring.\n"
            "\n"
            "Cycling (Total Duration mode only)\n"
            "──────────────────────────────────\n"
            "Heating / Cooling phases are entered in minutes. Each phase is capped at the configured Total Duration. Cooling mode is either Stop (ultrasound off) or Hold (PID maintains a lower setpoint).\n"
            "\n"
            "Manual-stop PIN protection\n"
            "──────────────────────────\n"
            "The Preflight Confirmation dialog has an optional username + 4-digit PIN. When enabled, the manual STOP button asks for the PIN before stopping. The Sequence dialog has its own equivalent PIN that gates the sequence STOP button independently. In both cases the 2-second force-stop escalation and the stall watchdog still bypass the prompt so safety paths cannot be locked out.\n"
            "\n"
            "Sequence (Edit → Sequence…)\n"
            "───────────────────────────\n"
            "Queue multiple .config files to run back-to-back as independent experiments. Drag files from Explorer or click \"Add Configs…\". Reorder with the ↑/↓ buttons; remove a row with ×. Each pair of adjacent configurations has its own \"Interval\" spin box (minutes, 0–600) — the gap that runs after one experiment finishes and before the next starts. The estimated total adds every configuration's duration plus every gap; it shows \"unavailable\" when any queued config uses after-target hold (the hold timer can't be predicted). Each entry produces its own timestamped CSV/JSON log under ./logs, exactly as a single run would. The sequence has its own START / STOP buttons; main page START is disabled while a sequence is queued. No preflight confirmation window is shown for sequence runs.\n"
            "\n"
            "CONNECT tab\n"
            "───────────\n"
            "Scan Ports refreshes serial port discovery. COM3 = ultrasound controller, UDP host/port = waveform target. The HH806AU section connects the dual-channel thermocouple meter; pick T1, T2, or Average. Use Test before running PID. Temperature monitoring is mandatory for PID and After-target Hold.\n"
            "\n"
            "Configuration files\n"
            "───────────────────\n"
            "File → Save Configuration / Load Configuration use a UTF-8 .config file. Edit → Set Auto-save Directory stores the default completed-experiment output folder. Edit → Sequence opens the experiment-sequence dialog. File → Import Log File loads a CSV log and redraws the temperature plot, picking up the recorded setpoint as the reference line when present.\n"
            "\n"
            "Logging\n"
            "───────\n"
            "Streaming CSV log per session under ./logs/<timestamp>_log.csv; metadata JSON next to it; auto-saved bundle under ./experiments/<session>/ (or your configured auto-save directory) on clean completion. The HH806AU raw log rotates daily as hh806au_raw_YYYYMMDD.log. Sequence runs do not create a separate log type — every queued configuration writes its own session log just like a normal single run."
        );
        layout->addWidget(body, 1);

        auto* close = new QDialogButtonBox(QDialogButtonBox::Close);
        connect(close, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        connect(close, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        layout->addWidget(close);
        dlg.exec();
    }

    void showAboutDialog() {
        QDialog dlg(this);
        dlg.setWindowTitle("About SonoControl");
        dlg.setModal(true);
        dlg.resize(620, 640);
        auto* layout = new QVBoxLayout(&dlg);
        layout->setContentsMargins(20, 20, 20, 16);
        layout->setSpacing(10);

        auto* name = new QLabel("SonoControl");
        name->setStyleSheet("font-size:24px; font-weight:800;");
        layout->addWidget(name);

        auto* tagline = new QLabel("Ultrasound thermal controller — desktop application driving an FPGA ultrasound device with PID-regulated temperature feedback.");
        tagline->setWordWrap(true);
        tagline->setStyleSheet("font-size:13px; color:#475467;");
        layout->addWidget(tagline);

        auto* versionRow = new QLabel(QString("Version %1   ·   Qt %2   ·   %3 build")
            .arg(QApplication::applicationVersion())
            .arg(qstr(qVersion()))
            .arg(debug_sim_build() ? "Debug (simulation enabled)" : "Release"));
        versionRow->setStyleSheet("font-size:12px; color:#667085;");
        layout->addWidget(versionRow);

        auto* sep1 = new QFrame; sep1->setFrameShape(QFrame::HLine); sep1->setStyleSheet("color:#e4e7ec;");
        layout->addWidget(sep1);

        auto* sectionsLbl = new QLabel("What's in this build");
        sectionsLbl->setStyleSheet("font-size:14px; font-weight:700; letter-spacing:0.4px;");
        layout->addWidget(sectionsLbl);

        auto* body = new QPlainTextEdit;
        body->setReadOnly(true);
        body->setStyleSheet("font-family:'Segoe UI',Arial,sans-serif; font-size:12px;");
        body->setPlainText(
            "Experiment control\n"
            "  • Ultrasound amplitude, carrier frequency, PRF, duty cycle, duration, interval\n"
            "  • Sine / square / triangle waveforms over a 4096-sample DDS table (UDP)\n"
            "  • Length modes: Total Duration, Repeating Cycles, After-target Hold\n"
            "  • Temperature cycling (heating / cooling, in minutes, capped at total duration)\n"
            "  • PID with predictive thermal model T_future = T + τ·dT/dt·(1 − e^(−t/τ))\n"
            "  • Configurable safety cutoff with debounce (N samples, min spacing)\n"
            "\n"
            "Experiment Sequence (Edit → Sequence…)\n"
            "  • Queue multiple .config files; each runs back-to-back as an independent experiment\n"
            "  • Drag-and-drop import from Explorer, or manual file picker; ↑/↓/× per row to reorder\n"
            "  • Per-gap interval in minutes (0–600), edited inline between every adjacent pair\n"
            "  • Estimated total time computed from each config's duration + every gap (shown\n"
            "    as 'unavailable' when any queued config uses after-target hold)\n"
            "  • Independent START / STOP for the sequence; main page buttons lock out while a\n"
            "    sequence is queued; sequence STOP has its own PIN protection\n"
            "  • No preflight confirmation window for sequence runs; each queued config writes\n"
            "    its own session log — no separate sequence log type\n"
            "\n"
            "Hardware\n"
            "  • Ultrasound FPGA: COM3 9600 8N1 control + UDP 4096-sample waveform table\n"
            "  • Omega HH806AU dual-channel thermocouple meter: COM5 19200 8E1\n"
            "  • Persistent COM3, exponential-backoff retry, force-stop cancels pending I/O\n"
            "\n"
            "Safety & operations\n"
            "  • Manual-stop PIN protection (username + 4-digit PIN, per-run and per-sequence)\n"
            "  • Stall watchdog: auto-escalates to force-stop after configurable timeout\n"
            "  • Status pills (COM3 / UDP / Temp) updated by a background probe — never blocks the UI\n"
            "  • Auto-theme: dark mode 17:00–08:00 by default\n"
            "\n"
            "Logging\n"
            "  • Streaming CSV per session with per-sample columns: T1, T2, temperature,\n"
            "    full ultrasound params, plus phase, setpoint, PID demand %, US active\n"
            "  • Metadata JSON with bounded event/error ring (no unbounded growth)\n"
            "  • HH806AU raw diagnostic log rotates daily and coalesces repeats\n"
            "  • CSV import accepts both the new schema and the legacy schema\n"
            "  • Sequence runs produce one independent session log per queued configuration\n"
            "\n"
            "Long-run optimizations in this build\n"
            "  • Plot retains the full session — paint downsamples for display\n"
            "  • Cached UDP destination and waveform preview avoid per-packet rebuilds\n"
            "  • Larger CSV streambuf and decoupled meta-rewrite cadence\n"
            "  • Errors flush CSV and metadata immediately\n"
            "\n"
            "Credits\n"
            "  • SonoControl is provided as-is for research use.\n"
            "  • Built with Qt Widgets, C++17, CMake."
        );
        layout->addWidget(body, 1);

        auto* pathsLbl = new QLabel(QString("Logs folder:   %1\nAuto-save:     %2")
            .arg(qstr(logger_.log_dir().string()))
            .arg(autoSaveDir_.isEmpty() ? QString("(default ./experiments/<session>)") : autoSaveDir_));
        pathsLbl->setStyleSheet("font-family:Consolas,monospace; font-size:11px; color:#475467;");
        pathsLbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addWidget(pathsLbl);

        auto* btns = new QDialogButtonBox(QDialogButtonBox::Close);
        auto* openLogsBtn = btns->addButton("Open Log Folder", QDialogButtonBox::ActionRole);
        connect(openLogsBtn, &QPushButton::clicked, this, &SonoControlWindow::openLogs);
        connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        layout->addWidget(btns);
        dlg.exec();
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
        if (seqPlot_) seqPlot_->update();
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

QWidget#targetHoldPanel {
    background: transparent;
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
        edit->addSeparator();
        auto* actSequence = edit->addAction("Sequence...");
        connect(actSequence, &QAction::triggered, this, &SonoControlWindow::openSequenceDialog);
        edit->addSeparator();
        auto* actSwitchDevice = edit->addAction("Switch Device...");
        connect(actSwitchDevice, &QAction::triggered, this, &SonoControlWindow::onSwitchDevice);

        auto* about = menuBar()->addMenu("About");
        auto* actReadme = about->addAction("Readme");
        auto* actAbout = about->addAction("About SonoControl...");
        about->addSeparator();
        auto* actOpenLogs = about->addAction("Open Log Folder");
        connect(actReadme, &QAction::triggered, this, &SonoControlWindow::showReadme);
        connect(actAbout, &QAction::triggered, this, &SonoControlWindow::showAboutDialog);
        connect(actOpenLogs, &QAction::triggered, this, &SonoControlWindow::openLogs);
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
        // Status pills — compact background-colored labels for at-a-glance state.
        // The FPGA device shows COM3/UDP/Temp; the Hyus device shows a single LAN
        // pill (green once a device is detected on the local network).
        const bool hyus = (deviceKind_ == sonocontrol::DeviceKind::Hyus);
        lblCom3Status_ = new QLabel("COM3");
        lblUsStatus_ = new QLabel("UDP");
        lblTempStatus_ = new QLabel("Temp");
        lblLanStatus_ = new QLabel("LAN");
        for (auto* x : {lblCom3Status_, lblUsStatus_, lblTempStatus_, lblLanStatus_}) {
            x->setStyleSheet(statusStyle("#b54708"));
            x->setMinimumHeight(22);
            x->setMaximumHeight(24);
            l->addWidget(x);
            l->addSpacing(4);
        }
        lblCom3Status_->setVisible(!hyus);
        lblUsStatus_->setVisible(!hyus);
        lblTempStatus_->setVisible(true);   // temperature monitoring applies to both devices
        lblLanStatus_->setVisible(hyus);
        btnStart_ = new QPushButton(" ▶  START "); btnStart_->setObjectName("start"); btnStart_->setMinimumWidth(120); btnStart_->setCursor(Qt::PointingHandCursor); connect(btnStart_, &QPushButton::clicked, this, &SonoControlWindow::onStart);
        btnStop_ = new QPushButton(" ■  EMERGENCY STOP "); btnStop_->setObjectName("stop"); btnStop_->setMinimumWidth(170); btnStop_->setEnabled(false); btnStop_->setCursor(Qt::PointingHandCursor);
        // Dispatch click depending on which phase we're in: first click sends
        // a graceful request_stop; if the worker doesn't exit in 2 s, the
        // button is re-labelled "FORCE STOP" and the next click cancels I/O.
        connect(btnStop_, &QPushButton::clicked, this, [this]() {
            if (deviceKind_ == sonocontrol::DeviceKind::Hyus) { onStopHyus(); return; }
            if (btnStop_->text().contains("FORCE", Qt::CaseInsensitive)) onForceStop();
            else onStop();
        });
        l->addSpacing(6);
        l->addWidget(btnStart_); l->addSpacing(6); l->addWidget(btnStop_);
        return h;
    }

    // Per-device UI policy. All tabs are available for both devices now; the
    // per-device differences are handled inside the individual tab builders
    // (PARAMS/PID/CONNECT swap their groups by device).
    void applyDeviceUiPolicy() {
        if (!tabs_) return;
    }

    QWidget* buildLeftPanel() {
        auto* scroll = new QScrollArea; scroll->setWidgetResizable(true); scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* cont = new QWidget; cont->setMaximumWidth(410);
        auto* v = new QVBoxLayout(cont); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* tabs = new QTabWidget; tabs->addTab(buildParamsTab(), "PARAMS"); tabs->addTab(buildPidTab(), "PID"); tabs->addTab(buildCycleTab(), "CYCLE"); tabs->addTab(buildConnTab(), "CONNECT");
        tabs_ = tabs;
        applyDeviceUiPolicy();
        v->addWidget(tabs);
        lblConfigStatus_ = new QLabel("Config: GUI defaults");
        lblConfigStatus_->setWordWrap(true);
        lblConfigStatus_->setStyleSheet("color:#667085; font-size:12px; padding:4px;");
        v->addWidget(lblConfigStatus_);
        v->addStretch(); scroll->setWidget(cont); return scroll;
    }

    QWidget* buildParamsTab() {
        const bool hyus = (deviceKind_ == sonocontrol::DeviceKind::Hyus);
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);

        // --- Ultrasound (FPGA): amplitude / carrier freq / PRF / duty / waveform ---
        auto* us = new QGroupBox("Ultrasound"); auto* g = new QGridLayout(us);
        spnAmp_ = dspin(0.0,1.0,0.5,3,0.01); spnCfreq_ = dspin(1.0,10000.0,500.0,1,10.0); spnPrf_ = dspin(1.0,100000.0,1000.0,0,100.0); spnDuty_ = dspin(0.0,100.0,50.0,2,0.1);
        cmbWave_ = new QComboBox; cmbWave_->addItems({"Sine","Square","Triangle"});
        addRow(g,0,"Amplitude",spnAmp_,"(0–1)"); addRow(g,1,"CFreq",spnCfreq_,"kHz"); addRow(g,2,"PRF",spnPrf_,"Hz"); addRow(g,3,"Duty Cycle",spnDuty_,"%"); addRow(g,4,"Waveform",cmbWave_,""); v->addWidget(us);
        // --- Ultrasound (Hyus): pulse frequency + pulse amplitude only ---
        auto* usH = new QGroupBox("Ultrasound"); auto* gH = new QGridLayout(usH);
        spnHyusFreqMhz_ = dspin(0.0, 1.0, 1.0, 3, 0.03125); spnHyusAmpPct_ = dspin(0.0, 100.0, 50.0, 1, 1.0);
        addRow(gH,0,"Pulse Frequency",spnHyusFreqMhz_,"MHz"); addRow(gH,1,"Pulse Amplitude",spnHyusAmpPct_,"%"); v->addWidget(usH);

        // --- Timing (FPGA): duration / interval / sample rate ---
        auto* timing = new QGroupBox("Timing"); auto* gt = new QGridLayout(timing);
        // Interval lower bound is sonocontrol::kMinIntervalTimeS (see the
        // header for rationale: serial CFREQ/PRF/DUR with the hardware
        // 100 ms COM gap each + 4096 UDP datagrams). Preflight raises an
        // extra soft warning at <0.5 s.
        spnDuration_ = dspin(10.0,60000.0,1000.0,0,100.0); spnInterval_ = dspin(sonocontrol::kMinIntervalTimeS,3600.0,5.0,2,0.5); spnSampleRate_ = dspin(0.1,20.0,2.0,1,0.5);
        // spnSampleRate_ is shared; it is added to whichever Timing group is
        // shown so temperature sampling rate is editable for both devices.
        addRow(gt,0,"Duration",spnDuration_,"ms"); addRow(gt,1,"Interval",spnInterval_,"s"); if (!hyus) addRow(gt,2,"Sample Rate",spnSampleRate_,"Hz"); v->addWidget(timing);
        // --- Timing (Hyus): pulse length/period + sequence length/period ---
        auto* timingH = new QGroupBox("Timing"); auto* gtH = new QGridLayout(timingH);
        spnHyusPulseLenUs_ = dspin(0.1, 1000000.0, 160.0, 1, 10.0);
        spnHyusPulsePeriodUs_ = dspin(0.1, 1000000.0, 400.0, 1, 10.0);
        spnHyusSeqLenMs_ = dspin(0.001, 1000000.0, 1.0, 3, 1.0);
        spnHyusSeqPeriodMs_ = dspin(0.001, 1000000.0, 1000.0, 3, 100.0);
        addRow(gtH,0,"Pulse Length",spnHyusPulseLenUs_,"us"); addRow(gtH,1,"Pulse Period",spnHyusPulsePeriodUs_,"us"); addRow(gtH,2,"Sequence Length",spnHyusSeqLenMs_,"ms"); addRow(gtH,3,"Sequence Period",spnHyusSeqPeriodMs_,"ms"); if (hyus) addRow(gtH,4,"Sample Rate",spnSampleRate_,"Hz"); v->addWidget(timingH);

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
        // Target-hold is temperature-based and applies to both devices because
        // both share the HH806AU temperature path.
        auto* targetHold = new QWidget;
        targetHold->setObjectName("targetHoldPanel");
        auto* thv = new QVBoxLayout(targetHold); thv->setContentsMargins(0,0,0,0);
        auto* row3 = new QHBoxLayout; row3->addWidget(rbTargetHold_); row3->addWidget(spnTargetHoldMin_); row3->addWidget(new QLabel("min"));
        auto* row4 = new QHBoxLayout; row4->addSpacing(26); row4->addWidget(new QLabel("Target tolerance ±")); row4->addWidget(spnTargetTol_); row4->addWidget(new QLabel(QString::fromUtf8("°C")));
        thv->addLayout(row3); thv->addLayout(row4);
        tv->addLayout(row1); tv->addLayout(row2); tv->addWidget(targetHold);
        auto* note = new QLabel(hyus
            ? "Total duration = Repeating × Sequence period. After-target Hold uses the shared HH806AU temperature feedback."
            : "PID mode allows Total Duration or After-target Hold only. Cycle mode is available only with Total Duration.");
        note->setWordWrap(true); note->setStyleSheet("color:#667085; font-size:12px;");
        tv->addWidget(note);
        v->addWidget(total);
        connect(rbTotalDur_, &QRadioButton::toggled, this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(rbRepeating_, &QRadioButton::toggled, this, &SonoControlWindow::updateLengthModeUi);
        connect(rbTargetHold_, &QRadioButton::toggled, this, &SonoControlWindow::updateLengthModeUi);
        connect(spnTotalDur_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(spnInterval_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SonoControlWindow::updateRepeatingFromDuration);
        connect(spnRepeating_, qOverload<int>(&QSpinBox::valueChanged), this, &SonoControlWindow::updateDurationFromRepeating);

        grpSafety_ = new QGroupBox("Safety");
        auto* safetyV = new QVBoxLayout(grpSafety_);
        auto* sh = new QHBoxLayout;
        spnCutoff_ = dspin(35.0, 100.0, 45.0, 1, 0.5);
        sh->addWidget(new QLabel("Cutoff Temp"));
        sh->addWidget(spnCutoff_);
        sh->addWidget(new QLabel(QString::fromUtf8("°C")));
        sh->addStretch();
        safetyV->addLayout(sh);
        chkTempRequired_ = new QCheckBox("Abort run if temperature sensor stops responding (fail-closed)");
        chkTempRequired_->setToolTip("When enabled, three consecutive invalid temperature samples will stop ultrasound and abort the run. PID and after-target-hold runs always fail closed regardless of this flag.");
        safetyV->addWidget(chkTempRequired_);
        v->addWidget(grpSafety_); v->addStretch();

        // Per-device visibility. Widgets for the other device stay created (so
        // buildConfig/applyConfigToUi remain null-safe) but hidden.
        us->setVisible(!hyus); timing->setVisible(!hyus);
        usH->setVisible(hyus); timingH->setVisible(hyus);
        // Safety cutoff applies to both devices (temperature monitoring is shared).
        grpSafety_->setVisible(true);
        targetHold->setVisible(true);
        if (hyus) {
            using sonocontrol::hyus::Cmd;
            namespace hp = sonocontrol::hyus;
            auto vc = qOverload<double>(&QDoubleSpinBox::valueChanged);
            enforceHyusTiming();  // establish initial dependent ranges
            // Each edit clamps timing to the constraints, refreshes the schematic,
            // and is sent to the device immediately (whether or not running).
            connect(spnHyusFreqMhz_, vc, this, [this](double v){ updateSequencePreview(); sendHyusParam(Cmd::Cfreq, hp::encode_cfreq(v * 1.0e6)); });
            connect(spnHyusAmpPct_, vc, this, [this](double v){ updateSequencePreview(); sendHyusParam(Cmd::Amplitude, hp::encode_amplitude_percent(v)); });
            connect(spnHyusPulseLenUs_, vc, this, [this](double){ enforceHyusTiming(); sendHyusParam(Cmd::PulseLen, hp::encode_pulse_len_us(spnHyusPulseLenUs_->value())); updateSequencePreview(); });
            connect(spnHyusPulsePeriodUs_, vc, this, [this](double){ enforceHyusTiming(); sendHyusParam(Cmd::PulsePeriod, hp::encode_pulse_period_us(spnHyusPulsePeriodUs_->value())); updateSequencePreview(); });
            connect(spnHyusSeqLenMs_, vc, this, [this](double){ enforceHyusTiming(); sendHyusParam(Cmd::SeqLen, hp::encode_seq_len_ms(spnHyusSeqLenMs_->value())); updateSequencePreview(); });
            connect(spnHyusSeqPeriodMs_, vc, this, [this](double){ enforceHyusTiming(); sendHyusParam(Cmd::SeqPeriod, hp::encode_seq_period_ms(spnHyusSeqPeriodMs_->value())); updateRepeatingFromDuration(); });
            // Experiment-length edits also map to live frames (run mode + total duration).
            connect(spnTotalDur_, vc, this, [this](double v){ if (rbTotalDur_ && rbTotalDur_->isChecked()) sendHyusParam(Cmd::TotalDuration, hp::encode_total_duration_s(v * 60.0)); });
            connect(rbTotalDur_, &QRadioButton::toggled, this, [this](bool on){ if (on) { sendHyusParam(Cmd::RunMode, 1u); sendHyusParam(Cmd::TotalDuration, hp::encode_total_duration_s(spnTotalDur_->value() * 60.0)); } });
            connect(rbRepeating_, &QRadioButton::toggled, this, [this](bool on){ if (on) sendHyusParam(Cmd::RunMode, 0u); });
            connect(rbTargetHold_, &QRadioButton::toggled, this, [this](bool on){ if (on) sendHyusParam(Cmd::RunMode, 0u); });
        } else {
            auto vc = qOverload<double>(&QDoubleSpinBox::valueChanged);
            enforceZhuhaiTiming();  // establish initial Duration/Interval ranges
            // Keep the schematic preview live as the user edits the FPGA params.
            connect(spnAmp_, vc, this, [this](double){ updateSequencePreview(); });
            connect(spnPrf_, vc, this, [this](double){ updateSequencePreview(); });
            connect(spnDuty_, vc, this, [this](double){ updateSequencePreview(); });
            connect(spnDuration_, vc, this, [this](double){ enforceZhuhaiTiming(); updateSequencePreview(); });
            connect(spnInterval_, vc, this, [this](double){ enforceZhuhaiTiming(); updateSequencePreview(); });
        }
        return w;
    }

    QWidget* buildPidTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* en = new QGroupBox("PID Enable"); auto* ev = new QVBoxLayout(en); chkPid_ = new QCheckBox("Enable PID Temperature Control"); ev->addWidget(chkPid_); connect(chkPid_, &QCheckBox::toggled, this, [this](bool v){ idleSensor_.reset(); onPidChanged(v); }); v->addWidget(en);
        auto* sp = new QGroupBox("Setpoint"); auto* sph = new QHBoxLayout(sp); spnSetpoint_ = dspin(20.0,50.0,40.0,1,0.5); sph->addWidget(new QLabel("Target Temp")); sph->addWidget(spnSetpoint_); sph->addWidget(new QLabel(QString::fromUtf8("°C"))); v->addWidget(sp);
        const bool hyus = (deviceKind_ == sonocontrol::DeviceKind::Hyus);
        // FPGA: multi-select controlled parameters.
        auto* sel = new QGroupBox("Controlled Parameters"); auto* sv = new QVBoxLayout(sel); chkPidAmp_ = new QCheckBox("Amplitude"); chkPidDuration_ = new QCheckBox("Duration"); chkPidInterval_ = new QCheckBox("Interval Time"); chkPidDuty_ = new QCheckBox("Duty Cycle"); chkPidAmp_->setChecked(true); for (auto* c : {chkPidAmp_, chkPidDuration_, chkPidInterval_, chkPidDuty_}) sv->addWidget(c); sv->addWidget(new QLabel("Interval ≥ Duration enforced")); v->addWidget(sel);
        // Hyus: single-select controlled variable. PID drives the chosen knob
        // live (no run interruption); the configured value is the upper limit.
        auto* selH = new QGroupBox("Controlled Variable (single)"); auto* svH = new QVBoxLayout(selH);
        rbHyusPidAmp_ = new QRadioButton("Pulse amplitude (% of set value)");
        rbHyusPidPulse_ = new QRadioButton("Pulse length / period (fixed period, adjust length)");
        rbHyusPidSeq_ = new QRadioButton("Sequence length / period (fixed period, adjust length)");
        rbHyusPidPulse_->setChecked(true);  // default
        auto* bgPid = new QButtonGroup(this); bgPid->addButton(rbHyusPidAmp_); bgPid->addButton(rbHyusPidPulse_); bgPid->addButton(rbHyusPidSeq_);
        for (auto* r : {rbHyusPidAmp_, rbHyusPidPulse_, rbHyusPidSeq_}) svH->addWidget(r);
        auto* pidNoteH = new QLabel("PID adjusts only the selected variable, sending modification commands during the run without pausing it.");
        pidNoteH->setWordWrap(true); pidNoteH->setStyleSheet("color:#667085; font-size:12px;");
        svH->addWidget(pidNoteH);
        v->addWidget(selH);
        sel->setVisible(!hyus);
        selH->setVisible(hyus);
        auto* gains = new QGroupBox("PID Gains");
        auto* gg = new QGridLayout(gains);
        spnKp_ = dspin(0.0,10.0,0.8,3,0.05);
        spnKi_ = dspin(0.0,5.0,0.05,3,0.01);
        spnKd_ = dspin(0.0,5.0,0.2,3,0.05);
        spnTau_ = dspin(0.0,3600.0,25.0,1,1.0);
        spnHorizon_ = dspin(0.0,300.0,0.0,1,1.0);
        // Range is [1/sample-rate, kMaxRateWindowS]. The absolute floor here
        // (0.05 s = 1/20 Hz, the sample-rate spinbox max) is tightened to the
        // live 1/sample-rate by enforceRateWindowRange(); two decimals so the
        // sub-second floor is representable.
        spnRateWindow_ = dspin(0.05,sonocontrol::kMaxRateWindowS,30.0,2,1.0);
        addRow(gg,0,"Kp",spnKp_,"");
        addRow(gg,1,"Ki",spnKi_,"");
        addRow(gg,2,"Kd",spnKd_,"");
        addRow(gg,3,"Thermal const",spnTau_,"s");
        addRow(gg,4,"Prediction horizon",spnHorizon_,"s");
        addRow(gg,5,"Temp smoothing window",spnRateWindow_,"s");
        // Keep the window floor in lockstep with the sample rate (1/sample-rate).
        enforceRateWindowRange();
        if (spnSampleRate_)
            connect(spnSampleRate_, qOverload<double>(&QDoubleSpinBox::valueChanged),
                    this, [this](double){ enforceRateWindowRange(); });
        auto* tauNote = new QLabel("Model: T_future = T + thermal_const × dT/dt × (1 - exp(-horizon/thermal_const)). Set thermal_const=0 to disable prediction. Set horizon=0 to use the current hardware interval. The temp smoothing window is the span of the least-squares fit used to estimate dT/dt (larger = smoother but slower-reacting slope).");
        tauNote->setWordWrap(true);
        tauNote->setStyleSheet("color:#667085; font-size:12px;");
        gg->addWidget(tauNote,6,0,1,3);
        v->addWidget(gains);

        pidWidgets_ = {sp, sel, selH, gains}; v->addStretch(); return w;
    }

    QWidget* buildCycleTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        auto* en = new QGroupBox("Temperature Cycling"); auto* ev = new QVBoxLayout(en); chkCycling_ = new QCheckBox("Enable Heating / Cooling Cycles"); ev->addWidget(chkCycling_); auto* cycleNote = new QLabel("Available only in Total Duration mode. Each phase is capped at the total duration."); cycleNote->setWordWrap(true); cycleNote->setStyleSheet("color:#667085; font-size:12px;"); ev->addWidget(cycleNote); v->addWidget(en);
        // Phase durations are entered in minutes. The underlying Config struct
        // still stores seconds (`heating_s`, `cooling_s`) so the experiment
        // loop and existing .config files keep working unchanged — conversion
        // happens at buildConfig() / applyConfigToUi().
        const double initial_total_min = spnTotalDur_ ? spnTotalDur_->value() : 60.0;
        spnHeatS_ = dspin(0.05, initial_total_min, 1.0, 2, 0.5);
        spnCoolS_ = dspin(0.05, initial_total_min, 0.5, 2, 0.5);
        auto* heat = new QGroupBox("Heating Phase"); auto* hh = new QHBoxLayout(heat); hh->addWidget(new QLabel("Duration")); hh->addWidget(spnHeatS_); hh->addWidget(new QLabel("min")); v->addWidget(heat);
        auto* cool = new QGroupBox("Cooling Phase"); auto* cv = new QVBoxLayout(cool); auto* cr = new QHBoxLayout; cr->addWidget(new QLabel("Duration")); cr->addWidget(spnCoolS_); cr->addWidget(new QLabel("min")); cv->addLayout(cr);
        rbCoolStop_ = new QRadioButton("Stop ultrasound"); rbCoolLow_ = new QRadioButton("Hold at temperature (PID)"); rbCoolStop_->setChecked(true); auto* bg = new QButtonGroup(this); bg->addButton(rbCoolStop_); bg->addButton(rbCoolLow_); cv->addWidget(rbCoolStop_); cv->addWidget(rbCoolLow_);
        auto* hold = new QGroupBox("Hold Temperature"); auto* hv = new QHBoxLayout(hold); spnCoolHoldTemp_ = dspin(20.0,45.0,37.0,1,0.5); hv->addWidget(new QLabel("Target")); hv->addWidget(spnCoolHoldTemp_); hv->addWidget(new QLabel(QString::fromUtf8("°C"))); cv->addWidget(hold); v->addWidget(cool); v->addStretch();
        // Keep the phase caps in sync with total duration as the user edits it.
        if (spnTotalDur_) {
            connect(spnTotalDur_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &SonoControlWindow::updateCyclePhaseCaps);
        }
        return w;
    }

    void updateCyclePhaseCaps() {
        if (!spnTotalDur_ || !spnHeatS_ || !spnCoolS_) return;
        const double cap = std::max(spnHeatS_->minimum(), spnTotalDur_->value());
        spnHeatS_->setMaximum(cap);
        spnCoolS_->setMaximum(cap);
    }

    QWidget* buildConnTab() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setSpacing(12); v->setContentsMargins(12,14,12,14);
        const bool hyus = (deviceKind_ == sonocontrol::DeviceKind::Hyus);
        auto* scan = new QPushButton("⟳  Scan Ports"); connect(scan, &QPushButton::clicked, this, &SonoControlWindow::refreshPorts); v->addWidget(scan);
        scan->setVisible(!hyus);  // serial-port scan is meaningless for the LAN device
        // SonoControl FPGA connection group (serial COM + UDP). The widgets are
        // always created (buildConfig/applyConfigToUi reference them
        // unconditionally); only the group's visibility is device-dependent.
        grpUsFpga_ = new QGroupBox("Ultrasound Device (FPGA)"); auto* gu = new QGridLayout(grpUsFpga_); cmbCom3_ = new QComboBox; txtUdpHost_ = new QLineEdit(sonocontrol::protocol::UDP_HOST_DEFAULT); spnUdpPort_ = ispin(1,65535,sonocontrol::protocol::UDP_PORT_DEFAULT); addRow(gu,0,"COM3 Port",cmbCom3_,""); addRow(gu,1,"UDP Host",txtUdpHost_,""); addRow(gu,2,"UDP Port",spnUdpPort_,""); v->addWidget(grpUsFpga_);
        // Hyus connection group (Ethernet). No IP/port entry: the device is
        // discovered automatically over the LAN (the PC broadcasts a beacon and
        // the device dials in) and the user simply picks it from the dropdown.
        grpUsHyus_ = new QGroupBox("Ultrasound Device (Hyus · Ethernet)");
        auto* gh = new QGridLayout(grpUsHyus_);
        cmbHyusDevice_ = new QComboBox; cmbHyusDevice_->setMinimumWidth(170);
        // Init batch is pushed when the user actively selects a device from the
        // dropdown (not automatically on connect).
        connect(cmbHyusDevice_, qOverload<int>(&QComboBox::activated), this, [this](int){
            const QString ip = selectedHyusDevice();
            if (!ip.isEmpty()) sendHyusInit(ip);
        });
        auto* scanHyus = new QPushButton("Scan"); scanHyus->setFixedWidth(72);
        connect(scanHyus, &QPushButton::clicked, this, &SonoControlWindow::rescanHyusDevices);
        lblHyusStatus_ = new QLabel("Detecting devices…");
        lblHyusStatus_->setWordWrap(true); lblHyusStatus_->setStyleSheet("color:#667085; font-size:12px;");
        gh->addWidget(new QLabel("Device"),0,0); gh->addWidget(cmbHyusDevice_,0,1); gh->addWidget(scanHyus,0,2);
        gh->addWidget(lblHyusStatus_,1,0,1,3);
        auto* hyusNote = new QLabel("Connect the device by RJ45 and set the PC wired network adapter to 192.168.0.x. "
                                    "The device appears here automatically once it connects.");
        hyusNote->setWordWrap(true); hyusNote->setStyleSheet("color:#667085; font-size:12px;");
        gh->addWidget(hyusNote,2,0,1,3);
        v->addWidget(grpUsHyus_);
        grpUsFpga_->setVisible(!hyus);
        grpUsHyus_->setVisible(hyus);
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

        // Web server controls. The group is always present; if the build
        // does not include Qt Network it is permanently disabled with a
        // single explanatory label so the operator can see *why* the feature
        // is not available rather than wondering whether it was removed.
        auto* webGrp = new QGroupBox("Monitoring Web Server");
        auto* wv = new QVBoxLayout(webGrp);
#if SONOCONTROL_WEB_SERVER
        chkWebServer_ = new QCheckBox("Enable web server (session only — never auto-starts on launch)");
        wv->addWidget(chkWebServer_);
        auto* webRow1 = new QHBoxLayout;
        webRow1->addWidget(new QLabel("Port"));
        spnWebPort_ = ispin(1024, 65535, 50896);
        spnWebPort_->setMinimumWidth(100);
        webRow1->addWidget(spnWebPort_);
        webRow1->addSpacing(16);
        webRow1->addWidget(new QLabel("Snapshot interval"));
        spnWebInterval_ = ispin(5, 3600, 900);
        spnWebInterval_->setSuffix(" s");
        spnWebInterval_->setMinimumWidth(110);
        webRow1->addWidget(spnWebInterval_);
        webRow1->addStretch();
        wv->addLayout(webRow1);
        chkWebLan_ = new QCheckBox("Allow LAN access — page is reachable from other devices on this network");
        chkWebLan_->setToolTip("Off (default): binds 127.0.0.1 — local browser only.\n"
                               "On: binds all interfaces so the wireless NIC's LAN address serves the page.\n"
                               "There is no authentication or HTTPS; enable only on trusted networks.");
        wv->addWidget(chkWebLan_);
        lblWebStatus_ = new QLabel("Stopped");
        lblWebStatus_->setStyleSheet("color:#475467; font-size:12px; font-family:Consolas,monospace;");
        lblWebStatus_->setTextInteractionFlags(Qt::TextSelectableByMouse);
        lblWebStatus_->setWordWrap(true);
        wv->addWidget(lblWebStatus_);
        connect(chkWebServer_, &QCheckBox::toggled, this, &SonoControlWindow::onWebServerEnabledToggled);
        connect(spnWebPort_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int){ onWebServerSettingsChanged(); });
        connect(spnWebInterval_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v){ if (webServer_) webServer_->setSnapshotIntervalSeconds(v); });
        connect(chkWebLan_, &QCheckBox::toggled, this, [this](bool){ onWebServerSettingsChanged(); });
#else
        auto* note = new QLabel("This build was compiled without Qt Network — the monitoring web server is unavailable.");
        note->setWordWrap(true);
        note->setStyleSheet("color:#98a2b3; font-size:12px;");
        wv->addWidget(note);
#endif
        v->addWidget(webGrp);

        v->addStretch(); return w;
    }

    // buildExportPanel() is an alternative compact layout kept for reference.
    // It is NOT called from buildUi() — the buttons it contains are available
    // via the menu bar. Do NOT assign lblConfigStatus_ here: that pointer is
    // already owned by buildLeftPanel(), and a second assignment would orphan
    // the first widget and break all updateConfigStatus() calls.
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
        auto* statusLbl = new QLabel("Config: GUI defaults");
        statusLbl->setWordWrap(true);
        statusLbl->setStyleSheet("color:#667085; font-size:12px;");
        g->addWidget(load,0,0); g->addWidget(save,0,1);
        g->addWidget(templ,1,0); g->addWidget(logs,1,1);
        g->addWidget(csv,2,0); g->addWidget(graph,2,1);
        g->addWidget(statusLbl,3,0,1,2);
        return grp;
    }

    QWidget* buildCenterPanel() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* temp = new QGroupBox("Temperature"); auto* tv = new QVBoxLayout(temp); tempPlot_ = new TemperaturePlot; tv->addWidget(tempPlot_); v->addWidget(temp, 2);
        {
            // Both devices use the pulse-sequence schematic.
            auto* wf = new QGroupBox("Pulse Sequence"); auto* wv = new QVBoxLayout(wf);
            seqPlot_ = new PulseSequencePlot; wv->addWidget(seqPlot_); v->addWidget(wf, 1);
        }
        return w;
    }

    QWidget* buildRightPanel() {
        auto* w = new QWidget; auto* v = new QVBoxLayout(w); v->setContentsMargins(0,0,0,0); v->setSpacing(12);
        auto* timer = new QGroupBox("Session Timer"); auto* tg = new QGridLayout(timer); lblElapsed_ = statLabel("00:00:00"); lblRemaining_ = statLabel("--:--:--"); lblCycle_ = statLabel("IDLE", "#5a6070"); addStat(tg,0,"Elapsed",lblElapsed_); addStat(tg,1,"Remaining",lblRemaining_); addStat(tg,2,"Phase",lblCycle_); v->addWidget(timer);
        auto* cur = new QGroupBox("Temperature"); auto* cg = new QGridLayout(cur); lblT1_ = statLabel("--.-°C"); lblT2_ = statLabel("--.-°C"); lblTavg_ = statLabel("--.-°C"); addStat(cg,0,"T1",lblT1_); addStat(cg,1,"T2",lblT2_); addStat(cg,2,"Avg",lblTavg_); v->addWidget(cur);
        auto* active = new QGroupBox("Active Parameters"); auto* ag = new QGridLayout(active);
        // The six value labels are reused for both devices; only the row titles
        // and the values written into them differ. For Hyus they are populated
        // by updateSequencePreview() (the FPGA runtime's onParams() does not run).
        lblCurAmp_=statLabel("--"); lblCurCfreq_=statLabel("--"); lblCurPrf_=statLabel("--"); lblCurDuty_=statLabel("--"); lblCurDur_=statLabel("--"); lblCurIntv_=statLabel("--");
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) {
            addStat(ag,0,"Pulse Amp",lblCurAmp_); addStat(ag,1,"Pulse Len",lblCurCfreq_); addStat(ag,2,"Pulse Period",lblCurPrf_); addStat(ag,3,"Seq Len",lblCurDuty_); addStat(ag,4,"Seq Period",lblCurDur_); addStat(ag,5,"Repeating",lblCurIntv_);
        } else {
            addStat(ag,0,"Amplitude",lblCurAmp_); addStat(ag,1,"CFreq",lblCurCfreq_); addStat(ag,2,"PRF",lblCurPrf_); addStat(ag,3,"Duty",lblCurDuty_); addStat(ag,4,"Duration",lblCurDur_); addStat(ag,5,"Interval",lblCurIntv_);
        }
        v->addWidget(active); v->addStretch(); return w;
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
#ifndef _WIN32
        // On Windows the port list is already sorted numerically (COM3, COM5,
        // COM10 …) by the std::sort at the end of the WIN32 block above.
        // Applying Qt's lexicographic sort here would overwrite that order and
        // produce COM10 < COM3. Only apply it on non-Windows where the ports
        // are ttyUSB* / ttyACM* strings that sort naturally as text.
        out.sort();
#endif
        return out;
    }


    // HH806AU readiness check for the Hyus device. Mirrors the temperature
    // portion of the FPGA runHardwarePreflight: hard-fail when PID needs the
    // sensor, otherwise offer to continue without monitoring.
    bool hyusTemperaturePreflight(sonocontrol::Config& cfg) {
        if (!cfg.temperature_enabled || cfg.simulate_temp) return true;
        if (cmbCom11_ && (QString::fromStdString(cfg.com11_port).trimmed().isEmpty() ||
                          cmbCom11_->currentText().contains("no ports", Qt::CaseInsensitive))) {
            if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
                QMessageBox::critical(this, "Missing Temperature Port",
                    "This mode requires temperature feedback. Select a valid HH806AU COM port.");
                return false;
            }
            const auto choice = QMessageBox::warning(this, "Temperature Sensor Not Available",
                "Temperature monitoring is enabled, but no valid HH806AU COM port is selected.\n\n"
                "Continue without temperature monitoring and software cutoff for this run?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (choice != QMessageBox::Yes) return false;
            cfg.temperature_enabled = false; cfg.simulate_temp = false;
            return true;
        }
        try {
            sonocontrol::HH806AUSensor sensor(cfg.com11_port, cfg.min_plausible_temp_c,
                                              cfg.max_plausible_temp_c, cfg.max_temp_rate_c_per_s);
            std::optional<double> t1, t2;
            for (int a = 0; a < 3; ++a) {
                if (a) QThread::msleep(150);
                auto pr = sensor.read(); t1 = pr.first; t2 = pr.second;
                if (t1 || t2) break;
            }
            if (!(t1 || t2)) {
                if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) {
                    QMessageBox::critical(this, "Preflight Failed",
                        "This mode requires temperature feedback, but HH806AU returned no valid data after 3 attempts.");
                    return false;
                }
                const auto choice = QMessageBox::warning(this, "Temperature Monitor Not Ready",
                    "HH806AU returned no valid data after 3 attempts.\n\n"
                    "Continue without temperature monitoring and software cutoff?",
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
                if (choice != QMessageBox::Yes) return false;
                cfg.temperature_enabled = false; cfg.simulate_temp = false;
            } else {
                appendConsole(QString("Preflight: HH806AU OK  T1=%1  T2=%2")
                    .arg(t1 ? QString::number(*t1, 'f', 2) + QString::fromUtf8("°C") : "N/C")
                    .arg(t2 ? QString::number(*t2, 'f', 2) + QString::fromUtf8("°C") : "N/C"));
            }
        } catch (const std::exception& e) {
            if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) { QMessageBox::critical(this, "Preflight Failed", qstr(e.what())); return false; }
            const auto choice = QMessageBox::warning(this, "Temperature Monitor Not Ready",
                QString("HH806AU could not be opened/read:\n\n") + qstr(e.what()) +
                "\n\nContinue without temperature monitoring and software cutoff?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (choice != QMessageBox::Yes) return false;
            cfg.temperature_enabled = false; cfg.simulate_temp = false;
        }
        return true;
    }

    bool preflightCheckHyus(sonocontrol::Config& cfg) {
        if (cfg.hyus_pulse_len_us > cfg.hyus_pulse_period_us) {
            QMessageBox::warning(this, "Invalid Timing", "Pulse length must be ≤ pulse period. The run was not started.");
            return false;
        }
        if (cfg.hyus_seq_len_ms > cfg.hyus_seq_period_ms) {
            QMessageBox::warning(this, "Invalid Timing", "Sequence length must be ≤ sequence period. The run was not started.");
            return false;
        }
        if (cfg.hyus_pulse_period_us > cfg.hyus_seq_len_ms * 1000.0) {
            QMessageBox::warning(this, "Invalid Timing", "Pulse period must be ≤ sequence length. The run was not started.");
            return false;
        }
        if ((cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) && cfg.cutoff_temp <= cfg.pid_setpoint) {
            QMessageBox::warning(this, "Invalid Safety Limit", "Cutoff temperature must be higher than the target temperature.");
            return false;
        }
        if (cfg.use_cycling && cfg.length_mode != sonocontrol::LengthMode::TotalDuration) {
            QMessageBox::warning(this, "Invalid Cycle Mode", "Temperature cycling is only available in Total Duration mode.");
            return false;
        }
        if (cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget && !cfg.temperature_enabled) {
            QMessageBox::warning(this, "Temperature Required",
                                 "After-target Hold mode requires temperature monitoring, because the hold timer starts only after the target temperature is reached.");
            return false;
        }
        if (!hyusTemperaturePreflight(cfg)) return false;
        return showPreflightConfirmation(cfg);
    }

    bool preflightCheck(sonocontrol::Config& cfg) {
        const double duration_s = spnDuration_->value() / 1000.0;
        if (spnInterval_->value() < duration_s) {
            QMessageBox::warning(this, "Invalid Timing",
                                 "Interval must be greater than or equal to Duration. The experiment was not started.");
            return false;
        }
        // Pulse period (1/PRF) must fit within the sequence length (Duration).
        if (cfg.prf_hz > 0.0 && (1.0 / cfg.prf_hz) > duration_s) {
            QMessageBox::warning(this, "Invalid Timing",
                                 "Pulse period (1/PRF) must be less than or equal to Duration (sequence length). "
                                 "Increase Duration or PRF. The experiment was not started.");
            return false;
        }
        // Soft warning when the interval is below the realistic transmit
        // floor. The validate_config clamp already enforced >= 0.2 s; this
        // dialog informs the operator so they understand why cycles may be
        // longer than configured in their CSV.
        if (cfg.interval_time_s < 0.5) {
            const auto choice = QMessageBox::warning(this, "Very Short Interval",
                QString("Interval = %1 s is below the typical transmit time for a single burst "
                        "(serial CFREQ/PRF/DUR with 100 ms COM gaps + 4096 UDP datagrams).\n\n"
                        "Cycles will likely be spaced wider than requested. Continue?").arg(cfg.interval_time_s, 0, 'f', 2),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (choice != QMessageBox::Yes) return false;
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

    QString configSummaryHyus(const sonocontrol::Config& cfg) const {
        const double seqp_s = cfg.hyus_seq_period_ms / 1000.0;
        const double total_s = (cfg.length_mode == sonocontrol::LengthMode::TotalDuration)
            ? cfg.total_duration_mins * 60.0
            : ((cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles)
                ? static_cast<double>(cfg.repeating) * seqp_s
                : cfg.hold_after_target_mins * 60.0);
        const char* pidVar = (cfg.hyus_pid_var == 0) ? "pulse amplitude"
                           : (cfg.hyus_pid_var == 2) ? "sequence length/period" : "pulse length/period";
        QString s;
        s += "Hyus device (LAN / Ethernet)\n";
        s += QString("  Device IP: %1\n\n").arg(QString::fromStdString(cfg.hyus_device_ip));
        s += "Ultrasound output\n";
        s += QString("  Pulse frequency: %1 MHz\n").arg(cfg.cfreq_hz / 1.0e6, 0, 'f', 3);
        s += QString("  Pulse amplitude: %1 %\n").arg(cfg.amplitude * 100.0, 0, 'f', 1);
        s += QString("  Pulse length / period: %1 / %2 us\n").arg(cfg.hyus_pulse_len_us, 0, 'f', 1).arg(cfg.hyus_pulse_period_us, 0, 'f', 1);
        s += QString("  Sequence length / period: %1 / %2 ms\n\n").arg(cfg.hyus_seq_len_ms, 0, 'f', 3).arg(cfg.hyus_seq_period_ms, 0, 'f', 3);
        s += "Experiment length\n";
        if (cfg.length_mode == sonocontrol::LengthMode::TotalDuration) {
            s += QString("  Mode: Total duration\n");
            s += QString("  Planned total: %1 s (%2 h), repeating %3 cycles\n").arg(total_s, 0, 'f', 1).arg(total_s / 3600.0, 0, 'f', 2).arg(seqp_s > 0 ? static_cast<int>(total_s / seqp_s) : 0);
            s += QString("  Cycling: %1\n").arg(cfg.use_cycling ? "enabled" : "disabled");
        } else if (cfg.length_mode == sonocontrol::LengthMode::RepeatingCycles) {
            s += QString("  Mode: Repeating cycles\n");
            s += QString("  Repeating: %1 cycles (≈ %2 s)\n").arg(cfg.repeating).arg(total_s, 0, 'f', 1);
        } else {
            s += QString("  Mode: After target reached, hold\n");
            s += QString("  Hold time after first target reach: %1 min (%2 s)\n").arg(cfg.hold_after_target_mins, 0, 'f', 1).arg(total_s, 0, 'f', 1);
            s += QString("  Target reach criterion: setpoint ± %1 °C\n").arg(cfg.target_tolerance_c, 0, 'f', 2);
            s += "  Total duration is not used in this mode.\n";
        }
        s += "\nTemperature / safety\n";
        s += QString("  Temperature monitoring: %1\n").arg(cfg.temperature_enabled ? "enabled" : "disabled");
        s += QString("  HH806AU: %1\n").arg(cfg.temperature_enabled ? (QString::fromStdString(cfg.com11_port) + (cfg.simulate_temp ? " [SIM]" : "")) : "not used");
        s += QString("  Sampling rate: %1 Hz\n").arg(cfg.sampling_rate_hz, 0, 'f', 1);
        s += QString("  PID: %1%2\n").arg(cfg.pid_enabled ? "enabled" : "disabled").arg(cfg.pid_enabled ? (QString(" — controls ") + pidVar) : QString());
        if (cfg.pid_enabled || cfg.length_mode == sonocontrol::LengthMode::HoldAfterTarget) s += QString("  Target setpoint: %1 °C\n").arg(cfg.pid_setpoint, 0, 'f', 1);
        s += QString("  Cutoff: %1 °C%2 (%3 samples)\n").arg(cfg.cutoff_temp, 0, 'f', 1).arg(cfg.temperature_enabled ? "" : " [inactive]").arg(cfg.cutoff_confirm_samples);
        s += QString("  Config source: %1%2\n").arg(QString::fromStdString(cfg.config_source_type)).arg(cfg.config_file_path.empty() ? "" : (" [" + QString::fromStdString(cfg.config_file_path) + "]"));
        return s;
    }

    QString configSummary(const sonocontrol::Config& cfg) const {
        if (cfg.device_kind == sonocontrol::DeviceKind::Hyus) return configSummaryHyus(cfg);
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
        s += QString("  Temp smoothing window: %1 s (dT/dt least-squares fit span)\n")
                 .arg(cfg.temp_rate_window_s, 0, 'f', 2);
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
        dlg.resize(840, 780);
        auto* layout = new QVBoxLayout(&dlg);
        auto* title = new QLabel("Review settings before starting");
        title->setStyleSheet("font-size:18px; font-weight:700;");
        layout->addWidget(title);

        auto* summary = new QPlainTextEdit;
        summary->setReadOnly(true);
        summary->setPlainText(configSummary(cfg));
        summary->setStyleSheet("font-family:Consolas,monospace; font-size:12px;");
        layout->addWidget(summary, 1);

        // Session-scoped manual-stop PIN. Independent of any persisted config —
        // intentionally re-entered for every run so a stale PIN from a previous
        // operator can't block a hand-off. The PIN gates the manual STOP button
        // only; the watchdog and force-stop escalation always proceed.
        auto* lockGrp = new QGroupBox("Manual-stop PIN protection");
        auto* lockGrid = new QGridLayout(lockGrp);
        lockGrid->setHorizontalSpacing(14);
        lockGrid->setVerticalSpacing(10);
        auto* chkLock = new QCheckBox("Require username + 4-digit PIN to stop this run manually");
        chkLock->setChecked(pendingPinEnabled_);
        auto* lblUser = new QLabel("Username");
        auto* txtUser = new QLineEdit;
        txtUser->setMaxLength(48);
        txtUser->setPlaceholderText("e.g. Lab Operator");
        txtUser->setText(pendingPinUsername_);
        auto* lblPin = new QLabel("4-digit PIN");
        auto* pinEntry = new PinEntry(4, /*large=*/false);
        if (!pendingPin_.isEmpty()) pinEntry->setFromString(pendingPin_);
        auto* lockErr = new QLabel(" ");
        lockErr->setStyleSheet("color:#d93025; font-size:12px;");
        auto* lockNote = new QLabel("If enabled, clicking STOP opens a prompt that shows the username and requires the PIN. The 2-second force-stop escalation and the stall watchdog still bypass the prompt for safety.");
        lockNote->setWordWrap(true);
        lockNote->setStyleSheet("color:#667085; font-size:12px;");
        lockGrid->addWidget(chkLock, 0, 0, 1, 2);
        lockGrid->addWidget(lblUser, 1, 0);
        lockGrid->addWidget(txtUser, 1, 1);
        lockGrid->addWidget(lblPin, 2, 0);
        lockGrid->addWidget(pinEntry, 2, 1, Qt::AlignLeft);
        lockGrid->addWidget(lockErr, 3, 0, 1, 2);
        lockGrid->addWidget(lockNote, 4, 0, 1, 2);
        layout->addWidget(lockGrp);

        auto setLockEnabled = [&, pinEntry](bool on) {
            for (auto* w : std::initializer_list<QWidget*>{lblUser, txtUser, lblPin}) {
                w->setEnabled(on);
            }
            pinEntry->setEntryEnabled(on);
        };
        setLockEnabled(chkLock->isChecked());
        connect(chkLock, &QCheckBox::toggled, &dlg, [setLockEnabled](bool on){ setLockEnabled(on); });
        // Clear any prior error as soon as the operator edits either field.
        connect(txtUser, &QLineEdit::textEdited, lockErr, [lockErr](const QString&){ lockErr->setText(" "); });
        connect(pinEntry, &PinEntry::changed, lockErr, [lockErr]{ lockErr->setText(" "); });

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
        connect(buttons, &QDialogButtonBox::accepted, &dlg, [&, pinEntry, lockErr]() {
            if (chkLock->isChecked()) {
                const QString user = txtUser->text().trimmed();
                const QString pin = pinEntry->pin();
                if (user.isEmpty()) {
                    lockErr->setText("Enter a username (shown on the stop prompt).");
                    txtUser->setFocus();
                    return;
                }
                static const QRegularExpression kPinPattern("^[0-9]{4}$");
                if (!kPinPattern.match(pin).hasMatch()) {
                    lockErr->setText("Enter all 4 digits of the PIN.");
                    pinEntry->focusFirst();
                    return;
                }
            }
            dlg.accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        // PC power status, shown large just left of the Start button, refreshed
        // while the dialog is open so unplugging mains is noticed immediately.
        auto* battery = new BatteryIndicator;
        battery->setStatus(acLineStatus());
        auto* powerTimer = new QTimer(&dlg);
        powerTimer->setInterval(2000);
        connect(powerTimer, &QTimer::timeout, battery, [battery]{ battery->setStatus(acLineStatus()); });
        powerTimer->start();
        auto* btnRow = new QHBoxLayout;
        btnRow->addStretch();
        btnRow->addWidget(battery, 0, Qt::AlignVCenter);
        btnRow->addSpacing(12);
        btnRow->addWidget(buttons);
        layout->addLayout(btnRow);

        if (dlg.exec() != QDialog::Accepted) return false;

        // Commit the PIN settings for this run. Cleared in onFinished() so the
        // next run requires re-entry.
        pendingPinEnabled_ = chkLock->isChecked();
        pendingPinUsername_ = pendingPinEnabled_ ? txtUser->text().trimmed() : QString();
        pendingPin_ = pendingPinEnabled_ ? pinEntry->pin() : QString();
        return true;
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
            if (seqPlot_) {
                QPixmap wavePix(seqPlot_->size());
                seqPlot_->render(&wavePix);
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
        const QString status = (code == 0) ? "completed" : (code == 2 ? "stopped by safety cutoff" : (code == 3 ? "stopped manually" : (code == 4 ? "stopped automatically (communication stall)" : "aborted")));
        QString msg;
        msg += QString("Experiment %1.\n\n").arg(status);
        if (code == 4) {
            msg += "The device stopped responding for longer than the stall "
                   "watchdog timeout, so the run was ended automatically and an "
                   "emergency STOP was sent. No operator action was taken.\n\n"
                   "This usually means the ultrasound/FPGA link (UDP) or the COM3 "
                   "serial connection briefly wedged. Check the network/USB cable, "
                   "the device, and the UDP host/port before restarting. If the "
                   "device is simply slow, raise watchdog_timeout_ms.\n\n";
        }
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
        // Widen the dynamically-constrained ranges before loading so a valid
        // saved config is never clamped by ranges left over from prior edits.
        if (spnDuration_) { spnDuration_->setMinimum(10.0); spnDuration_->setMaximum(60000.0); }
        if (spnInterval_) { spnInterval_->setMinimum(sonocontrol::kMinIntervalTimeS); spnInterval_->setMaximum(3600.0); }
        for (auto* s : {spnHyusPulseLenUs_, spnHyusPulsePeriodUs_}) if (s) { s->setMinimum(0.1); s->setMaximum(1000000.0); }
        for (auto* s : {spnHyusSeqLenMs_, spnHyusSeqPeriodMs_}) if (s) { s->setMinimum(0.001); s->setMaximum(1000000.0); }
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
        if (chkTempRequired_) chkTempRequired_->setChecked(cfg.temperature_required);
#if SONOCONTROL_WEB_SERVER
        // Port / interval / LAN flag persist; the enable flag deliberately
        // does not (see config.hpp). Loading a config will never silently
        // turn on a LAN-reachable endpoint.
        if (spnWebPort_) spnWebPort_->setValue(static_cast<int>(cfg.web_server_port));
        if (spnWebInterval_) spnWebInterval_->setValue(cfg.web_server_snapshot_interval_s);
        if (chkWebLan_) chkWebLan_->setChecked(cfg.web_server_lan);
#endif
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
        enforceRateWindowRange();  // floor depends on the sample rate set above
        if (spnRateWindow_) spnRateWindow_->setValue(cfg.temp_rate_window_s);
        autoSaveDir_ = qstr(cfg.auto_save_dir);
        chkCycling_->setChecked(cfg.use_cycling);
        // Config stores phase durations in seconds; spinboxes display minutes.
        updateCyclePhaseCaps();
        spnHeatS_->setValue(cfg.heating_s / 60.0);
        spnCoolS_->setValue(cfg.cooling_s / 60.0);
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
        // Restore Hyus parameters (config_io persists these for the Hyus device).
        if (spnHyusFreqMhz_) spnHyusFreqMhz_->setValue(cfg.cfreq_hz / 1.0e6);
        if (spnHyusAmpPct_) spnHyusAmpPct_->setValue(cfg.amplitude * 100.0);
        if (spnHyusPulseLenUs_) spnHyusPulseLenUs_->setValue(cfg.hyus_pulse_len_us);
        if (spnHyusPulsePeriodUs_) spnHyusPulsePeriodUs_->setValue(cfg.hyus_pulse_period_us);
        if (spnHyusSeqLenMs_) spnHyusSeqLenMs_->setValue(cfg.hyus_seq_len_ms);
        if (spnHyusSeqPeriodMs_) spnHyusSeqPeriodMs_->setValue(cfg.hyus_seq_period_ms);
        if (rbHyusPidAmp_ && rbHyusPidPulse_ && rbHyusPidSeq_) {
            if (cfg.hyus_pid_var == 0) rbHyusPidAmp_->setChecked(true);
            else if (cfg.hyus_pid_var == 2) rbHyusPidSeq_->setChecked(true);
            else rbHyusPidPulse_->setChecked(true);
        }
        updateRepeatingFromDuration();
        updateLengthModeUi();
        onPidChanged(chkPid_->isChecked());
        onTempMonitoringChanged(chkTempMonitor_ && chkTempMonitor_->isChecked());
        enforceHyusTiming();
        enforceZhuhaiTiming();
        updateSequencePreview();
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
        c.temperature_required = chkTempRequired_ && chkTempRequired_->isChecked();
#if SONOCONTROL_WEB_SERVER
        // Web server: do NOT propagate the live enable flag into Config.
        // Persisting it could let a shared .config auto-start a LAN-reachable
        // server on launch elsewhere. Port / interval / LAN flag are fine.
        if (spnWebPort_) c.web_server_port = static_cast<uint16_t>(spnWebPort_->value());
        if (spnWebInterval_) c.web_server_snapshot_interval_s = spnWebInterval_->value();
        if (chkWebLan_) c.web_server_lan = chkWebLan_->isChecked();
        c.web_server_enabled = false;  // session-only
#endif
        c.pid_setpoint = spnSetpoint_->value(); c.pid_amplitude = chkPidAmp_->isChecked(); c.pid_duration = chkPidDuration_->isChecked(); c.pid_duty = chkPidDuty_->isChecked(); c.pid_interval = chkPidInterval_->isChecked(); c.pid_kp = spnKp_->value(); c.pid_ki = spnKi_->value(); c.pid_kd = spnKd_->value(); c.pid_prediction_tau_s = spnTau_ ? spnTau_->value() : 25.0; c.pid_prediction_horizon_s = spnHorizon_ ? spnHorizon_->value() : 0.0; c.temp_rate_window_s = spnRateWindow_ ? spnRateWindow_->value() : 30.0; c.auto_save_dir = autoSaveDir_.toStdString();
        c.use_cycling = chkCycling_->isChecked();
        // Spinboxes are in minutes; Config carries seconds (interface unchanged).
        c.heating_s = spnHeatS_->value() * 60.0;
        c.cooling_s = spnCoolS_->value() * 60.0;
        c.cooling_mode = rbCoolStop_->isChecked() ? sonocontrol::CoolingMode::Stop : sonocontrol::CoolingMode::Low; c.cooling_hold_temp = spnCoolHoldTemp_->value();
        c.com3_port = cmbCom3_->currentData().toString().toStdString(); c.com11_port = cmbCom11_->currentData().toString().toStdString(); c.temp_channel = static_cast<sonocontrol::TempChannel>(cmbTempChannel_->currentIndex()); c.temp_channel_fallback = chkTempFallback_ && chkTempFallback_->isChecked(); c.udp_host = txtUdpHost_->text().trimmed().toStdString(); c.udp_port = static_cast<uint16_t>(spnUdpPort_->value());
        c.simulate_temp = c.temperature_enabled && debugSimChecked(chkSimTemp_); c.simulate_us = debugSimChecked(chkSimUs_);

        // --- Device selection + Hyus transport fields ---
        c.device_kind = deviceKind_;
        // IP comes from the auto-detected device dropdown; the TCP/beacon ports
        // are protocol constants and are intentionally not user-exposed (kept at
        // the config defaults 8192/8193).
        if (cmbHyusDevice_ && !cmbHyusDevice_->currentText().trimmed().isEmpty())
            c.hyus_device_ip = cmbHyusDevice_->currentText().trimmed().toStdString();
        // Hyus device total-duration mode is only used for Total Duration.
        // Repeating and After-target Hold are software-timed and stopped by
        // the worker with RUN=0.
        c.hyus_run_mode = (c.length_mode == sonocontrol::LengthMode::TotalDuration) ? 1 : 0;
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) {
            // Hyus reuses amplitude/cfreq_hz but takes them from its own inputs
            // (carrier in MHz, amplitude in percent), and carries pulse/sequence
            // timing in dedicated fields with no FPGA analog.
            if (spnHyusFreqMhz_) c.cfreq_hz = spnHyusFreqMhz_->value() * 1.0e6;
            if (spnHyusAmpPct_) c.amplitude = std::clamp(spnHyusAmpPct_->value() / 100.0, 0.0, 1.0);
            if (spnHyusPulseLenUs_) c.hyus_pulse_len_us = spnHyusPulseLenUs_->value();
            if (spnHyusPulsePeriodUs_) c.hyus_pulse_period_us = spnHyusPulsePeriodUs_->value();
            if (spnHyusSeqLenMs_) c.hyus_seq_len_ms = spnHyusSeqLenMs_->value();
            if (spnHyusSeqPeriodMs_) c.hyus_seq_period_ms = spnHyusSeqPeriodMs_->value();
            // PID controlled variable (single-select). Default pulse-duty.
            c.hyus_pid_var = (rbHyusPidAmp_ && rbHyusPidAmp_->isChecked()) ? 0
                           : (rbHyusPidSeq_ && rbHyusPidSeq_->isChecked()) ? 2 : 1;
        }
        return c;
    }

    void setupStatusProbe() {
        // Runs for both devices. For Hyus the COM3/UDP checks are suppressed in
        // pushProbeSettings() (those pills are hidden); only the temperature
        // probe is relevant, feeding the restored Temp pill.
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
        const bool hyus = (deviceKind_ == sonocontrol::DeviceKind::Hyus);
        // Hyus has no FPGA serial/UDP — suppress those checks (their pills are
        // hidden); only the temperature probe is relevant.
        s.com3_port = (hyus || !cmbCom3_) ? QString() : cmbCom3_->currentData().toString();
        s.com11_port = cmbCom11_ ? cmbCom11_->currentData().toString() : QString();
        s.udp_host = (hyus || !txtUdpHost_) ? QString() : txtUdpHost_->text().trimmed();
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
        // watchdogStop (not forceStop): this is an automatic recovery, so it
        // must report as a comms-stall stop (code 4), not an operator stop.
        worker_->watchdogStop();
        btnStop_->setText(" ■  FORCE STOP ");
        btnStop_->setEnabled(true);
    }

    void touchWorkerSignal() {
        lastWorkerSignalMs_ = QDateTime::currentMSecsSinceEpoch();
        workerStallNotified_ = false;
#if SONOCONTROL_WEB_SERVER
        // Backfill the canonical session id the moment the worker reports
        // its first signal — by that point logger.start_session has run on
        // the worker thread and session_id() is non-empty. The web server's
        // updateSessionId is a no-op if the value hasn't changed, so calling
        // this on every signal is microseconds.
        if (webServer_ && running_) {
            const QString id = QString::fromStdString(logger_.session_id());
            if (!id.isEmpty()) webServer_->updateSessionId(id);
        }
#endif
    }

    // ── Sequence runner ────────────────────────────────────────────────────
    void openSequenceDialog() {
        // The dialog is non-modal and reusable across opens. Closing the
        // dialog while a sequence is running does not stop the sequence — the
        // controller state lives on this window. When the dialog is reopened
        // it re-binds to that live state.
        if (!sequenceDialog_) {
            sequenceDialog_ = new SequenceDialog(this);
            connect(sequenceDialog_, &SequenceDialog::startRequested, this, &SonoControlWindow::onSequenceStartRequested);
            connect(sequenceDialog_, &SequenceDialog::stopRequested, this, &SonoControlWindow::onSequenceStopRequested);
        }
        sequenceDialog_->seedState(savedSequenceItems_, savedSequenceIntervalsMin_);
        sequenceDialog_->setRunningState(sequenceActive_);
        if (sequenceActive_) {
            sequenceDialog_->setCurrentIndex(sequenceIndex_);
            sequenceDialog_->setStatusText(QString("Sequence running: item %1 of %2.")
                                              .arg(sequenceIndex_ + 1).arg(sequenceTotal_));
        }
        sequenceDialog_->show();
        sequenceDialog_->raise();
        sequenceDialog_->activateWindow();
    }

    void onSequenceStartRequested() {
        if (!sequenceDialog_) return;
        if (sequenceActive_ || running_) return;
        const auto items = sequenceDialog_->items();
        if (items.isEmpty()) return;
        // Device match: every queued configuration must target the current
        // device, otherwise the run would use the wrong transport/parameters.
        {
            QStringList bad;
            for (const auto& it : items)
                if (it.cfg.device_kind != deviceKind_) bad << QFileInfo(it.path).fileName();
            if (!bad.isEmpty()) {
                QMessageBox::critical(this, "Device mismatch",
                    QString("These queued configurations target a different device than the current "
                            "'%1' session and cannot be run:\n\n%2\n\nRemove them (or switch device), then retry.")
                        .arg(deviceDisplayName(deviceKind_))
                        .arg(bad.join("\n")));
                return;
            }
        }
        savedSequenceItems_ = items;
        savedSequenceIntervalsMin_ = sequenceDialog_->intervalsMinutes();
        // Defensive normalization — should already match because seedState/
        // rebuild keep them in sync, but a sequence with N items must have
        // exactly N-1 interval entries before we start consuming them.
        const int wanted = std::max(0, static_cast<int>(items.size()) - 1);
        while (savedSequenceIntervalsMin_.size() < wanted) savedSequenceIntervalsMin_.append(5);
        while (savedSequenceIntervalsMin_.size() > wanted) savedSequenceIntervalsMin_.removeLast();
        sequencePinEnabled_ = sequenceDialog_->pinEnabled();
        sequencePinUsername_ = sequencePinEnabled_ ? sequenceDialog_->pinUsername() : QString();
        sequencePin_ = sequencePinEnabled_ ? sequenceDialog_->pinValue() : QString();
        sequenceStopOnError_ = sequenceDialog_->stopOnError();
        sequenceActive_ = true;
        sequenceStopRequested_ = false;
        sequenceIndex_ = 0;
        sequenceTotal_ = static_cast<int>(items.size());
        // Lock the main page buttons for the entire sequence; the sequence
        // dialog provides its own start/stop.
        btnStart_->setEnabled(false);
        btnStop_->setEnabled(false);
        QString gapStr;
        if (savedSequenceIntervalsMin_.isEmpty()) {
            gapStr = "no gaps";
        } else {
            QStringList parts;
            for (int v : savedSequenceIntervalsMin_) parts << QString::number(v);
            gapStr = "intervals " + parts.join("/") + " min";
        }
        appendConsole(QString("=== Sequence started: %1 configuration(s), %2 ===")
                          .arg(sequenceTotal_).arg(gapStr));
        statusBar()->showMessage("Sequence running");
        if (sequenceDialog_) {
            sequenceDialog_->setRunningState(true);
            sequenceDialog_->setCurrentIndex(0);
            sequenceDialog_->setStatusText("Starting first configuration…");
        }
        launchCurrentSequenceItem();
    }

    void onSequenceStopRequested() {
        if (!sequenceActive_) return;
        if (sequencePinEnabled_ && !promptForSequenceStopPin()) {
            appendConsole(">>> Sequence stop cancelled (PIN not confirmed)");
            statusBar()->showMessage("Sequence stop cancelled");
            return;
        }
        sequenceStopRequested_ = true;
        sequenceIntervalTimer_.stop();
        appendConsole(">>> Sequence STOP requested");
        statusBar()->showMessage("Stopping sequence");
        if (sequenceDialog_) sequenceDialog_->setStatusText("Stopping sequence…");
        if (worker_) {
            worker_->stop();
            // Escalate to force-stop after 2 s of unresponsive worker,
            // mirroring the main onStop() path. The watchdog escalation is
            // also active and provides an independent safety net.
            QTimer::singleShot(2000, this, [this]() {
                if (running_ && worker_) {
                    appendConsole(">>> Sequence stop escalating to force-stop (cancelling pending I/O)");
                    worker_->forceStop();
                }
            });
        } else if (hyusWorker_) {
            // Hyus item running: the worker sends RUN=0 and finishes, which
            // routes to handleSequenceItemFinished and finalizes (because
            // sequenceStopRequested_ is set).
            hyusWorker_->requestStop();
        } else {
            // No worker currently running (we're in the inter-config gap) —
            // finalize the sequence right away.
            finalizeSequence(/*ok=*/false);
        }
    }

    void launchCurrentSequenceItem() {
        if (!sequenceActive_) return;
        if (sequenceIndex_ < 0 || sequenceIndex_ >= savedSequenceItems_.size()) {
            finalizeSequence(/*ok=*/true);
            return;
        }
        sonocontrol::Config cfg = savedSequenceItems_[sequenceIndex_].cfg;
        cfg.config_source_type = "sequence";
        cfg.config_file_path = savedSequenceItems_[sequenceIndex_].path.toStdString();
        if (sequenceDialog_) {
            sequenceDialog_->setCurrentIndex(sequenceIndex_);
            sequenceDialog_->setStatusText(QString("Running %1 of %2: %3")
                                              .arg(sequenceIndex_ + 1).arg(sequenceTotal_)
                                              .arg(QFileInfo(savedSequenceItems_[sequenceIndex_].path).fileName()));
        }
        if (deviceKind_ == sonocontrol::DeviceKind::Hyus) {
            launchHyusSequenceItem(cfg);
            return;
        }
        validate_config(cfg);
        launchWorkerForConfig(cfg, /*fromSequence=*/true);
    }

    void handleSequenceItemFinished(int code) {
        if (!sequenceActive_) return;
        if (sequenceStopRequested_) { finalizeSequence(/*ok=*/false); return; }
        // Exit codes from ExperimentRunner: 0=clean, 1=error, 2=cutoff,
        // 3=manual stop, 4=watchdog comms-stall stop. Any non-zero code is a
        // "failed" config — abort the queue if the operator selected
        // stop-on-error.
        if (sequenceStopOnError_ && code != 0) {
            appendConsole(QString("=== [Sequence] Aborting: configuration %1 ended with code %2 and 'stop on error' is enabled ===")
                              .arg(sequenceIndex_ + 1).arg(code));
            if (sequenceDialog_) sequenceDialog_->setStatusText("Sequence aborted on configuration failure.");
            finalizeSequence(/*ok=*/false);
            return;
        }
        const bool wasLast = (sequenceIndex_ + 1 >= sequenceTotal_);
        if (wasLast) { finalizeSequence(/*ok=*/true); return; }
        // Schedule the next item after the per-gap interval (in minutes).
        // savedSequenceIntervalsMin_[k] is the interval that runs AFTER
        // items[k] finishes and BEFORE items[k+1] starts, so we index by the
        // index of the experiment that just finished. The 0-minute case still
        // goes through the timer for one event-loop tick so the GUI repaints
        // cleanly between configs.
        const int gapMin = (sequenceIndex_ >= 0 && sequenceIndex_ < savedSequenceIntervalsMin_.size())
                               ? savedSequenceIntervalsMin_[sequenceIndex_]
                               : 0;
        const qint64 delayMs = static_cast<qint64>(std::max(0, gapMin)) * 60 * 1000;
        if (sequenceDialog_) {
            sequenceDialog_->setStatusText(QString("Waiting %1 min before configuration %2 of %3…")
                                               .arg(gapMin).arg(sequenceIndex_ + 2).arg(sequenceTotal_));
        }
        appendConsole(QString("=== [Sequence] Waiting %1 min before next configuration ===").arg(gapMin));
        sequenceIntervalTimer_.stop();
        sequenceIntervalTimer_.setSingleShot(true);
        sequenceIntervalTimer_.setInterval(static_cast<int>(std::min<qint64>(delayMs, std::numeric_limits<int>::max())));
        sequenceIntervalTimer_.start();
    }

    void onSequenceIntervalElapsed() {
        if (!sequenceActive_ || sequenceStopRequested_) return;
        ++sequenceIndex_;
        launchCurrentSequenceItem();
    }

#if SONOCONTROL_WEB_SERVER
    // Lazy construction so users who never enable monitoring don't pay the
    // QTcpServer / QTimer cost. Reuses the same instance across toggles.
    sonocontrol::WebServer* ensureWebServer() {
        if (webServer_) return webServer_;
        webServer_ = new sonocontrol::WebServer(this);
        connect(webServer_, &sonocontrol::WebServer::statusChanged,
                this, &SonoControlWindow::refreshWebServerStatus);
        return webServer_;
    }

    void onWebServerEnabledToggled(bool enabled) {
        if (enabled) {
            auto* ws = ensureWebServer();
            ws->setPort(static_cast<uint16_t>(spnWebPort_->value()));
            ws->setSnapshotIntervalSeconds(spnWebInterval_->value());
            ws->setAllowLan(chkWebLan_->isChecked());
            if (!ws->start()) {
                QMessageBox::warning(this, "Web Server",
                    "Could not start the monitoring web server:\n\n" + ws->bindError() +
                    "\n\nThe port may already be in use, or the requested address may not be bindable.");
                QSignalBlocker b(chkWebServer_);
                chkWebServer_->setChecked(false);
                return;
            }
            appendConsole(QString("Web server started: %1").arg(ws->listenUrls().join(", ")));
        } else if (webServer_) {
            webServer_->stop();
            appendConsole("Web server stopped");
        }
        refreshWebServerStatus();
    }

    void onWebServerSettingsChanged() {
        // Port or LAN flag changed. If the server is running, rebind on the
        // new values. If it isn't, just remember the new values for the next
        // start.
        if (!webServer_ || !webServer_->isRunning()) return;
        webServer_->setPort(static_cast<uint16_t>(spnWebPort_->value()));
        webServer_->setAllowLan(chkWebLan_->isChecked());
        if (!webServer_->start()) {
            QMessageBox::warning(this, "Web Server",
                "Could not rebind the monitoring web server:\n\n" + webServer_->bindError());
            QSignalBlocker b(chkWebServer_);
            chkWebServer_->setChecked(false);
        }
        refreshWebServerStatus();
    }

    void refreshWebServerStatus() {
        if (!lblWebStatus_) return;
        if (!webServer_ || !webServer_->isRunning()) {
            lblWebStatus_->setText("Stopped");
            return;
        }
        QStringList urls = webServer_->listenUrls();
        lblWebStatus_->setText("Listening on:  " + urls.join("    "));
    }
#endif

    void finalizeSequence(bool ok) {
        const bool wasActive = sequenceActive_;
        sequenceActive_ = false;
        sequenceIntervalTimer_.stop();
        if (!wasActive) return;
        if (ok) {
            appendConsole("=== Sequence finished ===");
            statusBar()->showMessage("Sequence finished");
        } else {
            appendConsole("=== Sequence stopped ===");
            statusBar()->showMessage("Sequence stopped");
        }
        // Restore main-page buttons.
        btnStart_->setEnabled(true);
        btnStop_->setEnabled(false);
        if (sequenceDialog_) sequenceDialog_->setSequenceFinished(ok);
        // Drop the sequence PIN so a future sequence forces re-entry.
        sequencePinEnabled_ = false;
        sequencePinUsername_.clear();
        sequencePin_.clear();
        sequenceStopOnError_ = false;
        sequenceIndex_ = -1;
        sequenceTotal_ = 0;
        sequenceStopRequested_ = false;
    }

    // Sequence-specific PIN prompt. Identical UX to promptForStopPin() but
    // reads from the sequence credentials so a sequence can have its own
    // operator/PIN distinct from a per-run PIN entered on a previous (non-
    // sequence) run.
    bool promptForSequenceStopPin() {
        QDialog dlg(this);
        dlg.setWindowTitle("Confirm Sequence Stop");
        dlg.setModal(true);
        dlg.setMinimumWidth(440);
        auto* v = new QVBoxLayout(&dlg);
        v->setContentsMargins(36, 28, 36, 22);
        v->setSpacing(14);
        auto* title = new QLabel("Stop sequence?");
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet("font-size:22px; font-weight:700;");
        v->addWidget(title);
        auto* who = new QLabel(QString("Current user: <b>%1</b>").arg(sequencePinUsername_.toHtmlEscaped()));
        who->setTextFormat(Qt::RichText);
        who->setAlignment(Qt::AlignCenter);
        who->setStyleSheet("font-size:14px; color:#344054;");
        v->addWidget(who);
        auto* prompt = new QLabel("Enter the 4-digit PIN to stop the sequence");
        prompt->setAlignment(Qt::AlignCenter);
        prompt->setStyleSheet("font-size:13px; color:#475467;");
        v->addWidget(prompt);
        v->addSpacing(4);
        auto* pinEntry = new PinEntry(4, /*large=*/true);
        v->addWidget(pinEntry, 0, Qt::AlignCenter);
        auto* err = new QLabel(" ");
        err->setAlignment(Qt::AlignCenter);
        err->setStyleSheet("color:#d93025; font-size:12px; min-height:18px;");
        v->addWidget(err);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        auto* okBtn = buttons->button(QDialogButtonBox::Ok);
        okBtn->setText("Stop");
        okBtn->setEnabled(false);
        v->addWidget(buttons);
        connect(pinEntry, &PinEntry::changed, okBtn, [pinEntry, okBtn, err]() {
            okBtn->setEnabled(pinEntry->pin().size() == 4);
            err->setText(" ");
        });
        connect(pinEntry, &PinEntry::completed, okBtn, [okBtn]() { if (okBtn->isEnabled()) okBtn->animateClick(); });
        connect(buttons, &QDialogButtonBox::accepted, &dlg, [&, pinEntry, err]() {
            if (pinEntry->pin() != sequencePin_) { err->setText("Incorrect PIN."); pinEntry->clearPin(); return; }
            dlg.accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        pinEntry->focusFirst();
        return dlg.exec() == QDialog::Accepted;
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

    // Manual-stop PIN state. Captured from the Preflight Confirmation dialog
    // for the current run only; cleared in onFinished() so the next run forces
    // re-entry. Watchdog/force-stop escalation paths ignore these.
    bool pendingPinEnabled_ = false;
    QString pendingPinUsername_;
    QString pendingPin_;

    // Experiment sequence state. The dialog itself is created lazily when the
    // user opens Edit→Sequence; savedSequenceItems_ + savedSequenceIntervalsMin_
    // persist across opens so the queue and per-gap intervals survive closing
    // the dialog. sequenceActive_ stays true for the entire sequence and gates
    // the main page Start/Stop buttons.
    //
    // Invariant: savedSequenceIntervalsMin_.size() == max(0, savedSequenceItems_.size() - 1)
    // when sequenceActive_ is true. The interval at index k runs AFTER
    // items[k] finishes and BEFORE items[k+1] starts.
    SequenceDialog* sequenceDialog_ = nullptr;
    QList<SequenceItem> savedSequenceItems_;
    QList<int> savedSequenceIntervalsMin_;
    bool sequenceActive_ = false;
    bool sequenceStopRequested_ = false;
    int sequenceIndex_ = -1;
    int sequenceTotal_ = 0;
    QTimer sequenceIntervalTimer_;
    bool sequencePinEnabled_ = false;
    QString sequencePinUsername_;
    QString sequencePin_;
    bool sequenceStopOnError_ = false;

#if SONOCONTROL_WEB_SERVER
    // Web server lives on the GUI thread. SonoControlWindow forwards
    // worker-thread signals (already arriving via QueuedConnection by
    // default) to it via plain function calls — no DirectConnection, no
    // cross-thread mutation. The pointer is null until enable is toggled
    // (lazy construction), so the GUI startup cost is unchanged for users
    // who never enable monitoring.
    sonocontrol::WebServer* webServer_ = nullptr;
    QCheckBox* chkWebServer_ = nullptr;
    QSpinBox* spnWebPort_ = nullptr;
    QSpinBox* spnWebInterval_ = nullptr;
    QCheckBox* chkWebLan_ = nullptr;
    QLabel* lblWebStatus_ = nullptr;
#endif

    QPushButton *btnStart_{}, *btnStop_{}, *btnTestTemp_{};
    QCheckBox *chkSimTemp_{}, *chkSimUs_{}, *chkPid_{}, *chkTempMonitor_{}, *chkTempFallback_{}, *chkTempRequired_{}, *chkPidAmp_{}, *chkPidDuration_{}, *chkPidInterval_{}, *chkPidDuty_{}, *chkCycling_{}, *chkConsole_{};
    QLabel *lblCom3Status_{}, *lblUsStatus_{}, *lblTempStatus_{}, *lblPortInfo_{}, *lblTempRequirement_{}, *lblConfigStatus_{};
    QLabel* lblLanStatus_ = nullptr;      // Hyus: LAN device-detected indicator
    QDoubleSpinBox *spnAmp_{}, *spnCfreq_{}, *spnPrf_{}, *spnDuty_{}, *spnDuration_{}, *spnInterval_{}, *spnSampleRate_{}, *spnTotalDur_{}, *spnTargetHoldMin_{}, *spnTargetTol_{}, *spnCutoff_{}, *spnSetpoint_{}, *spnKp_{}, *spnKi_{}, *spnKd_{}, *spnTau_{}, *spnHorizon_{}, *spnRateWindow_{}, *spnHeatS_{}, *spnCoolS_{}, *spnCoolHoldTemp_{}, *spnTempMin_{}, *spnTempMax_{}, *spnTempRate_{};
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

    // --- New device (Hyus) support. Independent of the FPGA widgets above. ---
    sonocontrol::DeviceKind deviceKind_ = sonocontrol::DeviceKind::SonoControlFpga;
    QTabWidget* tabs_ = nullptr;          // kept so PID/CYCLE tabs can be disabled per-device
    QGroupBox* grpUsFpga_ = nullptr;      // CONNECT: serial COM + UDP (SonoControl FPGA)
    QGroupBox* grpUsHyus_ = nullptr;      // CONNECT: LAN device dropdown (Hyus)
    QComboBox* cmbHyusDevice_ = nullptr;  // detected Hyus devices (by IP)
    QLabel* lblHyusStatus_ = nullptr;
    QStringList hyusKnownDevices_;        // for detecting newly-connected devices
    std::unique_ptr<sonocontrol::hyus::HyusDiscovery> hyusDiscovery_;
    QTimer hyusScanTimer_;                // polls hyusDiscovery_ to refresh the dropdown
    QTimer hyusStopTimer_;                // single-shot end-of-run for total-duration mode
    // Hyus PARAMS-tab inputs (created alongside the FPGA inputs; only one set is
    // visible at a time so buildConfig()/applyConfigToUi() stay null-safe).
    QDoubleSpinBox* spnHyusFreqMhz_ = nullptr;
    QDoubleSpinBox* spnHyusAmpPct_ = nullptr;
    QDoubleSpinBox* spnHyusPulseLenUs_ = nullptr;
    QDoubleSpinBox* spnHyusPulsePeriodUs_ = nullptr;
    QDoubleSpinBox* spnHyusSeqLenMs_ = nullptr;
    QDoubleSpinBox* spnHyusSeqPeriodMs_ = nullptr;
    PulseSequencePlot* seqPlot_ = nullptr;  // pulse-sequence schematic (both devices)
    QRadioButton *rbHyusPidAmp_ = nullptr, *rbHyusPidPulse_ = nullptr, *rbHyusPidSeq_ = nullptr;
    QThread* hyusWorkerThread_ = nullptr;
    class HyusRunWorker* hyusWorker_ = nullptr;
};

} // namespace

int main(int argc, char** argv) {
    qRegisterMetaType<sonocontrol::ActiveParams>("sonocontrol::ActiveParams");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");
    app.setApplicationName("SonoControl");
    app.setApplicationVersion(SONOCONTROL_VERSION_STR);
    // Device selection loop: pick a device, run a window, and if the user
    // chooses Edit -> Switch Device, return here to pick again. Each window is
    // scoped so its discovery socket (TCP 8192) is released before the next one
    // binds. The chosen kind determines the window's tabs and connection fields.
    for (;;) {
        g_switchDeviceRequested = false;
        const auto deviceKind = sonocontrol::gui::selectDevice();
        if (!deviceKind) break;  // user cancelled the picker
        {
            SonoControlWindow w(*deviceKind);
            w.show();
            app.exec();
        }  // window destroyed here, releasing its device connection
        if (!g_switchDeviceRequested) break;
    }
    return 0;
}

#include "main_gui.moc"
