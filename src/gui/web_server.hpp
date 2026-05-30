#pragma once

// Optional in-app monitoring web server. Lives entirely on the GUI thread,
// owns a QTcpServer, and serves a precomputed HTML page + JSON blob that
// summarize the most recent temperature data + run state. Compiled only when
// the GUI target is built against Qt's Network module — main_gui.cpp guards
// every reference to this header behind `#if SONOCONTROL_WEB_SERVER`.

#include <QtCore/QByteArray>
#include <QtCore/QDateTime>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtNetwork/QHostAddress>

#include <cstdint>
#include <deque>

class QTcpServer;
class QTcpSocket;
class QTimer;

namespace sonocontrol {

class WebServer final : public QObject {
    Q_OBJECT
public:
    enum class State { Idle, Running, Completed, Stopped, Error };

    explicit WebServer(QObject* parent = nullptr);
    ~WebServer() override;

    // Configuration — none of these touch the socket until applyAndRestart()
    // is called. Cheap setters mean the GUI can wire valueChanged slots
    // without rebinding on every keystroke.
    void setPort(uint16_t port);
    void setSnapshotIntervalSeconds(int seconds);
    void setAllowLan(bool allow);

    // Start / stop the listening socket. start() rebinds if already running.
    // Returns true on success; bindError() carries the human-readable reason
    // on failure. Both methods rebuild the snapshot immediately so a client
    // that hits the page seconds after enable doesn't see empty data.
    bool start();
    void stop();
    bool isRunning() const;
    QString bindError() const { return last_bind_error_; }

    // Status strings shown by the GUI. listenUrls() returns localhost +
    // every non-loopback IPv4 address bound, so the operator can see the
    // actual LAN URL when "Allow LAN access" is on.
    QStringList listenUrls() const;

    // Run lifecycle hooks. Called by SonoControlWindow on the GUI thread
    // (i.e. plain function calls — there are no cross-thread signals into
    // this object). Each transition triggers an immediate snapshot rebuild
    // so the page reflects state changes without waiting for the next tick.
    void onRunStarted(const QString& sessionId);
    void onRunFinished(int exitCode);  // 0=clean, 1=error, 2=cutoff, 3=manual, 4=watchdog comms-stall
    void onSampleAvailable(double time_s, double t1, double t2, double avg);
    void onTimesUpdated(double elapsed_s, double remaining_s);
    // Replace the session id once the logger publishes the real one (the
    // GUI cannot know it at run-start because logger::start_session runs on
    // the worker thread). No-op if the id is unchanged.
    void updateSessionId(const QString& id);

signals:
    void statusChanged();

private slots:
    void onNewConnection();
    void onSnapshotTick();

private:
    void rebuildSnapshot();
    QByteArray buildJson() const;
    QByteArray buildHtml(const QByteArray& json) const;
    QHostAddress chooseBindAddress() const;
    QString stateString() const;
    void updateTimerInterval();
    bool firstSampleSinceIdle() const { return first_sample_since_idle_; }

    QTcpServer* server_ = nullptr;
    QTimer* snapshot_timer_ = nullptr;

    // Configured values; only applied when start() / setSnapshotInterval are
    // called.
    uint16_t port_ = 50896;
    int interval_s_ = 900;
    bool allow_lan_ = false;
    QString last_bind_error_;

    // Live data — appended on every sample, kept bounded so memory doesn't
    // grow on multi-day runs. ~500 samples × 32 bytes ≈ 16 KB.
    struct Sample { double t_s; double t1; double t2; double avg; };
    std::deque<Sample> buffer_;
    static constexpr size_t kMaxBufferSamples = 500;

    // State for the published snapshot.
    State state_ = State::Idle;
    QString session_id_;
    QDateTime snapshot_time_;
    QDateTime last_sample_time_;
    double elapsed_s_ = 0.0;
    double remaining_s_ = 0.0;
    bool first_sample_since_idle_ = true;

    // Precomputed response bodies. Rebuilt on every event-triggered or
    // periodic snapshot refresh; HTTP handlers just write whichever the
    // request asks for. QByteArray is COW so per-connection copies are O(1).
    QByteArray cached_html_;
    QByteArray cached_json_;

    static constexpr int kClientHeaderCapBytes = 8 * 1024;
    static constexpr int kClientReadTimeoutMs = 5000;
    friend class WebServerClient;
};

}  // namespace sonocontrol
