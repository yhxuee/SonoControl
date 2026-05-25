#include "web_server.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonValue>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtNetwork/QAbstractSocket>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

#include <algorithm>
#include <cmath>

namespace sonocontrol {

// Per-connection helper. One of these is constructed for each accepted
// QTcpSocket; it reads the request header (capped + timed out), dispatches to
// the parent WebServer's cached payloads, writes the response, and deletes
// itself when the socket disconnects.
//
// Declared in the same namespace (not anonymous) so the `friend class
// WebServerClient` declaration in WebServer matches; MinGW correctly rejects
// the friend-with-anonymous-namespace mismatch.
class WebServerClient final : public QObject {
    Q_OBJECT
public:
    WebServerClient(QTcpSocket* socket, WebServer* server)
        : QObject(socket), socket_(socket), server_(server) {
        timeout_ = new QTimer(this);
        timeout_->setSingleShot(true);
        timeout_->setInterval(WebServer::kClientReadTimeoutMs);
        connect(timeout_, &QTimer::timeout, this, [this]{
            // Slow / silent client. Abort the connection so we don't tie up a
            // worker waiting for a partial header forever.
            socket_->abort();
        });
        connect(socket_, &QTcpSocket::readyRead, this, &WebServerClient::onReadyRead);
        connect(socket_, &QTcpSocket::disconnected, this, &QObject::deleteLater);
        timeout_->start();
    }

private slots:
    void onReadyRead() {
        buf_.append(socket_->readAll());
        if (buf_.size() > WebServer::kClientHeaderCapBytes) {
            writeStatus(413, "Payload Too Large", "Request header exceeded server cap.");
            socket_->disconnectFromHost();
            return;
        }
        const int sep = buf_.indexOf("\r\n\r\n");
        if (sep < 0) return;  // wait for the rest of the header
        timeout_->stop();

        // Parse the request line. Only the first line matters; we don't read
        // any body. Strict GET only; everything else gets 405.
        const int eol = buf_.indexOf("\r\n");
        const QByteArray req_line = buf_.left(eol);
        const QList<QByteArray> parts = req_line.split(' ');
        if (parts.size() < 2) {
            writeStatus(400, "Bad Request", "Malformed request line.");
            socket_->disconnectFromHost();
            return;
        }
        const QByteArray method = parts[0];
        QByteArray path = parts[1];
        if (method != "GET") {
            writeStatus(405, "Method Not Allowed", "Only GET is supported.");
            socket_->disconnectFromHost();
            return;
        }
        // Strip the query string so /data.json?cachebust=123 still routes.
        // Browsers and tools often append a cache-buster; treating that as
        // a 404 would be user-hostile.
        const int q = path.indexOf('?');
        if (q >= 0) path = path.left(q);

        if (path == "/") {
            writeBody(200, "OK", server_->cached_html_, "text/html; charset=utf-8");
        } else if (path == "/data.json") {
            writeBody(200, "OK", server_->cached_json_, "application/json; charset=utf-8");
        } else {
            writeStatus(404, "Not Found", "No such resource.");
        }
        socket_->disconnectFromHost();
    }

private:
    void writeStatus(int code, const char* reason, const QString& body_text) {
        writeBody(code, reason, body_text.toUtf8(), "text/plain; charset=utf-8");
    }

    void writeBody(int code, const char* reason, const QByteArray& body, const char* content_type) {
        QByteArray hdr;
        hdr.append("HTTP/1.1 ");
        hdr.append(QByteArray::number(code));
        hdr.append(' ');
        hdr.append(reason);
        hdr.append("\r\n");
        hdr.append("Content-Type: ");
        hdr.append(content_type);
        hdr.append("\r\n");
        hdr.append("Content-Length: ");
        hdr.append(QByteArray::number(body.size()));
        hdr.append("\r\n");
        hdr.append("Cache-Control: no-store\r\n");
        hdr.append("Connection: close\r\n");
        hdr.append("\r\n");
        socket_->write(hdr);
        socket_->write(body);
    }

    QTcpSocket* socket_;
    WebServer* server_;
    QTimer* timeout_;
    QByteArray buf_;
};

namespace {
QString fmt_time_iso(const QDateTime& dt) {
    if (!dt.isValid()) return QString();
    return dt.toString("yyyy-MM-dd HH:mm:ss");
}
}  // namespace

WebServer::WebServer(QObject* parent) : QObject(parent) {
    server_ = new QTcpServer(this);
    connect(server_, &QTcpServer::newConnection, this, &WebServer::onNewConnection);

    snapshot_timer_ = new QTimer(this);
    snapshot_timer_->setSingleShot(false);
    connect(snapshot_timer_, &QTimer::timeout, this, &WebServer::onSnapshotTick);
    updateTimerInterval();

    // Build an initial idle snapshot so the page is non-empty even if a
    // client connects before any sample arrives.
    rebuildSnapshot();
}

WebServer::~WebServer() {
    stop();
}

void WebServer::setPort(uint16_t port) { port_ = port; }

void WebServer::setSnapshotIntervalSeconds(int seconds) {
    interval_s_ = std::clamp(seconds, 5, 3600);
    updateTimerInterval();
}

void WebServer::setAllowLan(bool allow) { allow_lan_ = allow; }

void WebServer::updateTimerInterval() {
    if (!snapshot_timer_) return;
    // Convert seconds to ms; clamp to a safe int range.
    const qint64 ms = static_cast<qint64>(interval_s_) * 1000;
    snapshot_timer_->setInterval(static_cast<int>(std::min<qint64>(ms, std::numeric_limits<int>::max())));
}

QHostAddress WebServer::chooseBindAddress() const {
    return allow_lan_ ? QHostAddress(QHostAddress::Any) : QHostAddress(QHostAddress::LocalHost);
}

bool WebServer::start() {
    last_bind_error_.clear();
    if (server_->isListening()) {
        server_->close();
    }
    const QHostAddress addr = chooseBindAddress();
    if (!server_->listen(addr, port_)) {
        last_bind_error_ = server_->errorString();
        emit statusChanged();
        return false;
    }
    snapshot_timer_->start();
    // Critical: rebuild now so the first visitor sees current state, not the
    // stale snapshot from before the toggle.
    rebuildSnapshot();
    emit statusChanged();
    return true;
}

void WebServer::stop() {
    if (server_->isListening()) server_->close();
    snapshot_timer_->stop();
    emit statusChanged();
}

bool WebServer::isRunning() const { return server_ && server_->isListening(); }

QStringList WebServer::listenUrls() const {
    QStringList out;
    if (!server_ || !server_->isListening()) return out;
    const quint16 port = server_->serverPort();
    out << QString("http://127.0.0.1:%1/").arg(port);
    if (!allow_lan_) return out;
    // Iterate non-loopback IPv4 addresses on up interfaces. This is what the
    // operator actually types into a phone or another LAN machine, and is
    // strictly more informative than just "all interfaces, port X".
    const auto ifaces = QNetworkInterface::allInterfaces();
    for (const auto& iface : ifaces) {
        const auto flags = iface.flags();
        if (!(flags & QNetworkInterface::IsUp)) continue;
        if (!(flags & QNetworkInterface::IsRunning)) continue;
        if (flags & QNetworkInterface::IsLoopBack) continue;
        for (const auto& entry : iface.addressEntries()) {
            const QHostAddress ip = entry.ip();
            if (ip.protocol() != QAbstractSocket::IPv4Protocol) continue;
            if (ip.isLoopback()) continue;
            const QString url = QString("http://%1:%2/").arg(ip.toString()).arg(port);
            if (!out.contains(url)) out << url;
        }
    }
    return out;
}

void WebServer::onNewConnection() {
    while (server_ && server_->hasPendingConnections()) {
        QTcpSocket* sock = server_->nextPendingConnection();
        if (!sock) break;
        // The client object is parented to the socket, so deleteLater on the
        // socket cleans up both.
        new WebServerClient(sock, this);
    }
}

void WebServer::onSnapshotTick() { rebuildSnapshot(); }

void WebServer::onRunStarted(const QString& sessionId) {
    buffer_.clear();
    state_ = State::Running;
    session_id_ = sessionId;
    elapsed_s_ = 0.0;
    remaining_s_ = 0.0;
    last_sample_time_ = QDateTime();
    first_sample_since_idle_ = true;
    // Rebuild now so a tab opened just after the operator clicks START shows
    // "Running" rather than stale idle data.
    rebuildSnapshot();
}

void WebServer::onRunFinished(int exitCode) {
    switch (exitCode) {
        case 0:  state_ = State::Completed; break;
        case 2:  state_ = State::Error;     break;  // safety cutoff treated as error for monitoring
        case 3:  state_ = State::Stopped;   break;
        default: state_ = State::Error;     break;
    }
    first_sample_since_idle_ = true;  // a future restart re-arms the immediate rebuild
    rebuildSnapshot();
}

void WebServer::onSampleAvailable(double time_s, double t1, double t2, double avg) {
    Sample s{time_s, t1, t2, avg};
    buffer_.push_back(s);
    while (buffer_.size() > kMaxBufferSamples) buffer_.pop_front();
    last_sample_time_ = QDateTime::currentDateTime();
    if (first_sample_since_idle_) {
        first_sample_since_idle_ = false;
        // First data point after a fresh start — rebuild immediately so the
        // page transitions from "running, no samples yet" to a real chart
        // without waiting for the next tick.
        rebuildSnapshot();
    }
}

void WebServer::updateSessionId(const QString& id) {
    if (id == session_id_) return;
    session_id_ = id;
    // Rebuild so the next visitor sees the canonical id that matches the
    // CSV/JSON filename. This fires at most once per run.
    rebuildSnapshot();
}

void WebServer::onTimesUpdated(double elapsed_s, double remaining_s) {
    elapsed_s_ = elapsed_s;
    remaining_s_ = remaining_s;
    // No rebuild here: time updates arrive frequently and the displayed
    // timing values are recomputed at every snapshot tick anyway. Triggering
    // a rebuild here would defeat the cadence the user explicitly chose.
}

QString WebServer::stateString() const {
    switch (state_) {
        case State::Running:   return "running";
        case State::Completed: return "completed";
        case State::Stopped:   return "stopped";
        case State::Error:     return "error";
        case State::Idle:      break;
    }
    return "idle";
}

QByteArray WebServer::buildJson() const {
    QJsonObject root;
    root["state"] = stateString();
    root["session_id"] = session_id_;
    root["snapshot_time"] = fmt_time_iso(snapshot_time_);
    root["last_sample_time"] = fmt_time_iso(last_sample_time_);
    root["elapsed_s"] = elapsed_s_;
    root["remaining_s"] = remaining_s_;
    root["sample_count"] = static_cast<int>(buffer_.size());
    root["snapshot_interval_s"] = interval_s_;

    QJsonArray samples;  // QJsonArray has no reserve(); growth is amortized O(1) per append
    for (const auto& s : buffer_) {
        QJsonArray row;
        row.append(s.t_s);
        // NaN -> null. The plot in the page treats null as a gap.
        row.append(std::isnan(s.t1) ? QJsonValue() : QJsonValue(s.t1));
        row.append(std::isnan(s.t2) ? QJsonValue() : QJsonValue(s.t2));
        row.append(std::isnan(s.avg) ? QJsonValue() : QJsonValue(s.avg));
        samples.append(row);
    }
    root["samples"] = samples;

    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray WebServer::buildHtml(const QByteArray& json) const {
    // Single self-contained HTML page. Inline CSS + JS only, no external
    // network requests — the experimental platform may have unreliable
    // internet via its single wireless NIC, and we explicitly want the
    // monitoring page to work on a pure-LAN setup. The chart is drawn with
    // the Canvas API; total page size <4 KB before the JSON blob.
    QByteArray html;
    html.reserve(4096 + json.size());
    html.append("<!DOCTYPE html>\n<html lang=\"en\"><head>\n");
    html.append("<meta charset=\"utf-8\">\n");
    html.append("<title>SonoControl Monitor</title>\n");
    // The page auto-refreshes at the same cadence the server rebuilds the
    // snapshot, so a left-open tab catches up without polling.
    html.append("<meta http-equiv=\"refresh\" content=\"");
    html.append(QByteArray::number(std::max(5, interval_s_)));
    html.append("\">\n");
    html.append("<style>\n");
    html.append(
        "body{font-family:'Segoe UI',Arial,sans-serif;margin:24px;background:#f3f5f8;color:#1a1a2e;}"
        "h1{margin:0 0 12px 0;font-size:22px;display:flex;align-items:center;gap:12px;}"
        ".badge{padding:4px 10px;border-radius:12px;font-size:11px;font-weight:700;letter-spacing:0.6px;text-transform:uppercase;}"
        ".badge.running{background:#e6f4ea;color:#0a9e8a;}"
        ".badge.idle{background:#f3f5f8;color:#667085;border:1px solid #e4e7ec;}"
        ".badge.completed{background:#e8f0fe;color:#1a73e8;}"
        ".badge.stopped{background:#fff4e5;color:#b54708;}"
        ".badge.error{background:#fce8e6;color:#b42318;}"
        ".card{background:#fff;border-radius:8px;padding:16px 20px;margin-bottom:12px;box-shadow:0 1px 4px rgba(0,0,0,.06);}"
        ".stats{display:flex;gap:36px;flex-wrap:wrap;}"
        ".stat .label{font-size:11px;color:#98a2b3;text-transform:uppercase;letter-spacing:.6px;}"
        ".stat .value{font-size:22px;font-weight:700;font-family:Consolas,monospace;color:#0a9e8a;margin-top:4px;}"
        ".stat .value.muted{color:#98a2b3;}"
        "canvas{display:block;width:100%;max-width:1100px;height:420px;}"
        ".legend{display:flex;gap:20px;margin-top:10px;font-size:12px;color:#475467;}"
        ".swatch{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:6px;}"
        ".footer{font-size:11px;color:#98a2b3;margin-top:16px;}"
    );
    html.append("\n</style>\n</head><body>\n");
    html.append("<h1>SonoControl Monitor <span id=\"badge\" class=\"badge\"></span></h1>\n");
    html.append("<div class=\"card\"><div class=\"stats\">"
                "<div class=\"stat\"><div class=\"label\">Elapsed</div><div class=\"value\" id=\"elapsed\">--:--:--</div></div>"
                "<div class=\"stat\"><div class=\"label\">Remaining</div><div class=\"value\" id=\"remaining\">--:--:--</div></div>"
                "<div class=\"stat\"><div class=\"label\">Last Sample</div><div class=\"value\" id=\"lastSample\">--</div></div>"
                "<div class=\"stat\"><div class=\"label\">Snapshot</div><div class=\"value\" id=\"snapshot\">--</div></div>"
                "<div class=\"stat\"><div class=\"label\">Samples</div><div class=\"value\" id=\"count\">0</div></div>"
                "</div></div>\n");
    html.append("<div class=\"card\"><canvas id=\"chart\" width=\"1100\" height=\"420\"></canvas>"
                "<div class=\"legend\">"
                "<span><span class=\"swatch\" style=\"background:#1a73e8\"></span>T1</span>"
                "<span><span class=\"swatch\" style=\"background:#ea4335\"></span>T2</span>"
                "<span><span class=\"swatch\" style=\"background:#0a9e8a\"></span>Avg</span>"
                "</div></div>\n");
    html.append("<div class=\"footer\">Page rebuilt by the server every <span id=\"interval\"></span> s. "
                "Browser refreshes itself at the same cadence; force-reload for the latest.</div>\n");
    html.append("<script>\n");
    html.append("const D=");
    html.append(json);
    html.append(";\n");
    html.append(
        "function fmtTime(s){if(s==null||isNaN(s)||s<0)return '--:--:--';"
        "s=Math.floor(s);const h=String(Math.floor(s/3600)).padStart(2,'0');"
        "const m=String(Math.floor((s%3600)/60)).padStart(2,'0');"
        "const sec=String(s%60).padStart(2,'0');return h+':'+m+':'+sec;}"
        "function setText(id,t,muted){const e=document.getElementById(id);e.textContent=t;"
        "if(muted)e.classList.add('muted');else e.classList.remove('muted');}"
        "function applyState(){const b=document.getElementById('badge');b.className='badge '+(D.state||'idle');"
        "b.textContent=(D.state||'idle').toUpperCase();"
        "const live=D.state==='running';"
        "setText('elapsed',fmtTime(D.elapsed_s),!live);"
        "setText('remaining',fmtTime(D.remaining_s),!live);"
        "setText('lastSample',D.last_sample_time||'--',!D.last_sample_time);"
        "setText('snapshot',D.snapshot_time||'--',false);"
        "setText('count',String(D.sample_count||0),(D.sample_count||0)===0);"
        "document.getElementById('interval').textContent=D.snapshot_interval_s;}"
        "function drawChart(){const cv=document.getElementById('chart');const ctx=cv.getContext('2d');"
        "const w=cv.width,h=cv.height;ctx.clearRect(0,0,w,h);"
        "const r={l:64,t:20,r:20,b:44},cw=w-r.l-r.r,ch=h-r.t-r.b;"
        "ctx.fillStyle='#fff';ctx.fillRect(r.l,r.t,cw,ch);"
        "ctx.strokeStyle='#d0d5dd';ctx.lineWidth=1;ctx.strokeRect(r.l,r.t,cw,ch);"
        "ctx.strokeStyle='#e8eaee';ctx.beginPath();"
        "for(let i=1;i<5;i++){const y=r.t+ch*i/5;ctx.moveTo(r.l,y);ctx.lineTo(r.l+cw,y);}ctx.stroke();"
        "const samples=D.samples||[];"
        "if(samples.length<2){ctx.fillStyle='#98a2b3';ctx.font='14px sans-serif';ctx.textAlign='center';"
        "ctx.fillText(samples.length===0?'No samples in cache.':'Only one sample so far.',w/2,h/2);return;}"
        "const xs=samples.map(s=>s[0]);const xmin=xs[0];const xmax=xs[xs.length-1];const xspan=Math.max(1,xmax-xmin);"
        "const vs=[];for(const s of samples){for(let i=1;i<=3;i++)if(s[i]!=null&&!isNaN(s[i]))vs.push(s[i]);}"
        "let ymin=Math.min(...vs),ymax=Math.max(...vs);"
        "if(!isFinite(ymin)){ymin=35;ymax=45;}"
        "ymin=Math.floor(ymin-1);ymax=Math.ceil(ymax+1);"
        "if(ymax-ymin<4){ymin-=1;ymax+=3;}"
        "const mx=x=>r.l+(x-xmin)/xspan*cw;const my=y=>r.t+ch-(y-ymin)/(ymax-ymin)*ch;"
        "ctx.fillStyle='#475467';ctx.font='11px Consolas,monospace';ctx.textAlign='right';"
        "for(let i=0;i<=5;i++){const v=ymin+(ymax-ymin)*i/5;const y=r.t+ch-i/5*ch;ctx.fillText(v.toFixed(1),r.l-6,y+4);}"
        "ctx.textAlign='center';"
        "for(let i=0;i<=4;i++){const v=xmin+xspan*i/4;const x=r.l+cw*i/4;ctx.fillText(v.toFixed(0),x,r.t+ch+18);}"
        "ctx.fillText('time (s)',r.l+cw/2,r.t+ch+36);"
        "ctx.save();ctx.translate(20,r.t+ch/2);ctx.rotate(-Math.PI/2);ctx.fillText('Temperature (\\u00B0C)',0,0);ctx.restore();"
        "function line(idx,color,width){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();let started=false;"
        "for(const s of samples){const y=s[idx];if(y==null||isNaN(y)){if(started){ctx.stroke();ctx.beginPath();started=false;}continue;}"
        "const px=mx(s[0]),py=my(y);if(!started){ctx.moveTo(px,py);started=true;}else{ctx.lineTo(px,py);}}"
        "if(started)ctx.stroke();}"
        "line(1,'#1a73e8',1);line(2,'#ea4335',1);line(3,'#0a9e8a',2.5);}"
        "applyState();drawChart();"
    );
    html.append("\n</script></body></html>\n");
    return html;
}

void WebServer::rebuildSnapshot() {
    snapshot_time_ = QDateTime::currentDateTime();
    cached_json_ = buildJson();
    cached_html_ = buildHtml(cached_json_);
}

}  // namespace sonocontrol

#include "web_server.moc"
