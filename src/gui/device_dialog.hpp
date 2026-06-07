#pragma once

// Startup device-selection popup.
//
// Kept deliberately self-contained and header-only: it is a plain free function
// built from stock Qt widgets (no Q_OBJECT, so AUTOMOC need not process it), and
// it has no knowledge of SonoControlWindow. main() calls it once before the main
// window is constructed; the chosen DeviceKind then drives which connection
// fields and tabs the window exposes. Adding a future device means adding one
// entry to kDeviceOptions below — nothing else here changes.

#include "config.hpp"

#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtCore/QString>

#include <optional>
#include <vector>

namespace sonocontrol::gui {

struct DeviceOption {
    DeviceKind kind;
    QString title;
    QString connection;  // how it connects (shown under the title)
    QString notes;       // capabilities / caveats
};

// Single source of truth for the picker. Each device documents its own
// connection method so the prompt genuinely differs per device.
inline std::vector<DeviceOption> deviceOptions() {
    return {
        {DeviceKind::SonoControlFpga,
         QStringLiteral("Zhuhai (COM + UDP)"),
         QStringLiteral("Serial COM port + UDP waveform stream.\n"
                        "Set the COM3 port and UDP host/port in the CONNECT tab."),
         QStringLiteral("Full feature set: PID temperature control and "
                        "heating/cooling cycles.")},
        {DeviceKind::Hyus,
         QStringLiteral("Hyus (LAN)"),
         QStringLiteral("Ethernet (RJ45), auto-detected on the local network. The PC "
                        "listens on TCP 8192 and the device dials in.\n"
                        "Pick the detected device from the dropdown in the CONNECT tab."),
         QStringLiteral("Internal trigger. PID temperature control, heating/cooling "
                        "cycles, and safety cutoff supported.")},
    };
}

// Shows the modal picker. Returns the chosen device, or nullopt if the user
// closed/cancelled the dialog (caller should then exit without launching).
inline std::optional<DeviceKind> selectDevice(QWidget* parent = nullptr) {
    const auto options = deviceOptions();

    QDialog dlg(parent);
    dlg.setWindowTitle(QStringLiteral("Select Device"));
    dlg.setModal(true);
    dlg.setMinimumWidth(460);

    auto* v = new QVBoxLayout(&dlg);
    v->setSpacing(12);
    v->setContentsMargins(20, 20, 20, 16);

    auto* heading = new QLabel(QStringLiteral("Which device are you connecting to?"));
    heading->setStyleSheet(QStringLiteral("font-size:15px; font-weight:600;"));
    v->addWidget(heading);

    auto* group = new QButtonGroup(&dlg);
    for (int i = 0; i < static_cast<int>(options.size()); ++i) {
        const auto& opt = options[static_cast<size_t>(i)];
        auto* card = new QFrame;
        card->setFrameShape(QFrame::StyledPanel);
        auto* cv = new QVBoxLayout(card);
        cv->setContentsMargins(12, 10, 12, 10);
        cv->setSpacing(4);

        auto* rb = new QRadioButton(opt.title);
        rb->setStyleSheet(QStringLiteral("font-size:14px; font-weight:600;"));
        group->addButton(rb, i);
        cv->addWidget(rb);

        auto* conn = new QLabel(opt.connection);
        conn->setWordWrap(true);
        conn->setStyleSheet(QStringLiteral("color:#475467; font-size:12px;"));
        cv->addWidget(conn);

        auto* notes = new QLabel(opt.notes);
        notes->setWordWrap(true);
        notes->setStyleSheet(QStringLiteral("color:#667085; font-size:12px;"));
        cv->addWidget(notes);

        v->addWidget(card);
        if (i == 0) rb->setChecked(true);
    }

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("Continue"));
    v->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return std::nullopt;
    const int idx = group->checkedId();
    if (idx < 0 || idx >= static_cast<int>(options.size())) return std::nullopt;
    return options[static_cast<size_t>(idx)].kind;
}

} // namespace sonocontrol::gui
