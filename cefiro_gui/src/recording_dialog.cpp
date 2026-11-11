#include "cefiro_gui/recording_dialog.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QList>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTextEdit>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace cefiro::gui {
namespace {
QDoubleSpinBox* makeDouble(QWidget* parent, double lo, double hi,
                           double initial, int decimals = 2) {
    auto* widget = new QDoubleSpinBox(parent);
    widget->setRange(lo, hi);
    widget->setDecimals(decimals);
    widget->setValue(initial);
    widget->setKeyboardTracking(false);
    return widget;
}
QString formatTime(double seconds, int unit) {
    switch (unit) {
        case 0: return QString::number(seconds * 1000.0, 'f', 1) + " ms";
        case 2: return QString::number(seconds / 60.0, 'f', 3) + " min";
        default: return QString::number(seconds, 'f', 3) + " s";
    }
}
}

RecordingDialog::RecordingDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("CEFIRO / EXPERIMENT DATA ACQUISITION");
    setModal(false);
    setMinimumSize(500, 480);
    resize(900, 790);
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(14, 12, 14, 12);
    outer->setSpacing(8);
    auto* heading = new QLabel("EXPERIMENT RECORDING", this);
    heading->setObjectName("pageTitle");
    outer->addWidget(heading);
    auto* subtitle = new QLabel("Configure capture rules, exported columns and recording progress", this);
    subtitle->setObjectName("pageSubtitle");
    outer->addWidget(subtitle);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    outer->addWidget(scroll, 1);
    auto* contents = new QWidget(scroll);
    scroll->setWidget(contents);
    auto* root = new QVBoxLayout(contents);
    root->setContentsMargins(1, 2, 1, 2);
    root->setSpacing(9);

    auto* destination = new QFrame(contents);
    destination->setObjectName("panel");
    auto* dv = new QVBoxLayout(destination);
    auto* cap = new QLabel("OUTPUT & EXPERIMENT", destination);
    cap->setObjectName("panelTitle");
    dv->addWidget(cap);
    auto* form = new QFormLayout;
    auto* folder = new QHBoxLayout;
    directory_ = new QLineEdit(destination);
    QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (documents.isEmpty()) documents = QDir::homePath();
    directory_->setText(QDir(documents).filePath("cefiro_data"));
    auto* browse = new QPushButton("Browse", destination);
    browse->setObjectName("secondary");
    folder->addWidget(directory_, 1);
    folder->addWidget(browse);
    form->addRow("Output directory", folder);
    prefix_ = new QLineEdit("cefiro_run", destination);
    form->addRow("File prefix", prefix_);
    notes_ = new QTextEdit(destination);
    notes_->setPlaceholderText("Objective, controller, gains, test conditions, operator...");
    notes_->setMaximumHeight(72);
    form->addRow("Experiment notes", notes_);
    metadata_ = new QCheckBox("Write companion YAML metadata and text performance summary", destination);
    metadata_->setChecked(true);
    form->addRow("", metadata_);
    dv->addLayout(form);
    root->addWidget(destination);
    connect(browse, &QPushButton::clicked, this, [this] {
        const QString folder_name = QFileDialog::getExistingDirectory(
            this, "Select acquisition directory", directory_->text());
        if (!folder_name.isEmpty()) directory_->setText(folder_name);
    });

    auto* capture = new QFrame(contents);
    capture->setObjectName("panel");
    auto* cv = new QVBoxLayout(capture);
    auto* capture_title = new QLabel("SMART CAPTURE", capture);
    capture_title->setObjectName("panelTitle");
    cv->addWidget(capture_title);
    auto* config_form = new QGridLayout;
    capture_mode_ = new QComboBox(capture);
    capture_mode_->addItems({"Manual / continuous", "Time window", "N waveform periods", "Threshold events"});
    interval_ = new QSpinBox(capture);
    interval_->setRange(0, 10000);
    interval_->setSuffix(" ms");
    interval_->setToolTip("0 = preserve all received IMU samples. Larger values decimate saved rows, not displayed telemetry.");
    time_unit_ = new QComboBox(capture);
    time_unit_->addItems({"Milliseconds", "Seconds", "Minutes"});
    time_unit_->setCurrentIndex(1);
    config_form->addWidget(new QLabel("Capture mode", capture), 0, 0);
    config_form->addWidget(capture_mode_, 0, 1);
    config_form->addWidget(new QLabel("Min. save interval", capture), 0, 2);
    config_form->addWidget(interval_, 0, 3);
    config_form->addWidget(new QLabel("Display time as", capture), 1, 0);
    config_form->addWidget(time_unit_, 1, 1);
    cv->addLayout(config_form);
    waveform_label_ = new QLabel("Waveform: --", capture);
    waveform_label_->setObjectName("pageSubtitle");
    cv->addWidget(waveform_label_);
    timing_panel_ = new QWidget(capture);
    auto* timing = new QGridLayout(timing_panel_);
    timing->setContentsMargins(0, 0, 0, 0);
    delay_ = makeDouble(timing_panel_, 0, 3600, 0, 2);
    duration_ = makeDouble(timing_panel_, 0.01, 86400, 10, 2);
    periods_ = new QSpinBox(timing_panel_);
    periods_->setRange(1, 100000);
    periods_->setValue(10);
    timing->addWidget(new QLabel("Start delay [s]", timing_panel_), 0, 0);
    timing->addWidget(delay_, 0, 1);
    timing->addWidget(new QLabel("Duration [s]", timing_panel_), 0, 2);
    timing->addWidget(duration_, 0, 3);
    timing->addWidget(new QLabel("Periods", timing_panel_), 1, 0);
    timing->addWidget(periods_, 1, 1);
    cv->addWidget(timing_panel_);

    peak_panel_ = new QWidget(capture);
    auto* peaks = new QGridLayout(peak_panel_);
    peaks->setContentsMargins(0, 0, 0, 0);
    peak_axis_ = new QComboBox(peak_panel_);
    peak_axis_->addItems({"Roll", "Pitch", "Yaw"});
    peak_threshold_ = makeDouble(peak_panel_, 0, 180, 10, 2);
    peak_separation_ = makeDouble(peak_panel_, 0, 3600, 0.2, 3);
    peak_target_ = new QSpinBox(peak_panel_);
    peak_target_->setRange(1, 100000);
    peak_target_->setValue(5);
    peak_absolute_ = new QCheckBox("Use absolute attitude magnitude", peak_panel_);
    peak_absolute_->setChecked(true);
    peaks->addWidget(new QLabel("Axis", peak_panel_), 0, 0);
    peaks->addWidget(peak_axis_, 0, 1);
    peaks->addWidget(new QLabel("Peak threshold [deg]", peak_panel_), 0, 2);
    peaks->addWidget(peak_threshold_, 0, 3);
    peaks->addWidget(new QLabel("Min. separation [s]", peak_panel_), 1, 0);
    peaks->addWidget(peak_separation_, 1, 1);
    peaks->addWidget(new QLabel("Stop after peaks", peak_panel_), 1, 2);
    peaks->addWidget(peak_target_, 1, 3);
    peaks->addWidget(peak_absolute_, 2, 0, 1, 4);
    cv->addWidget(peak_panel_);
    root->addWidget(capture);

    auto* columns = new QFrame(contents);
    columns->setObjectName("panel");
    auto* colv = new QVBoxLayout(columns);
    auto* col_title = new QLabel("SELECT EXPORTED COLUMNS", columns);
    col_title->setObjectName("panelTitle");
    colv->addWidget(col_title);
    auto* col_grid = new QGridLayout;
    save_attitude_ = new QCheckBox("Attitude: roll / pitch / yaw", columns);
    save_reference_ = new QCheckBox("Reference: ref_roll / ref_pitch / ref_yaw", columns);
    save_error_ = new QCheckBox("Tracking error", columns);
    save_rates_ = new QCheckBox("Angular rates p / q / r", columns);
    save_accel_ = new QCheckBox("Linear acceleration ax / ay / az", columns);
    save_mag_ = new QCheckBox("Magnetic field Bx / By / Bz", columns);
    save_baro_ = new QCheckBox("Barometer: pressure / relative altitude", columns);
    save_validity_ = new QCheckBox("Validity flags", columns);
    save_strategy_source_ = new QCheckBox("Strategy and source", columns);
    for (auto* box : {save_attitude_, save_reference_, save_error_, save_rates_, save_accel_, save_mag_, save_baro_, save_validity_, save_strategy_source_}) {
        box->setChecked(true);
    }
    QList<QCheckBox*> boxes{save_attitude_, save_reference_, save_error_, save_rates_, save_accel_,
                            save_mag_, save_baro_, save_validity_, save_strategy_source_};
    for (int i = 0; i < boxes.size(); ++i) col_grid->addWidget(boxes[i], i / 2, i % 2);
    colv->addLayout(col_grid);
    root->addWidget(columns);

    auto* status = new QFrame(contents);
    status->setObjectName("panel");
    auto* sv = new QVBoxLayout(status);
    auto* status_title = new QLabel("CAPTURE STATUS", status);
    status_title->setObjectName("panelTitle");
    sv->addWidget(status_title);
    auto* stats = new QGridLayout;
    state_label_ = new QLabel("IDLE", status);
    state_label_->setObjectName("metricValue");
    samples_label_ = new QLabel("0", status);
    duration_label_ = new QLabel("0.000 s", status);
    peaks_label_ = new QLabel("0", status);
    for (auto* metric : {samples_label_, duration_label_, peaks_label_}) metric->setObjectName("metricValue");
    stats->addWidget(new QLabel("STATE", status), 0, 0);
    stats->addWidget(state_label_, 0, 1);
    stats->addWidget(new QLabel("SAMPLES", status), 0, 2);
    stats->addWidget(samples_label_, 0, 3);
    stats->addWidget(new QLabel("ELAPSED", status), 0, 4);
    stats->addWidget(duration_label_, 0, 5);
    stats->addWidget(new QLabel("PEAKS", status), 0, 6);
    stats->addWidget(peaks_label_, 0, 7);
    sv->addLayout(stats);
    progress_bar_ = new QProgressBar(status);
    progress_bar_->setRange(0, 0);
    progress_bar_->setTextVisible(true);
    progress_bar_->setFormat("Continuous recording");
    estimate_label_ = new QLabel("Estimate: open-ended", status);
    estimate_label_->setObjectName("hint");
    sv->addWidget(progress_bar_);
    sv->addWidget(estimate_label_);
    file_label_ = new QLabel("File: --", status);
    file_label_->setWordWrap(true);
    file_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    sv->addWidget(file_label_);
    error_label_ = new QLabel(status);
    error_label_->setWordWrap(true);
    error_label_->setObjectName("errorText");
    sv->addWidget(error_label_);
    root->addWidget(status);
    root->addStretch();

    auto* buttons = new QHBoxLayout;
    start_btn_ = new QPushButton("START RECORDING", this);
    start_btn_->setObjectName("primaryAction");
    stop_btn_ = new QPushButton("STOP RECORDING", this);
    stop_btn_->setObjectName("secondary");
    stop_btn_->setEnabled(false);
    auto* close = new QPushButton("CLOSE", this);
    close->setObjectName("secondary");
    buttons->addWidget(start_btn_);
    buttons->addWidget(stop_btn_);
    buttons->addStretch();
    buttons->addWidget(close);
    outer->addLayout(buttons);
    connect(close, &QPushButton::clicked, this, &QDialog::hide);
    connect(capture_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { updateCaptureVisibility(); });
    updateCaptureVisibility();
}

void RecordingDialog::updateCaptureVisibility() {
    const int mode = capture_mode_->currentIndex();
    timing_panel_->setVisible(mode == 1 || mode == 2 || mode == 3);
    duration_->setEnabled(mode == 1);
    periods_->setEnabled(mode == 2);
    peak_panel_->setVisible(mode == 3);
}

RecordingConfig RecordingDialog::configuration() const {
    RecordingConfig c;
    c.directory = directory_->text().trimmed();
    c.prefix = prefix_->text().trimmed();
    c.notes = notes_->toPlainText();
    c.metadata = metadata_->isChecked();
    c.mode = capture_mode_->currentIndex();
    c.save_interval_ms = interval_->value();
    c.start_delay_s = delay_->value();
    c.duration_s = duration_->value();
    c.periods = periods_->value();
    c.peak_axis = peak_axis_->currentIndex();
    c.peak_threshold_deg = peak_threshold_->value();
    c.peak_separation_s = peak_separation_->value();
    c.peak_count = peak_target_->value();
    c.absolute_peaks = peak_absolute_->isChecked();
    c.time_unit = time_unit_->currentIndex();
    c.save_attitude = save_attitude_->isChecked();
    c.save_reference = save_reference_->isChecked();
    c.save_error = save_error_->isChecked();
    c.save_rates = save_rates_->isChecked();
    c.save_accel = save_accel_->isChecked();
    c.save_mag = save_mag_->isChecked();
    c.save_baro = save_baro_->isChecked();
    c.save_validity = save_validity_->isChecked();
    c.save_strategy_source = save_strategy_source_->isChecked();
    return c;
}
void RecordingDialog::setRecordingState(bool active) {
    start_btn_->setEnabled(!active);
    stop_btn_->setEnabled(active);
    for (QWidget* field : QList<QWidget*>{directory_, prefix_, notes_,
                           metadata_, capture_mode_, interval_, delay_, duration_,
                           periods_, peak_axis_, peak_threshold_, peak_separation_,
                           peak_target_, peak_absolute_, time_unit_, save_attitude_,
                           save_reference_, save_error_, save_rates_, save_accel_,
                           save_mag_, save_baro_, save_validity_, save_strategy_source_}) {
        field->setEnabled(!active);
    }
    if (!active) updateCaptureVisibility();
}
void RecordingDialog::updateStatus(const QString& state, quint64 samples,
                                   double seconds, int events, const QString& filename,
                                   double progress_fraction,
                                   const QString& estimate_text) {
    state_label_->setText(state);
    samples_label_->setText(QString::number(samples));
    duration_label_->setText(formatTime(seconds, time_unit_ ? time_unit_->currentIndex() : 1));
    peaks_label_->setText(QString::number(events));
    file_label_->setText(filename.isEmpty() ? "File: --" : "File: " + filename);
    if (progress_bar_) {
        if (progress_fraction < 0.0 || !std::isfinite(progress_fraction)) {
            progress_bar_->setRange(0, 0);
            progress_bar_->setFormat("Continuous recording");
        } else {
            const int value = int(std::clamp(progress_fraction, 0.0, 1.0) * 1000.0);
            progress_bar_->setRange(0, 1000);
            progress_bar_->setValue(value);
            progress_bar_->setFormat(QString::number(progress_fraction * 100.0, 'f', 1) + "% complete");
        }
    }
    if (estimate_label_) estimate_label_->setText(estimate_text.isEmpty() ? "Estimate: open-ended" : estimate_text);
}
void RecordingDialog::showError(const QString& message) { error_label_->setText(message); }
void RecordingDialog::setWaveformInformation(const QString& text) { waveform_label_->setText(text); }

}  // namespace cefiro::gui
