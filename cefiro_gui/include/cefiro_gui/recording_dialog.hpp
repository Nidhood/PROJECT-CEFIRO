#pragma once
#include <QDialog>
#include <QString>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTextEdit;

namespace cefiro::gui {

struct RecordingConfig {
    QString directory;
    QString prefix{"cefiro_run"};
    QString notes;
    bool metadata{true};
    int mode{0};  // Continuous, duration, waveform periods, threshold events.
    int save_interval_ms{0};
    double start_delay_s{0.0};
    double duration_s{10.0};
    int periods{10};
    int peak_axis{0};
    double peak_threshold_deg{10.0};
    double peak_separation_s{0.2};
    int peak_count{5};
    bool absolute_peaks{true};
    int time_unit{1};  // 0 = ms, 1 = s, 2 = min
    bool save_attitude{true};
    bool save_reference{true};
    bool save_error{true};
    bool save_rates{true};
    bool save_accel{true};
    bool save_mag{true};
    bool save_baro{true};
    bool save_validity{true};
    bool save_strategy_source{true};
};

class RecordingDialog final : public QDialog {
public:
    explicit RecordingDialog(QWidget* parent = nullptr);
    RecordingConfig configuration() const;
    QPushButton* startButton() const noexcept { return start_btn_; }
    QPushButton* stopButton() const noexcept { return stop_btn_; }
    void setRecordingState(bool active);
    void updateStatus(const QString& state, quint64 samples, double seconds,
                      int events, const QString& filename,
                      double progress_fraction = -1.0,
                      const QString& estimate_text = QString());
    void showError(const QString& message);
    void setWaveformInformation(const QString& text);
private:
    void updateCaptureVisibility();
    QLineEdit* directory_{nullptr};
    QComboBox* time_unit_{nullptr};
    QLineEdit* prefix_{nullptr};
    QTextEdit* notes_{nullptr};
    QCheckBox* metadata_{nullptr};
    QComboBox* capture_mode_{nullptr};
    QSpinBox* interval_{nullptr};
    QDoubleSpinBox* delay_{nullptr};
    QDoubleSpinBox* duration_{nullptr};
    QSpinBox* periods_{nullptr};
    QComboBox* peak_axis_{nullptr};
    QDoubleSpinBox* peak_threshold_{nullptr};
    QDoubleSpinBox* peak_separation_{nullptr};
    QSpinBox* peak_target_{nullptr};
    QCheckBox* peak_absolute_{nullptr};
    QWidget* timing_panel_{nullptr};
    QWidget* peak_panel_{nullptr};
    QLabel* state_label_{nullptr};
    QLabel* samples_label_{nullptr};
    QLabel* duration_label_{nullptr};
    QLabel* peaks_label_{nullptr};
    QLabel* file_label_{nullptr};
    QLabel* estimate_label_{nullptr};
    QLabel* error_label_{nullptr};
    QLabel* waveform_label_{nullptr};
    QProgressBar* progress_bar_{nullptr};
    QCheckBox* save_attitude_{nullptr};
    QCheckBox* save_reference_{nullptr};
    QCheckBox* save_error_{nullptr};
    QCheckBox* save_rates_{nullptr};
    QCheckBox* save_accel_{nullptr};
    QCheckBox* save_mag_{nullptr};
    QCheckBox* save_baro_{nullptr};
    QCheckBox* save_validity_{nullptr};
    QCheckBox* save_strategy_source_{nullptr};
    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
};

}  // namespace cefiro::gui
