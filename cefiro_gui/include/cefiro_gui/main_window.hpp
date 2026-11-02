#pragma once
#include "cefiro_gui/ros_bridge.hpp"
#include "cefiro_gui/waveform.hpp"
#include "cefiro_gui/recording_dialog.hpp"
#include "cefiro_gui/step_response.hpp"
#include <QMainWindow>
#include <QElapsedTimer>
#include <QFile>
#include <QList>
#include <QTextStream>
#include <array>
#include <memory>
#include <vector>
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;
class QStackedWidget;
class QTabBar;
class QTabWidget;
class QTimer;
class QVBoxLayout;
class QTableWidget;
namespace cefiro::gui {
class ScopePlot;
class AttitudeView;
class AttitudeCommandPad;
class MainWindow final:public QMainWindow {
public:
    explicit MainWindow(QWidget* parent=nullptr);
    ~MainWindow() override;
    void shutdown();
private:
    void buildToolbar();
    void buildDashboard();
    void buildControl();
    void buildSystem();
    void connectUi();
    void tick();
    void tickWaveform();
    void previewWaveform();
    void publishCommand();
    void publishMotorCommand();
    void startOutput();
    void stopOutput(bool emergency);
    void setManual(bool manual);
    void updateMode();
    void setMonitorAxis(int index);
    void setTheme(int index);
    void onSample(const Telemetry& sample,bool draw);
    void showRecordingDialog();
    void startRecording();
    void stopRecording();
    void saveRecord(const Telemetry& sample);
    void exportSummary();
    void sendGains();
    void refreshStepMetrics();
    WaveformConfig waveformConfig() const;
    static QDoubleSpinBox* spin(double min,double max,double value,double step=1.0,int decimals=2);
    static QLabel* label(const QString& text,const QString& object="");
    static QWidget* contentPage(QVBoxLayout*& layout,QWidget* parent);
    QWidget* titledPanel(const QString& title,QVBoxLayout*& inner,QWidget* parent);
    void updateSystem(const Telemetry& sample);
    RosBridge bridge_;
    QStackedWidget* stack_{nullptr};
    QTabBar* nav_{nullptr};
    QLabel* status_ros_{nullptr};
    QLabel* status_rec_{nullptr};
    QLabel* status_control_{nullptr};
    QLabel* status_samples_{nullptr};
    QTimer* refresh_{nullptr};
    QTimer* waveform_timer_{nullptr};
    QTimer* manual_publish_timer_{nullptr};
    std::array<ScopePlot*,6> dash_{};
    std::array<QLabel*,3> attitude_values_{};
    std::array<QLabel*,3> rate_values_{};
    std::array<QLabel*,3> rmse_values_{};
    std::array<QLabel*,3> manual_labels_{};
    std::array<QSlider*,4> motor_sliders_{};
    std::array<QDoubleSpinBox*,4> motor_speed_{};
    std::array<QComboBox*,4> motor_direction_{};
    std::array<QLabel*,4> motor_labels_{};
    QSlider* master_motor_slider_{nullptr};
    QDoubleSpinBox* master_motor_speed_{nullptr};
    QComboBox* master_motor_direction_{nullptr};
    QLabel* master_motor_label_{nullptr};
    AttitudeView* view_{nullptr};
    AttitudeCommandPad* command_pad_{nullptr};
    QStackedWidget* command_source_stack_{nullptr};
    QTabWidget* auto_command_tabs_{nullptr};
    ScopePlot* preview_{nullptr};
    ScopePlot* live_{nullptr};
    QComboBox* controller_{nullptr};
    QComboBox* axis_{nullptr};
    QComboBox* waveform_{nullptr};
    QComboBox* theme_combo_{nullptr};
    QComboBox* monitor_combo_{nullptr};
    QPushButton* manual_btn_{nullptr};
    QPushButton* automatic_btn_{nullptr};
    QPushButton* activate_btn_{nullptr};
    QPushButton* deactivate_btn_{nullptr};
    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QPushButton* emergency_btn_{nullptr};
    QPushButton* record_btn_{nullptr};
    RecordingDialog* recording_dialog_{nullptr};
    RecordingConfig recording_config_{};
    QPushButton* clear_btn_{nullptr};
    QCheckBox* repeat_{nullptr};
    QCheckBox* auto_record_{nullptr};
    QCheckBox* output_enabled_{nullptr};
    QCheckBox* link_all_motors_{nullptr};
    std::array<QDoubleSpinBox*,3> manual_{};
    QDoubleSpinBox *amplitude_{nullptr},*offset_{nullptr},*frequency_{nullptr},*end_frequency_{nullptr},*phase_{nullptr},*delay_{nullptr},*duration_{nullptr},*initial_{nullptr},*final_{nullptr},*rise_{nullptr},*pulse_{nullptr},*duty_{nullptr},*tau_{nullptr};
    QTableWidget* pid_gains_{nullptr};
    QTableWidget* lqr_gains_{nullptr};
    QTableWidget* response_metrics_table_{nullptr};
    QTabWidget* ref_tabs_{nullptr};
    QLabel* command_value_{nullptr};
    QLabel* applied_value_{nullptr};
    QLabel* metric_value_{nullptr};
    QLabel* system_imu_{nullptr};
    QLabel* system_mag_{nullptr};
    QLabel* system_baro_{nullptr};
    QLabel* system_controller_{nullptr};
    QLabel* system_topics_{nullptr};
    QLabel* system_frame_{nullptr};
    QLabel* system_angular_{nullptr};
    QLabel* system_accel_{nullptr};
    QLabel* system_mag_value_{nullptr};
    QLabel* system_pressure_{nullptr};
    QLabel* system_altitude_{nullptr};
    Vec3 command_deg_{};
    std::array<double,4> motor_command_rpm_{};
    std::array<int,4> motor_command_direction_{{1,1,1,1}};
    Vec3 last_published_deg_{};
    std::array<StepResponseAnalyzer,3> response_analyzers_{};
    Telemetry last_sample_{};
    ErrorStats stats_{};
    ErrorStats recording_stats_{};
    QElapsedTimer clock_{};
    QElapsedTimer waveform_clock_{};
    bool have_sample_{false};
    bool manual_mode_{false};
    bool manual_dirty_{false};
    bool command_active_{false};
    bool wave_running_{false};
    bool e_stop_{false};
    bool recorder_active_{false};
    bool shutting_down_{false};
    bool control_tab_active_{false};
    int monitor_axis_{0};
    int theme_index_{0};
    std::uint64_t collected_{0};
    QFile recording_file_;
    QTextStream recording_stream_;
    QString recording_path_;
    QElapsedTimer publish_clock_{};
    QElapsedTimer recording_clock_{};
    quint64 recorded_samples_{0};
    int recording_events_{0};
    double recording_end_s_{-1.0};
    double recording_start_ros_s_{0.0};
    double last_saved_s_{-1.e9};
    double last_peak_s_{-1.e9};
    double peak_prev2_{0.0}, peak_prev1_{0.0};
    bool peak_have_1_{false}, peak_have_2_{false};
    bool recording_should_stop_{false};
};
}
