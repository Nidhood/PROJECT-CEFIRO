#include "cefiro_gui/main_window.hpp"
#include "cefiro_gui/scope_plot.hpp"
#include "cefiro_gui/attitude_view.hpp"
#include "cefiro_gui/attitude_command_pad.hpp"
#include <QSignalBlocker>
#include "cefiro_gui/recording_dialog.hpp"
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QAbstractItemView>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QList>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStatusBar>
#include <QSlider>
#include <QTabBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QVariant>
#include <QTextStream>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace cefiro::gui {
namespace {
const QStringList kAxisNames={"ROLL","PITCH","YAW"};
const QStringList kTypes={"CONSTANT","STEP","RAMP","PULSE","SQUARE","TRIANGLE","SINE","EXPONENTIAL","CHIRP"};
const QStringList kThemes={"Tron Ares","MATLAB / Lab","Light","Dark","Tron Evolution","Cyberpunk Neon"};
struct Theme {QString bg,fg,grid,red,blue,teal;};
const std::array<Theme,6> kTheme{{
 {"#070a0f","#e8eef4","#213549","#ff5468","#69b5f0","#00d8c6"},
 {"#fafafa","#1a242e","#c7d1dd","#0072bd","#d95319","#77ac30"},
 {"#ffffff","#0f172a","#cbd5e1","#3b82f6","#10b981","#8b5cf6"},
 {"#141b2e","#e2e8f0","#334155","#60a5fa","#34d399","#a78bfa"},
 {"#000f1a","#d0f0ff","#135071","#00e6ff","#b084ff","#00ff99"},
 {"#0b0b18","#eaf2ff","#26325b","#ff2bd6","#00f5ff","#ccff00"}}};
QString f(double v,int places=2){return QString::number(std::abs(v)<1.e-7?0.:v,'f',places);}
QFrame* hline(QWidget* p){auto* l=new QFrame(p);l->setFrameShape(QFrame::HLine);l->setObjectName("hudSeparator");return l;}
}
QDoubleSpinBox* MainWindow::spin(double min,double max,double value,double step,int decimals){auto* s=new QDoubleSpinBox;s->setRange(min,max);s->setDecimals(decimals);s->setSingleStep(step);s->setValue(value);s->setKeyboardTracking(false);return s;}
QLabel* MainWindow::label(const QString& text,const QString& id){auto* l=new QLabel(text);if(!id.isEmpty())l->setObjectName(id);return l;}
QWidget* MainWindow::contentPage(QVBoxLayout*& layout,QWidget* parent){
 auto* w=new QWidget(parent);layout=new QVBoxLayout(w);layout->setContentsMargins(12,9,12,12);layout->setSpacing(8);return w;
}
QWidget* MainWindow::titledPanel(const QString& title,QVBoxLayout*& inner,QWidget* parent){
 auto* panel=new QFrame(parent);panel->setObjectName("panel");inner=new QVBoxLayout(panel);inner->setContentsMargins(11,9,11,10);inner->setSpacing(7);inner->addWidget(label(title,"panelTitle"));return panel;
}
MainWindow::MainWindow(QWidget* parent):QMainWindow(parent){
 setWindowTitle("CEFIRO GUI v1.5.0 | ATTITUDE CONTROL HMI");setMinimumSize(1180,700);resize(1600,930);
 clock_.start();publish_clock_.start();
 stack_=new QStackedWidget(this);setCentralWidget(stack_);
 buildToolbar();buildDashboard();buildControl();buildSystem();
 recording_dialog_ = new RecordingDialog(this);
 recording_config_ = recording_dialog_->configuration();
 connectUi();setTheme(0);
 nav_->setCurrentIndex(0);
 stack_->setCurrentIndex(0);
 for (auto* plot : dash_) if (plot) plot->setActive(true);
 if (preview_) preview_->setActive(false);
 if (live_) live_->setActive(false);
 bridge_.start();refresh_=new QTimer(this);refresh_->setInterval(33);refresh_->setTimerType(Qt::CoarseTimer);
 connect(refresh_,&QTimer::timeout,this,[this]{tick();});refresh_->start();
 waveform_timer_=new QTimer(this);waveform_timer_->setInterval(20);waveform_timer_->setTimerType(Qt::PreciseTimer);
 connect(waveform_timer_,&QTimer::timeout,this,[this]{tickWaveform();});
 manual_publish_timer_ = new QTimer(this);
 manual_publish_timer_->setInterval(50);
 connect(manual_publish_timer_, &QTimer::timeout, this, [this] {
     if (command_active_ && manual_mode_) publishMotorCommand();
 });
 manual_publish_timer_->start();
 statusBar()->setSizeGripEnabled(false);
 status_ros_=label("ROS: WAITING");status_rec_=label("REC: IDLE");status_control_=label("CONTROL: DISABLED");status_samples_=label("SAMPLES: 0");
 for(auto* l:{status_ros_,status_rec_,status_control_,status_samples_}){statusBar()->addPermanentWidget(l);statusBar()->addPermanentWidget(label(" | "));}
}
MainWindow::~MainWindow(){shutdown();}
void MainWindow::shutdown(){if(shutting_down_)return;shutting_down_=true;
    if (refresh_) {
        refresh_->stop();
    }
    if (waveform_timer_) {
        waveform_timer_->stop();
    }
    if (manual_publish_timer_) manual_publish_timer_->stop();
    wave_running_ = false;
    if (command_active_) {
        bridge_.publishMotorCommand({0.0,0.0,0.0,0.0}, motor_command_direction_);
        bridge_.setMotorArmed(false);
        bridge_.emergencyStop();
    }
    command_active_ = false;
 stopRecording();bridge_.stop();
}
void MainWindow::buildToolbar(){
 auto* bar=new QToolBar(this);bar->setObjectName("topbar");bar->setMovable(false);bar->setFloatable(false);
 nav_=new QTabBar(bar);nav_->setObjectName("topTabs");nav_->setExpanding(false);nav_->setDrawBase(false);
 for(const QString& name:QStringList{"Dashboard","Signal Generator","System"})nav_->addTab(name);
 connect(nav_,&QTabBar::currentChanged,this,[this](int i){stack_->setCurrentIndex(i);control_tab_active_=(i==1);
   for (auto* p : dash_) {
       if (p) {
           p->setActive(i == 0);
       }
   }
   if (preview_) {
       preview_->setActive(i == 1);
   }
   if (live_) {
       live_->setActive(i == 1);
   }
 });
 bar->addWidget(nav_);auto* spacer=new QWidget(bar);spacer->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);bar->addWidget(spacer);
 bar->addWidget(label("THEME  ","eyebrow"));theme_combo_=new QComboBox(bar);theme_combo_->addItems(kThemes);theme_combo_->setMinimumWidth(160);bar->addWidget(theme_combo_);
 connect(theme_combo_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int i){setTheme(i);});addToolBar(Qt::TopToolBarArea,bar);
}
void MainWindow::buildDashboard(){
 QVBoxLayout* body=nullptr;auto* page=contentPage(body,stack_);stack_->addWidget(page);
 auto* head=new QHBoxLayout;head->addWidget(label("CEFIRO / LIVE TELEMETRY","pageTitle"));head->addStretch();
 clear_btn_=new QPushButton("CLEAR CHARTS");clear_btn_->setObjectName("secondary");head->addWidget(clear_btn_);body->addLayout(head);
 body->addWidget(label("Attitude, angular rates and FMU measurements  ·  cursor hover / zoom controls on each scope","pageSubtitle"));
 auto* scroll=new QScrollArea(page);scroll->setWidgetResizable(true);scroll->setFrameShape(QFrame::NoFrame);body->addWidget(scroll,1);
 auto* container=new QWidget(scroll);scroll->setWidget(container);auto* grid=new QGridLayout(container);grid->setSpacing(8);grid->setContentsMargins(0,2,0,2);
 const QStringList titles={"ROLL / φ","PITCH / θ","YAW / ψ","ANGULAR RATES / p q r","LINEAR ACCELERATION / axyz","MAGNETIC FIELD / Bxyz"};
 const QStringList units={"deg","deg","deg","deg/s","m/s²","µT"};
 for(int i=0;i<6;++i){QVBoxLayout* inner=nullptr;auto* panel=titledPanel(titles[i],inner,page);
    auto* metric=label(i<3?"-- deg":"--","metricValue");if(i<3)attitude_values_[size_t(i)]=metric;else if(i==3)rate_values_[0]=metric;else if(i==4)rate_values_[1]=metric;else rate_values_[2]=metric;
    inner->addWidget(metric);
    QStringList keys=i<3?QStringList{"Reference","Measured"}:i==3?QStringList{"p","q","r"}:i==4?QStringList{"ax","ay","az"}:QStringList{"Bx","By","Bz"};
    dash_[size_t(i)]=new ScopePlot("",units[i],keys,panel);dash_[size_t(i)]->setMinimumHeight(360);
    if(i<3){dash_[size_t(i)]->setDashed(0);dash_[size_t(i)]->setYRange(i==1?-95:-185,i==1?95:185);}
    if(i==3)dash_[size_t(i)]->setYRange(-30,30);
    if(i==4)dash_[size_t(i)]->setYRange(-15,15);
    if(i==5)dash_[size_t(i)]->setYRange(-80,80);
    inner->addWidget(dash_[size_t(i)],1);panel->setMinimumHeight(430);grid->addWidget(panel,i/2,i%2);
 }
 grid->setColumnStretch(0,1);grid->setColumnStretch(1,1);
 connect(clear_btn_,&QPushButton::clicked,this,[this]{for(auto* p:dash_)p->clear();if(live_)live_->clear();stats_.reset();});
}
void MainWindow::buildControl() {
    QVBoxLayout* body = nullptr;
    auto* page = contentPage(body, stack_);
    stack_->addWidget(page);
    body->setContentsMargins(9, 5, 9, 6);
    body->setSpacing(5);

    auto* header = new QHBoxLayout;
    auto* title = new QVBoxLayout;
    title->setSpacing(0);
    title->addWidget(label("CEFIRO / ATTITUDE OPERATIONS", "eyebrow"));
    title->addWidget(label("ATTITUDE CONTROL & EXPERIMENTS", "pageTitle"));
    title->addWidget(label("CEFIRO GUI v1.5.0  ·  ATTITUDE COMMAND & ANALYSIS", "pageSubtitle"));
    header->addLayout(title);
    header->addStretch();
    record_btn_ = new QPushButton("DATA RECORDER", page);
    record_btn_->setObjectName("recordButton");
    header->addWidget(record_btn_);
    body->addLayout(header);

    // Independent columns: the attitude view and response scope get 64% width,
    // and the reference panel never forces a third full-height grid row.
    auto* columns = new QHBoxLayout;
    columns->setSpacing(8);
    body->addLayout(columns, 1);
    auto* left_widget = new QWidget(page);
    auto* left = new QVBoxLayout(left_widget);
    left->setContentsMargins(0, 0, 0, 0);
    left->setSpacing(7);
    left_widget->setMinimumWidth(385);
    auto* right_widget = new QWidget(page);
    auto* right = new QVBoxLayout(right_widget);
    right->setContentsMargins(0, 0, 0, 0);
    right->setSpacing(7);
    right_widget->setMinimumWidth(510);
    columns->addWidget(left_widget, 36);
    columns->addWidget(right_widget, 64);

    QVBoxLayout* arch = nullptr;
    auto* architecture = titledPanel("CONTROL ARCHITECTURE", arch, left_widget);
    arch->setSpacing(4);
    auto* ctrl = new QHBoxLayout;
    ctrl->addWidget(label("STRATEGY", "eyebrow"));
    controller_ = new QComboBox(architecture);
    controller_->addItems({"OPEN LOOP", "PID", "LQR", "CUSTOM"});
    ctrl->addWidget(controller_, 1);
    arch->addLayout(ctrl);
    auto* source = new QHBoxLayout;
    automatic_btn_ = new QPushButton("AUTOMATIC", architecture);
    manual_btn_ = new QPushButton("MANUAL", architecture);
    for (auto* b : {automatic_btn_,manual_btn_}) {
        b->setCheckable(true);
        b->setObjectName("sourceModeButton");
        source->addWidget(b, 1);
    }
    automatic_btn_->setChecked(true);
    arch->addLayout(source);
    auto* actions = new QHBoxLayout;
    activate_btn_ = new QPushButton("APPLY / ACTIVATE", architecture);
    activate_btn_->setObjectName("primaryAction");
    deactivate_btn_ = new QPushButton("DEACTIVATE", architecture);
    deactivate_btn_->setObjectName("secondary");
    actions->addWidget(activate_btn_, 2);
    actions->addWidget(deactivate_btn_, 1);
    arch->addLayout(actions);
    applied_value_ = label("Command output: DISABLED", "hint");
    arch->addWidget(applied_value_);
    left->addWidget(architecture, 0);

    QVBoxLayout* ref = nullptr;
    auto* ref_panel = titledPanel("ATTITUDE COMMAND & ANALYSIS", ref, left_widget);
    ref->setSpacing(4);
    ref_tabs_ = new QTabWidget(ref_panel);
    ref_tabs_->setObjectName("referenceTabs");
    ref_tabs_->setUsesScrollButtons(true);

    auto* reference_tab = new QWidget(ref_tabs_);
    auto* reference_layout = new QVBoxLayout(reference_tab);
    reference_layout->setContentsMargins(4, 4, 4, 3);
    reference_layout->setSpacing(3);
    command_source_stack_ = new QStackedWidget(reference_tab);
    reference_layout->addWidget(command_source_stack_, 1);

    // The command page changes with the selected operating mode.
    // AUTOMATIC: either an interactive 3-axis gimbal OR a trajectory generator.
    // MANUAL: direct motor-by-motor excitation for actuator testing.
    auto* auto_page = new QWidget(command_source_stack_);
    auto* auto_layout = new QVBoxLayout(auto_page);
    auto_layout->setContentsMargins(2, 0, 2, 0);
    auto_layout->setSpacing(4);
    auto_command_tabs_ = new QTabWidget(auto_page);
    auto_command_tabs_->setUsesScrollButtons(true);
    auto_layout->addWidget(auto_command_tabs_, 1);

    auto* gimbal_tab = new QWidget(auto_command_tabs_);
    auto* gimbal_layout = new QVBoxLayout(gimbal_tab);
    gimbal_layout->setContentsMargins(4, 4, 4, 4);
    gimbal_layout->setSpacing(4);
    gimbal_layout->addWidget(label("3-AXIS REFERENCE GIMBAL  /  DRAG THE COLORED RINGS", "eyebrow"));
    command_pad_ = new AttitudeCommandPad(gimbal_tab);
    command_pad_->setMinimumHeight(210);
    gimbal_layout->addWidget(command_pad_, 1);
    auto* auto_fields = new QGridLayout;
    auto_fields->setHorizontalSpacing(4);
    auto_fields->setVerticalSpacing(2);
    for (int i=0; i<3; ++i) {
        const double limit = i == 1 ? 89.0 : 180.0;
        auto_fields->addWidget(label(kAxisNames[i], "eyebrow"), 0, i);
        manual_[std::size_t(i)] = spin(-limit, limit, 0., 1., 2);
        auto_fields->addWidget(manual_[std::size_t(i)], 1, i);
        manual_labels_[std::size_t(i)] = label("0.00°", "hint");
        auto_fields->addWidget(manual_labels_[std::size_t(i)], 2, i);
    }
    gimbal_layout->addLayout(auto_fields);
    gimbal_layout->addWidget(label("Wheel: 1° increments · Double-click: zero selected axis", "hint"));
    auto_command_tabs_->addTab(gimbal_tab, "Reference gimbal");

    auto* trajectory_tab = new QWidget(auto_command_tabs_);
    auto* trajectory_layout = new QVBoxLayout(trajectory_tab);
    trajectory_layout->setContentsMargins(4, 4, 4, 4);
    trajectory_layout->setSpacing(4);
    trajectory_layout->addWidget(label("AUTOMATIC TRAJECTORY GENERATOR", "eyebrow"));
    auto* signal_select = new QHBoxLayout;
    signal_select->addWidget(label("AXIS", "eyebrow"));
    axis_ = new QComboBox(trajectory_tab);
    axis_->addItems(kAxisNames);
    signal_select->addWidget(axis_, 1);
    signal_select->addWidget(label("WAVEFORM", "eyebrow"));
    waveform_ = new QComboBox(trajectory_tab);
    waveform_->addItems(kTypes);
    signal_select->addWidget(waveform_, 2);
    trajectory_layout->addLayout(signal_select);
    auto* wavefields = new QGridLayout;
    wavefields->setHorizontalSpacing(8);
    wavefields->setVerticalSpacing(3);
    const QStringList fieldnames={"Amplitude [deg]","Offset [deg]","Frequency [Hz]",
      "End frequency [Hz]","Phase [deg]","Delay [s]","Duration [s]",
      "Initial [deg]","Final [deg]","Rise time [s]","Pulse width [s]",
      "Duty cycle [%]","Tau [s]"};
    std::array<QDoubleSpinBox**,13> ptrs={&amplitude_,&offset_,&frequency_,
      &end_frequency_,&phase_,&delay_,&duration_,&initial_,&final_,&rise_,
      &pulse_,&duty_,&tau_};
    const double defaults[]={10,0,.25,2,0,0,10,0,10,2,.5,50,1};
    for (int i=0; i<13; ++i) {
        const double lo = i==2||i==3 ? .001 :
            (i==5||i==6||i==9||i==10||i==12 ? .001 : (i==11 ? 0.0 : -180.0));
        const double hi = i==2||i==3 ? 20.0 :
            (i==5||i==6||i==9||i==10||i==12 ? 3600.0 :
             (i==11 ? 100.0 : 180.0));
        *ptrs[std::size_t(i)] = spin(lo, hi, defaults[i],
                                     i==2||i==3 ? .05 : .5, 3);
        const int row = i / 2;
        const int col = (i % 2) * 2;
        wavefields->addWidget(label(fieldnames[i],"eyebrow"),row,col);
        wavefields->addWidget(*ptrs[std::size_t(i)],row,col+1);
    }
    wavefields->setColumnStretch(1, 1);
    wavefields->setColumnStretch(3, 1);
    trajectory_layout->addLayout(wavefields);
    repeat_ = new QCheckBox("Repeat signal", trajectory_tab);
    trajectory_layout->addWidget(repeat_);
    auto_command_tabs_->addTab(trajectory_tab, "Trajectory generator");
    command_source_stack_->addWidget(auto_page);

    auto* manual_page = new QWidget(command_source_stack_);
    auto* manual_layout = new QVBoxLayout(manual_page);
    manual_layout->setContentsMargins(2, 0, 2, 0);
    manual_layout->setSpacing(4);
    manual_layout->addWidget(label("DIRECT MOTOR COMMAND  /  TEST EACH ROTOR INDEPENDENTLY", "eyebrow"));
    auto* motor_hint = label("NOMINAL RPM REQUESTS mapped open-loop to DShot, NOT measured RPM. The Raspberry configures the DShot cap (300 by default; protocol max 2047). MASTER links all four requests; disable link to trim each motor. Direction is determined by physical phases.", "hint");
    motor_hint->setWordWrap(true);
    manual_layout->addWidget(motor_hint);

    auto* master_panel = new QFrame(manual_page);
    master_panel->setObjectName("panel");
    auto* master_layout = new QGridLayout(master_panel);
    master_layout->setContentsMargins(8, 8, 8, 8);
    master_layout->setHorizontalSpacing(8);
    master_layout->setVerticalSpacing(4);
    master_layout->addWidget(label("MASTER COMMAND  /  APPLY SAME REQUEST TO ALL MOTORS", "eyebrow"), 0, 0, 1, 5);
    link_all_motors_ = new QCheckBox("Link all motors to master", master_panel);
    master_layout->addWidget(link_all_motors_, 1, 0, 1, 2);
    master_layout->addWidget(label("MASTER NOMINAL RPM", "eyebrow"), 1, 2);
    master_motor_speed_ = spin(0.0, 12000.0, 0.0, 50.0, 0);
    master_layout->addWidget(master_motor_speed_, 1, 3);
    master_motor_direction_ = new QComboBox(master_panel);
    master_motor_direction_->addItems({"Direction: hardware phases"});
    master_motor_direction_->setEnabled(false);
    master_layout->addWidget(master_motor_direction_, 1, 4);
    master_motor_slider_ = new QSlider(Qt::Horizontal, master_panel);
    master_motor_slider_->setRange(0, 12000);
    master_motor_slider_->setTickPosition(QSlider::TicksBelow);
    master_motor_slider_->setTickInterval(1000);
    master_motor_slider_->setValue(0);
    master_motor_slider_->setStyleSheet(
        "QSlider::groove:horizontal { background:#162431; border:1px solid #213549; height:16px; border-radius:8px; }"
        "QSlider::sub-page:horizontal { background:#69b5f0; border-radius:8px; }"
        "QSlider::add-page:horizontal { background:#0d1620; border-radius:8px; }"
        "QSlider::handle:horizontal { background:#ff5468; border:1px solid #070a0f; width:22px; margin:-4px 0; border-radius:11px; }");
    master_layout->addWidget(master_motor_slider_, 2, 0, 1, 5);
    master_motor_label_ = label("0 nominal RPM · DShot cap: awaiting bridge", "hint");
    master_motor_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    master_layout->addWidget(master_motor_label_, 3, 0, 1, 5);
    master_layout->setColumnStretch(0, 1);
    master_layout->setColumnStretch(1, 1);
    master_layout->setColumnStretch(2, 0);
    master_layout->setColumnStretch(3, 0);
    master_layout->setColumnStretch(4, 0);
    manual_layout->addWidget(master_panel);

    auto* motors = new QHBoxLayout;
    motors->setSpacing(8);
    for (int i=0; i<4; ++i) {
        auto* motor_panel = new QFrame(manual_page);
        motor_panel->setObjectName("chartFrame");
        auto* mv = new QVBoxLayout(motor_panel);
        mv->setContentsMargins(8,6,8,6);
        mv->setSpacing(4);
        auto* name = label(QString("MOTOR %1").arg(i+1), "panelTitle");
        name->setAlignment(Qt::AlignCenter);
        mv->addWidget(name);
        motor_sliders_[std::size_t(i)] = new QSlider(Qt::Vertical, motor_panel);
        motor_sliders_[std::size_t(i)]->setRange(0, 12000);
        motor_sliders_[std::size_t(i)]->setTickPosition(QSlider::TicksLeft);
        motor_sliders_[std::size_t(i)]->setTickInterval(1000);
        motor_sliders_[std::size_t(i)]->setValue(0);
        motor_sliders_[std::size_t(i)]->setFixedHeight(250);
        motor_sliders_[std::size_t(i)]->setFixedWidth(46);
        motor_sliders_[std::size_t(i)]->setStyleSheet(
            "QSlider::groove:vertical { background:#162431; border:1px solid #213549; width:20px; border-radius:10px; }"
            "QSlider::sub-page:vertical { background:#69b5f0; border-radius:10px; }"
            "QSlider::add-page:vertical { background:#0d1620; border-radius:10px; }"
            "QSlider::handle:vertical { background:#ff5468; border:1px solid #070a0f; height:24px; margin:0 -6px; border-radius:12px; }");
        mv->addWidget(motor_sliders_[std::size_t(i)], 1, Qt::AlignHCenter);
        motor_speed_[std::size_t(i)] = spin(0.0, 12000.0, 0.0, 50.0, 0);
        mv->addWidget(motor_speed_[std::size_t(i)]);
        motor_direction_[std::size_t(i)] = new QComboBox(motor_panel);
        motor_direction_[std::size_t(i)]->addItems({"Direction: hardware phases"});
        motor_direction_[std::size_t(i)]->setEnabled(false);
        mv->addWidget(motor_direction_[std::size_t(i)]);
        motor_labels_[std::size_t(i)] = label("0 nominal RPM · DShot TX 0", "hint");
        motor_labels_[std::size_t(i)]->setAlignment(Qt::AlignCenter);
        mv->addWidget(motor_labels_[std::size_t(i)]);
        motors->addWidget(motor_panel, 1);
    }
    manual_layout->addLayout(motors, 1);
    manual_layout->addWidget(label("Bench use: keep the cap at 300 initially, remove propellers, secure the drone and measure current/temperature. After validation use hardware launch argument dshot_max:=2047. No RPM feedback or physical direction control is provided by DShot here.", "hint"));
    command_source_stack_->addWidget(manual_page);
    ref_tabs_->addTab(reference_tab, "Command");

    auto* preview_tab = new QWidget(ref_tabs_);
    auto* pv = new QVBoxLayout(preview_tab);
    pv->setContentsMargins(4, 4, 4, 3);
    preview_ = new ScopePlot("REFERENCE PREVIEW", "deg", {"Reference"}, preview_tab);
    preview_->setMinimumHeight(115);
    pv->addWidget(preview_, 1);
    pv->addWidget(label("Preview matches the published generator waveform.", "hint"));
    ref_tabs_->addTab(preview_tab, "Preview");

    auto* topology_tab = new QWidget(ref_tabs_);
    auto* tp = new QVBoxLayout(topology_tab);
    tp->setSpacing(7);
    for (const QString& text : QStringList{
            "REFERENCE  /  φ θ ψ  [rad]", "↓",
            "EXTERNAL PID / LQR / CUSTOM CONTROLLER", "↓",
            "CEFIRO / GAZEBO GIMBAL", "↓",
            "FMU IMU → ROS 2 TELEMETRY → RECORDER"}) {
        auto* item = label(text, "topologyLine");
        item->setAlignment(Qt::AlignCenter);
        tp->addWidget(item);
    }
    tp->addStretch();
    ref_tabs_->addTab(topology_tab, "Topology");

    auto* gains_tab = new QWidget(ref_tabs_);
    auto* gains_layout = new QVBoxLayout(gains_tab);
    gains_layout->setContentsMargins(4,4,4,4);
    gains_layout->setSpacing(3);
    auto* gain_tabs = new QTabWidget(gains_tab);
    pid_gains_ = new QTableWidget(3,3,gain_tabs);
    pid_gains_->setHorizontalHeaderLabels({"Kp","Ki","Kd"});
    pid_gains_->setVerticalHeaderLabels(kAxisNames);
    pid_gains_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    for (int r=0; r<3; ++r)
        for (int c=0; c<3; ++c)
            pid_gains_->setItem(r,c,new QTableWidgetItem(c==0 ? "0.500" : "0.000"));
    gain_tabs->addTab(pid_gains_,"PID gains");
    lqr_gains_ = new QTableWidget(3,6,gain_tabs);
    lqr_gains_->setHorizontalHeaderLabels({"φ","θ","ψ","p","q","r"});
    lqr_gains_->setVerticalHeaderLabels(kAxisNames);
    lqr_gains_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    for (int r=0; r<3; ++r)
        for (int c=0; c<6; ++c)
            lqr_gains_->setItem(r,c,new QTableWidgetItem("0.000"));
    gain_tabs->addTab(lqr_gains_,"LQR / K 3×6");
    gains_layout->addWidget(gain_tabs,1);
    auto* send = new QPushButton("PUBLISH CONTROLLER PARAMETERS",gains_tab);
    send->setObjectName("secondary");
    gains_layout->addWidget(send);
    connect(send,&QPushButton::clicked,this,[this]{sendGains();});
    gains_layout->addWidget(label("Requires a compatible external controller subscriber.","hint"));
    ref_tabs_->addTab(gains_tab,"Gains");

    auto* metrics_tab = new QWidget(ref_tabs_);
    auto* metrics_layout = new QVBoxLayout(metrics_tab);
    metrics_layout->setContentsMargins(4,4,4,4);
    metrics_layout->setSpacing(3);
    const QStringList metric_names = {"Step [°]", "Peak [°]", "Peak time [s]", "Overshoot [%]",
        "Dead time 5% [s]", "Rise 10–90% [s]", "Settling ±2% [s]",
        "Ringing [cross]", "Steady error [°]", "RMSE [°]", "MAE [°]",
        "Max error [°]"};
    response_metrics_table_ = new QTableWidget(metric_names.size(),3,metrics_tab);
    response_metrics_table_->setHorizontalHeaderLabels(kAxisNames);
    response_metrics_table_->setVerticalHeaderLabels(metric_names);
    response_metrics_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    response_metrics_table_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    response_metrics_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    response_metrics_table_->setSelectionMode(QAbstractItemView::NoSelection);
    for (int row=0; row<metric_names.size(); ++row)
        for (int col=0; col<3; ++col)
            response_metrics_table_->setItem(row,col,new QTableWidgetItem("—"));
    metrics_layout->addWidget(response_metrics_table_,1);
    auto* note = label("Metrics require a measured step ≥1°. Settling needs ≥0.5 s in band."
                       " Dashes indicate insufficient evidence.","hint");
    note->setWordWrap(true);
    metrics_layout->addWidget(note);
    ref_tabs_->addTab(metrics_tab,"Metrics");
    ref->addWidget(ref_tabs_, 1);

    auto* options = new QHBoxLayout;
    auto_record_ = new QCheckBox("Auto record",ref_panel);
    output_enabled_ = new QCheckBox("Output enabled",ref_panel);
    options->addWidget(auto_record_);
    options->addWidget(output_enabled_);
    options->addStretch();
    ref->addLayout(options);
    auto* commands = new QHBoxLayout;
    start_btn_ = new QPushButton("START TEST",ref_panel);
    start_btn_->setObjectName("primaryAction");
    stop_btn_ = new QPushButton("SAFE STOP",ref_panel);
    stop_btn_->setObjectName("secondary");
    emergency_btn_ = new QPushButton("EMERGENCY ZERO",ref_panel);
    emergency_btn_->setObjectName("emergencyButton");
    for (auto* b : {start_btn_,stop_btn_,emergency_btn_}) commands->addWidget(b);
    ref->addLayout(commands);
    command_value_ = label("Reference: R 0.00° · P 0.00° · Y 0.00°","hint");
    ref->addWidget(command_value_);
    left->addWidget(ref_panel,1);

    QVBoxLayout* visual = nullptr;
    auto* visual_panel = titledPanel("ATTITUDE / IMU FRAME",visual,right_widget);
    view_ = new AttitudeView(visual_panel);
    view_->setMinimumSize(260,160);
    visual->addWidget(view_,1);
    auto* readouts = new QGridLayout;
    for (int i=0; i<3; ++i) {
        readouts->addWidget(label(kAxisNames[i],"eyebrow"),0,i);
        rmse_values_[std::size_t(i)] = label("-- ° / RMSE --","metricValue");
        readouts->addWidget(rmse_values_[std::size_t(i)],1,i);
    }
    visual->addLayout(readouts);
    right->addWidget(visual_panel,6);

    QVBoxLayout* liveinner = nullptr;
    auto* livepanel = titledPanel("LIVE ATTITUDE RESPONSE",liveinner,right_widget);
    auto* monitor_row = new QHBoxLayout;
    monitor_row->addWidget(label("MONITOR AXIS","eyebrow"));
    monitor_combo_ = new QComboBox(livepanel);
    monitor_combo_->addItems(kAxisNames);
    monitor_row->addWidget(monitor_combo_);
    monitor_row->addStretch();
    metric_value_ = label("RMSE --°  |  MAE --°  |  PEAK --°","metricValue");
    monitor_row->addWidget(metric_value_);
    liveinner->addLayout(monitor_row);
    live_ = new ScopePlot("REFERENCE vs MEASURED","deg",{"Reference","Measured"},livepanel);
    live_->setMinimumHeight(125);
    live_->setDashed(0);
    live_->setYRange(-60,60);
    liveinner->addWidget(live_,1);
    right->addWidget(livepanel,4);
}
void MainWindow::buildSystem(){
 QVBoxLayout* root=nullptr;auto* page=contentPage(root,stack_);stack_->addWidget(page);
 root->addWidget(label("CEFIRO / SYSTEM STATUS","pageTitle"));root->addWidget(label("Live ROS 2 connection, IMU orientation, sensor frames and controller feedback","pageSubtitle"));
 auto* g=new QGridLayout;g->setSpacing(9);root->addLayout(g);
 QVBoxLayout* sensors=nullptr;auto* sensor_panel=titledPanel("FMU SENSOR LINK",sensors,page);
 system_imu_=label("IMU: WAITING","metricValue");system_mag_=label("MAG: WAITING","metricValue");system_baro_=label("BARO: WAITING","metricValue");
 system_frame_=label("Frames: --","hint");system_angular_=label("Gyro: -- deg/s","metricValue");system_accel_=label("Accel: -- m/s²","metricValue");system_mag_value_=label("Mag: -- µT","metricValue");system_pressure_=label("Pressure: -- hPa","metricValue");system_altitude_=label("Z relative: -- m","metricValue");
 for(QWidget* w : QList<QWidget*>{system_imu_,system_mag_,system_baro_,hline(sensor_panel),system_frame_,system_angular_,system_accel_,system_mag_value_,system_pressure_,system_altitude_})sensors->addWidget(w);
 g->addWidget(sensor_panel,0,0);
 QVBoxLayout* ros=nullptr;auto* ros_panel=titledPanel("ROS 2 INTERFACE",ros,page);
 system_controller_=label("Controller: NO FEEDBACK","metricValue");system_topics_=label("Topics initializing...","hint");system_topics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
 ros->addWidget(system_controller_);ros->addWidget(hline(ros_panel));ros->addWidget(system_topics_);ros->addStretch();g->addWidget(ros_panel,0,1);
 g->setColumnStretch(0,1);g->setColumnStretch(1,1);
 auto* note=label("The GUI publishes controller requests but does not implement attitude dynamics, controllers, actuators or motor arming.\n"
 "A controller node must subscribe to the documented interface and report status before actuation can be confirmed.","hint");root->addWidget(note);root->addStretch();
 system_topics_->setText(QString("IMU: %1\nMAG: %2\nBARO: %3\nREF: %4\n"
 "ENABLE: /cefiro/control/enable\nSTOP: /cefiro/control/emergency_stop\n"
 "STATUS: /cefiro/control/status\nPID: /cefiro/control/pid_gains\nLQR: /cefiro/control/lqr_gain")
 .arg(QString::fromStdString(bridge_.imuTopic()),QString::fromStdString(bridge_.magTopic()),QString::fromStdString(bridge_.baroTopic()),QString::fromStdString(bridge_.refTopic())));
}
void MainWindow::connectUi(){
 connect(manual_btn_,&QPushButton::clicked,this,[this]{setManual(true);});
 connect(automatic_btn_,&QPushButton::clicked,this,[this]{setManual(false);});
 command_pad_->setOnEdited([this](int axis, double value) {
     if (axis < 0 || axis > 2 || manual_mode_) return;
     {
         QSignalBlocker block(manual_[std::size_t(axis)]);
         manual_[std::size_t(axis)]->setValue(value);
     }
     manual_labels_[std::size_t(axis)]->setText(f(value) + "°");
     command_deg_[std::size_t(axis)] = value;
     manual_dirty_ = true;
     if (command_active_ && publish_clock_.elapsed() > 45) publishCommand();
 });
 for(int i=0;i<3;++i){connect(manual_[size_t(i)],QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this,i](double value){
   manual_labels_[size_t(i)]->setText(f(value)+"°");
   command_pad_->setAngles({manual_[0]->value(), manual_[1]->value(), manual_[2]->value()});
   if (!manual_mode_) {
       command_deg_[size_t(i)] = value;
       manual_dirty_ = true;
       if (command_active_ && publish_clock_.elapsed() > 45) publishCommand();
   }
 });}
 auto updateMotorLabel = [this](int idx) {
   if (idx < 0 || idx >= 4) return;
   motor_labels_[size_t(idx)]->setText(
       QString("Nom %1 · dir %2")
           .arg(f(motor_command_rpm_[size_t(idx)],0))
           .arg(motor_command_direction_[size_t(idx)] >= 0 ? "+1" : "-1"));
 };
 auto updateMasterLabel = [this]() {
   if (!master_motor_label_) return;
   const int dir = master_motor_direction_ && master_motor_direction_->currentIndex() == 0 ? 1 : -1;
   master_motor_label_->setText(QString("Nom %1 · dir %2")
       .arg(f(master_motor_speed_ ? master_motor_speed_->value() : 0.0,0))
       .arg(dir >= 0 ? "+1" : "-1"));
 };
 auto syncAllMotors = [this, updateMotorLabel](double rpm, int direction_index, bool sync_direction) {
   for (int i = 0; i < 4; ++i) {
     {
       QSignalBlocker block1(motor_sliders_[size_t(i)]);
       QSignalBlocker block2(motor_speed_[size_t(i)]);
       motor_sliders_[size_t(i)]->setValue(int(std::round(rpm)));
       motor_speed_[size_t(i)]->setValue(rpm);
     }
     motor_command_rpm_[size_t(i)] = rpm;
     if (sync_direction) {
       QSignalBlocker block3(motor_direction_[size_t(i)]);
       motor_direction_[size_t(i)]->setCurrentIndex(direction_index);
       motor_command_direction_[size_t(i)] = direction_index == 0 ? 1 : -1;
     }
     updateMotorLabel(i);
   }
   if (command_active_ && manual_mode_ && publish_clock_.elapsed() > 45) publishMotorCommand();
 };
 for(int i=0;i<4;++i){
   connect(motor_sliders_[size_t(i)], &QSlider::valueChanged, this, [this,i,updateMotorLabel](int value){
       QSignalBlocker block(motor_speed_[size_t(i)]);
       motor_speed_[size_t(i)]->setValue(value);
       motor_command_rpm_[size_t(i)] = value;
       updateMotorLabel(i);
       if (command_active_ && manual_mode_ && publish_clock_.elapsed() > 45) publishMotorCommand();
   });
   connect(motor_speed_[size_t(i)], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this,i,updateMotorLabel](double value){
       QSignalBlocker block(motor_sliders_[size_t(i)]);
       motor_sliders_[size_t(i)]->setValue(int(std::round(value)));
       motor_command_rpm_[size_t(i)] = value;
       updateMotorLabel(i);
       if (command_active_ && manual_mode_ && publish_clock_.elapsed() > 45) publishMotorCommand();
   });
   connect(motor_direction_[size_t(i)], QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this,i,updateMotorLabel](int idx){
       motor_command_direction_[size_t(i)] = idx == 0 ? 1 : -1;
       updateMotorLabel(i);
       if (command_active_ && manual_mode_ && publish_clock_.elapsed() > 45) publishMotorCommand();
   });
 }
 connect(master_motor_slider_, &QSlider::valueChanged, this, [this,syncAllMotors,updateMasterLabel](int value){
     QSignalBlocker block(master_motor_speed_);
     master_motor_speed_->setValue(value);
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncAllMotors(value, master_motor_direction_->currentIndex(), false);
     }
 });
 connect(master_motor_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this,syncAllMotors,updateMasterLabel](double value){
     QSignalBlocker block(master_motor_slider_);
     master_motor_slider_->setValue(int(std::round(value)));
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncAllMotors(value, master_motor_direction_->currentIndex(), false);
     }
 });
 connect(master_motor_direction_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this,syncAllMotors,updateMasterLabel](int idx){
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncAllMotors(master_motor_speed_->value(), idx, true);
     }
 });
 connect(link_all_motors_, &QCheckBox::toggled, this, [this,syncAllMotors](bool on){
     for (int i = 0; i < 4; ++i) {
         if (motor_sliders_[size_t(i)]) motor_sliders_[size_t(i)]->setEnabled(manual_mode_ && !on);
         if (motor_speed_[size_t(i)]) motor_speed_[size_t(i)]->setEnabled(manual_mode_ && !on);
         if (motor_direction_[size_t(i)]) motor_direction_[size_t(i)]->setEnabled(false);
     }
     if (on) {
         syncAllMotors(master_motor_speed_->value(), master_motor_direction_->currentIndex(), true);
     }
 });
 updateMasterLabel();
 connect(activate_btn_,&QPushButton::clicked,this,[this]{startOutput();});
 connect(deactivate_btn_,&QPushButton::clicked,this,[this]{stopOutput(false);});
 connect(start_btn_,&QPushButton::clicked,this,[this]{startOutput();});
 connect(stop_btn_,&QPushButton::clicked,this,[this]{stopOutput(false);});
 connect(emergency_btn_,&QPushButton::clicked,this,[this]{stopOutput(true);});
 connect(output_enabled_,&QCheckBox::toggled,this,[this](bool on){if(!on && command_active_)stopOutput(false);});
 connect(record_btn_,&QPushButton::clicked,this,[this]{showRecordingDialog();});
 connect(recording_dialog_->startButton(), &QPushButton::clicked, this,
         [this] { startRecording(); });
 connect(recording_dialog_->stopButton(), &QPushButton::clicked, this,
         [this] { stopRecording(); });
 connect(monitor_combo_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int i){setMonitorAxis(i);});
 auto preview=[this]{previewWaveform();};
 for(auto* d:{amplitude_,offset_,frequency_,end_frequency_,phase_,delay_,duration_,initial_,final_,rise_,pulse_,duty_,tau_}){
   connect(d,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,preview);
 }
 connect(waveform_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,preview);
 connect(axis_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,preview);
 connect(repeat_,&QCheckBox::toggled,this,preview);
 previewWaveform();setManual(false);
}
WaveformConfig MainWindow::waveformConfig()const{
 WaveformConfig c;c.type=static_cast<WaveformType>(waveform_->currentIndex());c.amplitude=amplitude_->value();
 c.offset=offset_->value();c.frequency_hz=frequency_->value();c.end_frequency_hz=end_frequency_->value();
 c.phase_deg=phase_->value();c.delay_s=delay_->value();c.duration_s=duration_->value();
 c.initial_deg=initial_->value();c.final_deg=final_->value();c.rise_s=rise_->value();
 c.pulse_s=pulse_->value();c.duty_percent=duty_->value();c.tau_s=tau_->value();c.repeat=repeat_->isChecked();return c;
}
void MainWindow::previewWaveform(){if(!preview_)return;const auto c=waveformConfig();QList<QPointF> points;points.reserve(501);
 const double total=std::max(1.0,c.delay_s+c.duration_s);for(int i=0;i<=500;++i){const double t=total*double(i)/500.0;const double v=clampFinite(waveformValue(c,t),-180,180);points.push_back({t,v});}
 preview_->setSeriesData(0,points);preview_->setWindowSeconds(std::clamp(total,1.0,60.0));preview_->setYRange(-190,190);
}
void MainWindow::setManual(bool yes){
 if (command_active_ && manual_mode_ != yes) stopOutput(false);
 manual_mode_=yes;
 manual_dirty_=false;
 manual_btn_->setChecked(yes);
 automatic_btn_->setChecked(!yes);
 if (ref_tabs_) ref_tabs_->setCurrentIndex(0);
 if (waveform_) waveform_->setEnabled(!yes);
 if (axis_) axis_->setEnabled(!yes);
 for(auto* d:{amplitude_,offset_,frequency_,end_frequency_,phase_,delay_,duration_,initial_,final_,rise_,pulse_,duty_,tau_}) if(d) d->setEnabled(!yes);
 for(auto* m:manual_) if(m) m->setEnabled(!yes);
 const bool linked = link_all_motors_ && link_all_motors_->isChecked();
 for(auto* s:motor_sliders_) if(s) s->setEnabled(yes && !linked);
 for(auto* s:motor_speed_) if(s) s->setEnabled(yes && !linked);
 for(auto* d:motor_direction_) if(d) d->setEnabled(false);
 if (master_motor_slider_) master_motor_slider_->setEnabled(yes);
 if (master_motor_speed_) master_motor_speed_->setEnabled(yes);
 if (master_motor_direction_) master_motor_direction_->setEnabled(false);
 if (link_all_motors_) link_all_motors_->setEnabled(yes);
 command_source_stack_->setCurrentIndex(yes ? 1 : 0);
 if (command_pad_) {
   command_pad_->setEnabledForEditing(!yes);
   command_pad_->setAngles({manual_[0]->value(),manual_[1]->value(),manual_[2]->value()});
 }
 if(wave_running_){waveform_timer_->stop();wave_running_=false;}
 if(!yes) for(int i=0;i<3;++i) command_deg_[size_t(i)] = manual_[size_t(i)]->value();
 if(command_active_){ if(manual_mode_) publishMotorCommand(); else publishCommand(); }
}
void MainWindow::startOutput(){
 if(!output_enabled_->isChecked()){
  QMessageBox::information(this,"Output inhibited","Enable 'Output enabled' before sending controller commands.");return;
 }
 if(!manual_mode_ && !bridge_.imuFresh()){
  QMessageBox::warning(this,"IMU unavailable","No recent IMU data. Automatic reference generation is inhibited to avoid blind actuation.");return;
 }
 if(!manual_mode_ && !bridge_.feedbackFresh()){
  const auto response=QMessageBox::warning(this,"Controller feedback unavailable",
    "No external controller has acknowledged the command interface.\n"
    "Start publishing references anyway? This does NOT verify any actuator is armed.",
    QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
  if(response!=QMessageBox::Yes)return;
 }
 e_stop_=false;command_active_=true;bridge_.setEnabled(true);
 if (manual_mode_) bridge_.setMotorArmed(true);
 activate_btn_->setText("COMMANDS ENABLED");
 if(auto_record_->isChecked()&&!recorder_active_)startRecording();
 if(manual_mode_){
   wave_running_=false;
   publishMotorCommand();
 } else {
   waveform_clock_.start();
   wave_running_=true;
   waveform_timer_->start();
   tickWaveform();
 }
}
void MainWindow::stopOutput(bool emergency){
    if (waveform_timer_) {
        waveform_timer_->stop();
    }
    wave_running_ = false;
    command_active_ = false;
    e_stop_ = emergency;
    command_deg_={0.,0.,0.};
    bridge_.publishReference(command_deg_,"ATTITUDE",controller_->currentText().toStdString());
    bridge_.publishMotorCommand({0.0,0.0,0.0,0.0}, motor_command_direction_);
    bridge_.setMotorArmed(false);
    last_published_deg_ = command_deg_;
    if(emergency)bridge_.emergencyStop();else bridge_.setEnabled(false);
    activate_btn_->setText("APPLY / ACTIVATE");
    applied_value_->setText(emergency?"EMERGENCY STOP requested · disabled":"Command output: DISABLED");
    if(auto_record_->isChecked()&&recorder_active_)stopRecording();
}
void MainWindow::tickWaveform(){
 if(!wave_running_||!command_active_)return;
 const auto c=waveformConfig();const double t=waveform_clock_.elapsed()/1000.0;
 if(!c.repeat&&t>=c.delay_s+c.duration_s){waveform_timer_->stop();wave_running_=false;stopOutput(false);return;}
 command_deg_[size_t(axis_->currentIndex())]=clampFinite(waveformValue(c,t),axis_->currentIndex()==1?-89.0:-180.0,axis_->currentIndex()==1?89.0:180.0);
 publishCommand();
}
void MainWindow::publishCommand(){
 if(!command_active_||e_stop_)return;
 publish_clock_.restart();
 if (have_sample_ && last_sample_.orientation_valid && !manual_mode_) {
     for (int axis = 0; axis < 3; ++axis) {
         if (std::abs(angleErrorDeg(command_deg_[std::size_t(axis)],
                                    last_published_deg_[std::size_t(axis)])) >= 1.0) {
             response_analyzers_[std::size_t(axis)].start(
                 last_sample_.ros_time_s,
                 last_published_deg_[std::size_t(axis)],
                 command_deg_[std::size_t(axis)],
                 last_sample_.attitude_deg[std::size_t(axis)]);
         }
     }
 }
 bridge_.publishReference(command_deg_,"ATTITUDE",controller_->currentText().toStdString());
 last_published_deg_ = command_deg_;
 manual_dirty_ = false;
 command_value_->setText(QString("Reference: R %1° · P %2° · Y %3°").arg(f(command_deg_[0]),f(command_deg_[1]),f(command_deg_[2])));
 applied_value_->setText("Reference request published · awaiting controller feedback");
}
void MainWindow::publishMotorCommand(){
 if(!command_active_||e_stop_)return;
 publish_clock_.restart();
 bridge_.publishMotorCommand(motor_command_rpm_, motor_command_direction_);
 QStringList summary;
 for(int i=0;i<4;++i){
   summary << QString("M%1 %2 rpm (%3)").arg(i+1).arg(f(motor_command_rpm_[size_t(i)],0)).arg(motor_command_direction_[size_t(i)] >= 0 ? "+" : "-");
 }
 command_value_->setText("Motors: " + summary.join(" · "));
 applied_value_->setText("Nominal motor commands published (open-loop DShot); hardware phases determine direction");
}
void MainWindow::setMonitorAxis(int idx){if(idx<0||idx>2)return;monitor_axis_=idx;live_->clear();live_->setYRange(idx==1?-95.0:-185.0,idx==1?95.0:185.0);}
void MainWindow::tick(){
 const auto items=bridge_.drain();const bool fresh=bridge_.imuFresh();
 status_ros_->setText(fresh?"ROS: IMU ONLINE":"ROS: IMU WAITING / STALE");
 status_control_->setText(command_active_?"OUTPUT REQUEST: ENABLED":"CONTROL: DISABLED");
 if(bridge_.feedbackFresh())status_control_->setText("CTRL: "+QString::fromStdString(bridge_.controllerStatus()));
 // Show the bridge's requested DShot values, never a fabricated measured RPM.
 // The host's configured ceiling is authoritative (300 unless explicitly raised).
 if (manual_mode_) {
   if (bridge_.hardwareStatusFresh()) {
     const auto json=QJsonDocument::fromJson(QByteArray::fromStdString(bridge_.hardwareStatus()));
     if (json.isObject()) {
       const auto state=json.object();
       const int max_dshot=state.value("configured_dshot_max").toInt(300);
       const auto commands=state.value("last_commanded_dshot").toArray();
       const bool serial_ok=state.value("serial_connected").toBool(false);
       if (master_motor_label_) {
         master_motor_label_->setText(QString("%1 nominal RPM · host DShot cap %2/2047 · UART %3")
           .arg(f(master_motor_speed_ ? master_motor_speed_->value() : 0.0,0))
           .arg(max_dshot)
           .arg(serial_ok ? "connected" : "offline"));
       }
       for (int i=0; i<4; ++i) {
         if (!motor_labels_[size_t(i)]) continue;
         const int requested=commands.size()>i ? commands.at(i).toInt(0) : 0;
         motor_labels_[size_t(i)]->setText(QString("Nom %1\nDShot req %2")
           .arg(f(motor_command_rpm_[size_t(i)],0)).arg(requested));
       }
     }
   } else {
     if (master_motor_label_) master_motor_label_->setText("DShot bridge telemetry unavailable · commands NOT verified");
     for(int i=0;i<4;++i) if(motor_labels_[size_t(i)])
       motor_labels_[size_t(i)]->setText(QString("Nom %1\nBridge offline")
         .arg(f(motor_command_rpm_[size_t(i)],0)));
   }
 }
 for(size_t i=0;i<items.size();++i){const auto& s=items[i];++collected_;stats_.add(s);
    if (s.orientation_valid) {
        for (int axis = 0; axis < 3; ++axis) {
            response_analyzers_[std::size_t(axis)].add(
                s.ros_time_s, s.attitude_deg[std::size_t(axis)]);
        }
    }
    if(recorder_active_)saveRecord(s);
    onSample(s,i+1==items.size());
 }
 if (recorder_active_ && recording_end_s_ > 0.0 &&
     recording_clock_.elapsed() / 1000.0 >= recording_end_s_) {
     recording_should_stop_ = true;
 }
 if (recorder_active_ && recording_should_stop_) stopRecording();
 if (recorder_active_ && recording_dialog_ && recording_dialog_->isVisible()) {
     const double elapsed_s = recorder_active_ ? recording_clock_.elapsed() / 1000.0 : 0.0;
     double progress_fraction = -1.0;
     QString estimate_text("Estimate: open-ended / continuous capture");
     if (recording_config_.mode == 1 && recording_end_s_ > 0.0) {
         progress_fraction = std::clamp(elapsed_s / recording_end_s_, 0.0, 1.0);
         estimate_text = QString("Estimate: %1 s total · remaining %2 s")
             .arg(f(recording_end_s_,2)).arg(f(std::max(0.0, recording_end_s_ - elapsed_s),2));
     } else if (recording_config_.mode == 2 && recording_end_s_ > 0.0) {
         progress_fraction = std::clamp(elapsed_s / recording_end_s_, 0.0, 1.0);
         estimate_text = QString("Estimate: %1 waveform periods · remaining %2 s")
             .arg(recording_config_.periods).arg(f(std::max(0.0, recording_end_s_ - elapsed_s),2));
     } else if (recording_config_.mode == 3 && recording_config_.peak_count > 0) {
         progress_fraction = std::clamp(double(recording_events_) / double(recording_config_.peak_count), 0.0, 1.0);
         estimate_text = QString("Detected peaks: %1 / %2").arg(recording_events_).arg(recording_config_.peak_count);
     }
     recording_dialog_->updateStatus(
         recorder_active_ ? (elapsed_s < recording_config_.start_delay_s
                                ? "WAITING FOR START DELAY" : "RECORDING") : "IDLE",
         recorded_samples_,
         elapsed_s,
         recording_events_, recording_path_, progress_fraction, estimate_text);
 }
 status_samples_->setText(QString("SAMPLES: %1").arg(collected_));
 if(have_sample_){for(int i=0;i<3;++i){
   rmse_values_[size_t(i)]->setText(QString("%1° / RMSE %2°").arg(f(last_sample_.attitude_deg[i],1)).arg(f(stats_.rmse(i),2)));}
   metric_value_->setText(QString("RMSE %1°  |  MAE %2°  |  PEAK %3°")
   .arg(f(stats_.rmse(monitor_axis_)),f(stats_.mae(monitor_axis_)),f(stats_.peak[monitor_axis_])));
   updateSystem(last_sample_);
   if (ref_tabs_ && ref_tabs_->currentWidget() &&
       ref_tabs_->tabText(ref_tabs_->currentIndex()) == "Response") {
       refreshStepMetrics();
   }
 }
}
void MainWindow::onSample(const Telemetry& s,bool draw){
 last_sample_=s;have_sample_=true;
 if(!draw)return;
 const double t=clock_.elapsed()/1000.0;
 for(int i=0;i<3;++i){
   if(s.orientation_valid){dash_[size_t(i)]->append(0,t,s.setpoint_deg[i]);dash_[size_t(i)]->append(1,t,s.attitude_deg[i]);}
 }
 for(int i=0;i<3;++i){dash_[3]->append(i,t,s.angular_rate_deg_s[i]);dash_[4]->append(i,t,s.accel_m_s2[i]);if(s.magnetometer_valid)dash_[5]->append(i,t,s.mag_uT[i]);}
 for(int i=0;i<3;++i)attitude_values_[size_t(i)]->setText(s.orientation_valid?f(s.attitude_deg[i],2)+" deg":"NO ORIENTATION");
 rate_values_[0]->setText(QString("p %1   q %2   r %3 deg/s").arg(f(s.angular_rate_deg_s[0]),f(s.angular_rate_deg_s[1]),f(s.angular_rate_deg_s[2])));
 rate_values_[1]->setText(QString("ax %1   ay %2   az %3 m/s²").arg(f(s.accel_m_s2[0]),f(s.accel_m_s2[1]),f(s.accel_m_s2[2])));
 rate_values_[2]->setText(QString("Bx %1   By %2   Bz %3 µT").arg(f(s.mag_uT[0]),f(s.mag_uT[1]),f(s.mag_uT[2])));
 view_->setAttitude(s.orientation,s.orientation_valid);
 if(s.orientation_valid){live_->append(0,t,s.setpoint_deg[monitor_axis_]);live_->append(1,t,s.attitude_deg[monitor_axis_]);}
}
void MainWindow::updateSystem(const Telemetry& s){
 system_imu_->setText(QString("IMU  %1").arg(bridge_.imuFresh()?"ONLINE":"STALE"));
 system_mag_->setText(QString("MAG  %1").arg(bridge_.magFresh()?"ONLINE":"STALE"));
 system_baro_->setText(QString("BARO  %1").arg(bridge_.baroFresh()?"ONLINE":"STALE"));
 system_controller_->setText(bridge_.feedbackFresh()?"Controller: "+QString::fromStdString(bridge_.controllerStatus()):"Controller: NO CONFIRMED FEEDBACK");
 system_frame_->setText(QString("IMU frame: %1\nMAG frame: %2\nBARO frame: %3")
  .arg(QString::fromStdString(s.imu_frame),QString::fromStdString(s.mag_frame),QString::fromStdString(s.baro_frame)));
 system_angular_->setText(QString("Gyro: [%1, %2, %3] deg/s").arg(f(s.angular_rate_deg_s[0]),f(s.angular_rate_deg_s[1]),f(s.angular_rate_deg_s[2])));
 system_accel_->setText(QString("Accel: [%1, %2, %3] m/s²").arg(f(s.accel_m_s2[0]),f(s.accel_m_s2[1]),f(s.accel_m_s2[2])));
 system_mag_value_->setText(QString("Mag: [%1, %2, %3] µT").arg(f(s.mag_uT[0]),f(s.mag_uT[1]),f(s.mag_uT[2])));
 system_pressure_->setText(QString("Pressure: %1 hPa").arg(f(s.pressure_hPa,3)));
 system_altitude_->setText(QString("Z relative: %1 m (pressure-derived)").arg(f(s.relative_alt_m,3)));
}
void MainWindow::setTheme(int index){
    if (index < 0 || index >= int(kThemes.size())) {
        return;
    }
    theme_index_ = index;
 QString file=QString(":/theme/%1.qss").arg(QStringList{"tron_ares","matlab","light","dark","tron_evolution","cyberpunk"}[index]);
 QFile qss(file);if(qss.open(QIODevice::ReadOnly|QIODevice::Text))qApp->setStyleSheet(QString::fromUtf8(qss.readAll()));
 const auto& t=kTheme[size_t(index)];
 const QList<QColor> colors={QColor(t.red),QColor(t.blue),QColor(t.teal),QColor("#f8b869")};
 for(auto* plot:dash_)if(plot)plot->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(preview_)preview_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(live_)live_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(view_)view_->setTheme(QColor(t.bg),QColor(t.fg),QColor(t.red),QColor(t.blue));
}
void MainWindow::showRecordingDialog() {
    if (!recording_dialog_) return;
    const auto wave = waveformConfig();
    recording_dialog_->setWaveformInformation(
        QString("Waveform: %1 · f=%2 Hz · duration=%3 s · source=%4")
            .arg(waveform_->currentText())
            .arg(wave.frequency_hz, 0, 'g', 5)
            .arg(wave.duration_s, 0, 'g', 5)
            .arg(manual_mode_ ? "MANUAL" : "AUTOMATIC"));
    recording_dialog_->show();
    recording_dialog_->raise();
    recording_dialog_->activateWindow();
}

void MainWindow::startRecording() {
    if (recorder_active_ || !recording_dialog_) return;
    const RecordingConfig config = recording_dialog_->configuration();
    if (config.directory.isEmpty() || config.prefix.isEmpty()) {
        const QString message = "Specify an output directory and a file prefix.";
        recording_dialog_->showError(message);
        QMessageBox::warning(recording_dialog_, "Data recorder", message);
        return;
    }
    if (config.mode == 2 && manual_mode_) {
        const QString message = "N waveform periods requires AUTOMATIC reference mode.";
        recording_dialog_->showError(message);
        QMessageBox::warning(recording_dialog_, "Data recorder", message);
        return;
    }
    if (config.mode == 2 && waveformConfig().frequency_hz <= 0.0) {
        recording_dialog_->showError("Set a positive waveform frequency for period capture.");
        return;
    }
    QDir dir;
    if (!dir.mkpath(config.directory)) {
        const QString message = "Cannot create acquisition directory: " + config.directory;
        recording_dialog_->showError(message);
        QMessageBox::critical(recording_dialog_, "Data recorder", message);
        return;
    }
    QString safe_prefix = config.prefix;
    safe_prefix.replace(QRegularExpression("[^A-Za-z0-9_.-]"), "_");
    if (safe_prefix == "." || safe_prefix == "..") safe_prefix = "cefiro_run";
    safe_prefix = safe_prefix.left(80);
    recording_path_ = QDir(config.directory).filePath(
        safe_prefix + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz") + ".csv");
    recording_file_.setFileName(recording_path_);
    if (!recording_file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly)) {
        recording_dialog_->showError(recording_file_.errorString());
        QMessageBox::critical(recording_dialog_, "Data recorder", recording_file_.errorString());
        return;
    }
    recording_config_ = config;
    recording_start_ros_s_ = have_sample_ ? last_sample_.ros_time_s : 0.0;
    recording_end_s_ = -1.0;
    if (config.mode == 1) {
        recording_end_s_ = config.start_delay_s + config.duration_s;
    } else if (config.mode == 2) {
        // Frequency is captured at record start to make the duration deterministic.
        recording_end_s_ = config.start_delay_s +
            double(config.periods) / waveformConfig().frequency_hz;
    }
    recorded_samples_ = 0;
    recording_events_ = 0;
    recording_should_stop_ = false;
    last_saved_s_ = -1.e9;
    last_peak_s_ = -1.e9;
    peak_have_1_ = false;
    peak_have_2_ = false;
    recording_stats_.reset();
    recording_stream_.setDevice(&recording_file_);
    QStringList columns{"ros_time_s", "sequence"};
    if (recording_config_.save_attitude) columns << "roll_deg" << "pitch_deg" << "yaw_deg";
    if (recording_config_.save_reference) columns << "ref_roll_deg" << "ref_pitch_deg" << "ref_yaw_deg";
    if (recording_config_.save_error) columns << "error_roll_deg" << "error_pitch_deg" << "error_yaw_deg";
    if (recording_config_.save_rates) columns << "p_deg_s" << "q_deg_s" << "r_deg_s";
    if (recording_config_.save_accel) columns << "ax_m_s2" << "ay_m_s2" << "az_m_s2";
    if (recording_config_.save_mag) columns << "Bx_uT" << "By_uT" << "Bz_uT";
    if (recording_config_.save_baro) columns << "pressure_hPa" << "relative_alt_m";
    if (recording_config_.save_validity) columns << "orientation_valid" << "mag_valid" << "baro_valid";
    if (recording_config_.save_strategy_source) columns << "strategy" << "source";
    recording_stream_ << columns.join(',') << "\n";
    recording_clock_.start();
    recorder_active_ = true;
    record_btn_->setText("RECORDER ●");
    status_rec_->setText("REC: ACTIVE");
    recording_dialog_->setRecordingState(true);
    recording_dialog_->showError(QString());
    recording_dialog_->updateStatus("RECORDING", 0, 0.0, 0, recording_path_, -1.0, "Estimate: initializing...");

    if (config.metadata) {
        const QString filename = QFileInfo(recording_path_).absolutePath() + "/" +
                                 QFileInfo(recording_path_).completeBaseName() + "_metadata.yaml";
        QFile metadata(filename);
        if (metadata.open(QIODevice::WriteOnly | QIODevice::Text)) {
            const auto quoteYaml = [](const QString& value) {
                const QByteArray json = QJsonDocument(QJsonArray{value}).toJson(QJsonDocument::Compact);
                return QString::fromUtf8(json).mid(1, json.size() - 2);
            };
            QTextStream yaml(&metadata);
            yaml << "cefiro_experiment:\n";
            yaml << "  created_at: " << quoteYaml(QDateTime::currentDateTime().toString(Qt::ISODate)) << '\n';
            yaml << "  data_file: " << quoteYaml(recording_path_) << '\n';
            yaml << "  notes: " << quoteYaml(config.notes) << '\n';
            yaml << "  strategy: " << quoteYaml(controller_->currentText()) << '\n';
            yaml << "  source: " << quoteYaml(manual_mode_ ? "MANUAL" : "AUTOMATIC") << '\n';
            yaml << "  capture_mode: " << quoteYaml(QStringList{
                "Continuous", "Time window", "N waveform periods", "Peak count"}[config.mode]) << '\n';
            yaml << "  start_delay_s: " << config.start_delay_s << '\n';
            yaml << "  save_interval_ms: " << config.save_interval_ms << '\n';
            yaml << "  duration_s: " << config.duration_s << '\n';
            yaml << "  requested_periods: " << config.periods << '\n';
            yaml << "  waveform_frequency_hz: " << waveformConfig().frequency_hz << '\n';
            yaml << "  peak_axis: " << quoteYaml(kAxisNames[config.peak_axis]) << '\n';
            yaml << "  peak_threshold_deg: " << config.peak_threshold_deg << '\n';
            yaml << "  peak_target: " << config.peak_count << '\n';
            yaml << "  peak_min_separation_s: " << config.peak_separation_s << '\n';
            yaml << "  controller_acknowledgement: "
                 << (bridge_.feedbackFresh() ? "available" : "not_confirmed") << '\n';
            yaml << "  time_unit: " << quoteYaml(QStringList{"ms","s","min"}[config.time_unit]) << '\n';
            yaml << "  saved_groups:\n";
            yaml << "    attitude: " << (config.save_attitude ? "true" : "false") << '\n';
            yaml << "    reference: " << (config.save_reference ? "true" : "false") << '\n';
            yaml << "    error: " << (config.save_error ? "true" : "false") << '\n';
            yaml << "    rates: " << (config.save_rates ? "true" : "false") << '\n';
            yaml << "    acceleration: " << (config.save_accel ? "true" : "false") << '\n';
            yaml << "    magnetic_field: " << (config.save_mag ? "true" : "false") << '\n';
            yaml << "    barometer: " << (config.save_baro ? "true" : "false") << '\n';
            yaml << "    validity_flags: " << (config.save_validity ? "true" : "false") << '\n';
            yaml << "    strategy_source: " << (config.save_strategy_source ? "true" : "false") << '\n';
        } else {
            recording_dialog_->showError("CSV started, but metadata could not be written: " + metadata.errorString());
        }
    }
}

void MainWindow::stopRecording() {
    if (!recorder_active_) return;
    recorder_active_ = false;
    recording_should_stop_ = false;
    recording_stream_.flush();
    recording_file_.close();
    record_btn_->setText("DATA RECORDER");
    status_rec_->setText("REC: IDLE");
    if (recording_dialog_) {
        recording_dialog_->setRecordingState(false);
        recording_dialog_->updateStatus("COMPLETE", recorded_samples_,
            recording_clock_.elapsed() / 1000.0, recording_events_, recording_path_, 1.0, "Estimate: completed");
    }
    if (recording_config_.metadata) exportSummary();
}

void MainWindow::saveRecord(const Telemetry& sample) {
    if (!recorder_active_) return;
    const double elapsed = recording_clock_.elapsed() / 1000.0;
    if (elapsed < recording_config_.start_delay_s) return;
    // Use the measurement timestamp for interval/peak filters. A UI tick may
    // contain many IMU samples, all sharing the same wall-clock processing time.
    const double sample_time = std::isfinite(sample.ros_time_s) && sample.ros_time_s > 0.0
        ? sample.ros_time_s : elapsed;
    if (sample_time < last_saved_s_) last_saved_s_ = -1.e9;
    if (sample_time < last_peak_s_) last_peak_s_ = -1.e9;

    // Peak acquisition uses local maxima rather than simple threshold crossings.
    if (recording_config_.mode == 3 && sample.orientation_valid) {
        double value = sample.attitude_deg[std::size_t(recording_config_.peak_axis)];
        if (recording_config_.absolute_peaks) value = std::abs(value);
        if (peak_have_1_ && peak_have_2_ && peak_prev1_ > peak_prev2_ &&
            peak_prev1_ >= value && peak_prev1_ >= recording_config_.peak_threshold_deg &&
            sample_time - last_peak_s_ >= recording_config_.peak_separation_s) {
            ++recording_events_;
            last_peak_s_ = sample_time;
            if (recording_events_ >= recording_config_.peak_count) recording_should_stop_ = true;
        }
        peak_prev2_ = peak_prev1_;
        peak_prev1_ = value;
        peak_have_2_ = peak_have_1_;
        peak_have_1_ = true;
    }

    if (recording_config_.save_interval_ms > 0 &&
        (sample_time - last_saved_s_) * 1000.0 < recording_config_.save_interval_ms) return;
    last_saved_s_ = sample_time;
    recording_stats_.add(sample);
    auto& out = recording_stream_;
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(7);
    out << sample.ros_time_s << ',' << sample.sequence;
    if (recording_config_.save_attitude) {
        for (double v : sample.attitude_deg) out << ',' << v;
    }
    if (recording_config_.save_reference) {
        for (double v : sample.setpoint_deg) out << ',' << v;
    }
    if (recording_config_.save_error) {
        for (int i = 0; i < 3; ++i) {
            out << ',' << (sample.orientation_valid ? angleErrorDeg(sample.setpoint_deg[i], sample.attitude_deg[i]) : 0.0);
        }
    }
    if (recording_config_.save_rates) {
        for (double v : sample.angular_rate_deg_s) out << ',' << v;
    }
    if (recording_config_.save_accel) {
        for (double v : sample.accel_m_s2) out << ',' << v;
    }
    if (recording_config_.save_mag) {
        for (double v : sample.mag_uT) out << ',' << v;
    }
    if (recording_config_.save_baro) {
        out << ',' << sample.pressure_hPa << ',' << sample.relative_alt_m;
    }
    if (recording_config_.save_validity) {
        out << ',' << int(sample.orientation_valid)
            << ',' << int(sample.magnetometer_valid)
            << ',' << int(sample.barometer_valid);
    }
    if (recording_config_.save_strategy_source) {
        out << ',' << controller_->currentText()
            << ',' << (manual_mode_ ? "MANUAL" : "AUTOMATIC");
    }
    out << "\n";
    ++recorded_samples_;
    if (recorded_samples_ % 128 == 0) out.flush();
    if (out.status() != QTextStream::Ok) {
        recording_should_stop_ = true;
        if (recording_dialog_) recording_dialog_->showError("CSV write failed; stopping acquisition.");
    }
}

void MainWindow::exportSummary() {
    const QString filename = QFileInfo(recording_path_).absolutePath() + "/" +
                             QFileInfo(recording_path_).completeBaseName() + "_summary.txt";
    QFile output(filename);
    if (!output.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return;
    QTextStream out(&output);
    out << "CEFIRO ATTITUDE EXPERIMENT SUMMARY\n";
    out << "CSV: " << recording_path_ << '\n';
    out << "Notes: " << recording_config_.notes << '\n';
    out << "Controller selection: " << controller_->currentText() << '\n';
    out << "Source: " << (manual_mode_ ? "MANUAL" : "AUTOMATIC") << '\n';
    out << "Samples written to disk: " << recorded_samples_ << '\n';
    out << "Samples with valid orientation: " << recording_stats_.count << '\n';
    out << "Capture time [wall seconds]: " << recording_clock_.elapsed() / 1000.0 << '\n';
    out << "Detected local maxima: " << recording_events_ << '\n';
    out << "Note: controller actuation is not guaranteed without an external controller.\n";
    for (int i = 0; i < 3; ++i) {
        out << kAxisNames[i] << " RMSE=" << f(recording_stats_.rmse(i))
            << "deg MAE=" << f(recording_stats_.mae(i))
            << "deg Peak=" << f(recording_stats_.peak[i]) << "deg\n";
    }
    out << "\nLATEST STEP RESPONSES DURING THIS RECORDING\n";
    const auto metricValue = [](double v, int digits=3) {
        return std::isfinite(v) ? QString::number(v,'f',digits) : "N/A";
    };
    for (int axis=0; axis<3; ++axis) {
        const auto& analyzer = response_analyzers_[std::size_t(axis)];
        const auto m = analyzer.metrics();
        out << kAxisNames[axis] << ": ";
        if (!m.valid || analyzer.startTime() < recording_start_ros_s_) {
            out << "no step recorded in this acquisition\n";
            continue;
        }
        out << "step=" << metricValue(m.amplitude_deg) << " deg"
            << ", peak=" << metricValue(m.peak_deg) << " deg"
            << ", peak_time=" << metricValue(m.peak_time_s) << " s"
            << ", overshoot=" << metricValue(m.overshoot_percent) << " %"
            << ", dead_time_5pct=" << metricValue(m.dead_time_s) << " s"
            << ", rise_10_90=" << metricValue(m.rise_time_s) << " s"
            << ", settling_2pct=" << metricValue(m.settling_time_s) << " s"
            << ", ringing_crossings=" << m.ringing_crossings
            << ", steady_error=" << metricValue(m.steady_error_deg) << " deg"
            << ", rmse=" << metricValue(m.rmse_deg) << " deg"
            << ", mae=" << metricValue(m.mae_deg) << " deg"
            << ", peak_error=" << metricValue(m.peak_error_deg) << " deg\n";
    }
    out << "Step metrics are estimated from the last step commanded on each axis.\n";
}

void MainWindow::refreshStepMetrics() {
    if (!response_metrics_table_) return;
    for (int axis=0; axis<3; ++axis) {
        const auto m = response_analyzers_[std::size_t(axis)].metrics();
        auto set = [&](int row, double value, int precision=2) {
            auto* item = response_metrics_table_->item(row,axis);
            if (!item) return;
            item->setText(m.valid && std::isfinite(value)
                ? QString::number(value,'f',precision) : "—");
        };
        set(0,m.amplitude_deg);
        set(1,m.peak_deg);
        set(2,m.peak_time_s,3);
        set(3,m.overshoot_percent);
        set(4,m.dead_time_s,3);
        set(5,m.rise_time_s,3);
        set(6,m.settling_time_s,3);
        auto* crossings = response_metrics_table_->item(7,axis);
        if (crossings) crossings->setText(m.valid ? QString::number(m.ringing_crossings) : "—");
        set(8,m.steady_error_deg);
        set(9,m.rmse_deg);
        set(10,m.mae_deg);
        set(11,m.peak_error_deg);
    }
}

void MainWindow::sendGains(){
 std::vector<double> pid,lqr;pid.reserve(9);lqr.reserve(18);
 for(int r=0;r<3;++r)for(int c=0;c<3;++c){
    bool valid=false;auto* item=pid_gains_->item(r,c);
    const double v=item?item->text().toDouble(&valid):0.0;
    if(!valid||!std::isfinite(v)||std::abs(v)>1.e6){QMessageBox::warning(this,"Invalid PID matrix","Use finite numeric gains within ±1e6.");return;}
    pid.push_back(v);
 }
 for(int r=0;r<3;++r)for(int c=0;c<6;++c){
    bool valid=false;auto* item=lqr_gains_->item(r,c);
    const double v=item?item->text().toDouble(&valid):0.0;
    if(!valid||!std::isfinite(v)||std::abs(v)>1.e6){QMessageBox::warning(this,"Invalid LQR matrix","Use finite numeric gains within ±1e6.");return;}
    lqr.push_back(v);
 }
 bridge_.publishGains(pid,lqr);
 applied_value_->setText("Controller gain request published · NOT confirmed applied");
}
}
