#include "cefiro_gui/main_window.hpp"
#include "cefiro_gui/scope_plot.hpp"
#include "cefiro_gui/attitude_view.hpp"
#include "cefiro_gui/attitude_command_pad.hpp"
#include "cefiro_gui/ares_chrome.hpp"
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
#include <QEvent>
#include <QtGlobal>
#include <QGridLayout>
#include <QGraphicsDropShadowEffect>
#include <cstdio>
#include <QFormLayout>
#include <QFrame>
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
 {"#080307","#fbe4e2","#48151f","#ff304c","#ff9e79","#ffd0bd"},
 {"#fafafa","#1a242e","#c7d1dd","#0072bd","#d95319","#77ac30"},
 {"#ffffff","#0f172a","#cbd5e1","#3b82f6","#10b981","#8b5cf6"},
 {"#141b2e","#e2e8f0","#334155","#60a5fa","#34d399","#a78bfa"},
 {"#000f1a","#d0f0ff","#135071","#00e6ff","#b084ff","#00ff99"},
 {"#0b0b18","#eaf2ff","#26325b","#ff2bd6","#00f5ff","#ccff00"}}};
QString f(double v,int places=2){return QString::number(std::abs(v)<1.e-7?0.:v,'f',places);}
void startupTrace(const char* phase) {
    if(qEnvironmentVariableIsSet("CEFIRO_GUI_TRACE_STARTUP")) {
        std::fprintf(stderr,"[CEFIRO startup] %s\n",phase);
        std::fflush(stderr);
    }
}
QFrame* hline(QWidget* p){auto* l=new QFrame(p);l->setFrameShape(QFrame::HLine);l->setObjectName("hudSeparator");return l;}
}
QDoubleSpinBox* MainWindow::spin(double min,double max,double value,double step,int decimals){auto* s=new QDoubleSpinBox;s->setRange(min,max);s->setDecimals(decimals);s->setSingleStep(step);s->setValue(value);s->setKeyboardTracking(false);return s;}
QLabel* MainWindow::label(const QString& text,const QString& id){
 auto* l=new QLabel(text);
 if(!id.isEmpty())l->setObjectName(id);
 if(id=="hint"||id=="pageSubtitle"||id=="metricValue"||id=="panelTitle"){
     l->setWordWrap(true);
     l->setMinimumWidth(0);
 }
 return l;
}
QWidget* MainWindow::contentPage(QVBoxLayout*& layout,QWidget* parent){
 auto* w=new AresPage(parent);w->setObjectName("aresPage");layout=new QVBoxLayout(w);layout->setContentsMargins(12,9,12,12);layout->setSpacing(8);return w;
}
QWidget* MainWindow::titledPanel(const QString& title,QVBoxLayout*& inner,QWidget* parent){
 auto* panel=new AresPanel(parent);
 panel->setObjectName("panel");
 inner=new QVBoxLayout(panel);
 inner->setContentsMargins(13,12,13,12);
 inner->setSpacing(7);
 auto* heading=new QHBoxLayout;
 heading->setContentsMargins(0,0,0,0);
 heading->setSpacing(8);
 heading->addWidget(label("▰","panelGlyph"));
 heading->addWidget(label(title,"panelTitle"),1);
 heading->addWidget(label("// CEFIRO","panelTag"));
 inner->addLayout(heading);
 return panel;
}
MainWindow::MainWindow(QWidget* parent):QMainWindow(parent){
 setWindowTitle("CEFIRO GUI v2.4 | TRON ARES CONTROL");
 startupTrace("MainWindow constructor entered");
 setMinimumSize(610,480);
 resize(1530,910);
 clock_.start();publish_clock_.start();
 stack_=new QStackedWidget(this);setCentralWidget(stack_);
 buildToolbar();startupTrace("Toolbar built");
 buildDashboard();startupTrace("Dashboard built");
 buildControl();startupTrace("Control page built");
 buildSystem();startupTrace("System page built");
 recording_dialog_ = new RecordingDialog(this);
 recording_config_ = recording_dialog_->configuration();
 startupTrace("Recorder built");
 connectUi();startupTrace("UI signals connected");
 setTheme(0);startupTrace("Theme applied");
 updateCompactChrome(width());
 // Never move QGridLayout items from a synchronous Resize event (Qt can re-enter
 // geometry negotiation while the layout is still being modified).
 responsive_timer_=new QTimer(this);
 responsive_timer_->setSingleShot(true);
 responsive_timer_->setInterval(35);
 connect(responsive_timer_,&QTimer::timeout,this,[this]{applyResponsiveLayout();});
 ui_ready_=true;
 scheduleResponsiveLayout();
 startupTrace("Responsive scheduler initialized");
 nav_->setCurrentIndex(0);
 stack_->setCurrentIndex(0);
 for (auto* plot : dash_) if (plot) plot->setActive(true);
 if(altitude_plot_) altitude_plot_->setActive(true);
 if(rpm_target_plot_)rpm_target_plot_->setActive(true);
 if(rpm_feedback_plot_)rpm_feedback_plot_->setActive(true);
 if (preview_) preview_->setActive(false);
 if (live_) live_->setActive(false);
 startupTrace("Starting ROS executor");
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
 status_sim_connection_=label("", "connectionBadge");
 status_real_connection_=label("", "connectionBadge");
 status_sim_connection_->setMinimumWidth(98);
 status_real_connection_->setMinimumWidth(104);
 statusBar()->addWidget(status_sim_connection_);
 statusBar()->addWidget(status_real_connection_);
 setConnectionBadge(status_sim_connection_,"SIM",false);
 setConnectionBadge(status_real_connection_,"REAL",false);
 for(auto* l:{status_ros_,status_rec_,status_control_,status_samples_}){statusBar()->addPermanentWidget(l);statusBar()->addPermanentWidget(label(" | ","statusDivider"));}
 startupTrace("MainWindow constructor completed");
}
MainWindow::~MainWindow(){shutdown();}
void MainWindow::shutdown(){if(shutting_down_)return;shutting_down_=true;
    ui_ready_=false;
    if(responsive_timer_)responsive_timer_->stop();
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
 bar->setIconSize(QSize(16,16));
 bar->setMinimumHeight(48);
 bar->setMaximumHeight(48);
 brand_label_=label("▣ CEFIRO  //  ARES","aresBrand");
 brand_label_->setMinimumHeight(28);
 brand_label_->setMaximumHeight(28);
 bar->addWidget(brand_label_);
 nav_=new QTabBar(bar);nav_->setObjectName("topTabs");nav_->setExpanding(false);nav_->setDrawBase(false);
 nav_->setMinimumHeight(32);
 nav_->setMaximumHeight(32);
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
    if (altitude_plot_) altitude_plot_->setActive(i == 0);
    if (rpm_target_plot_) rpm_target_plot_->setActive(i == 0);
    if (rpm_feedback_plot_) rpm_feedback_plot_->setActive(i == 0);
 });
 bar->addWidget(nav_);
 auto* spacer=new QWidget(bar);
 spacer->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
 bar->addWidget(spacer);
 control_target_label_=label("CONTROL TARGET  ","eyebrow");
 bar->addWidget(control_target_label_);
 plant_selector_=new QComboBox(bar);
 plant_selector_->setObjectName("plantSelector");
 plant_selector_->addItem("SIMULATION",false);
 plant_selector_->addItem("REAL HARDWARE",true);
 plant_selector_->setMinimumWidth(130);
 plant_selector_->setFixedHeight(30);
 plant_selector_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
 plant_selector_->setToolTip("Select the command destination. Both plants are monitored independently. Stop commands before switching.");
 bar->addWidget(plant_selector_);
 connect(plant_selector_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int index){
     if(command_active_)return;
     bridge_.selectReal(index==1);
     resetSelectedFeedback();
     refreshReferenceLimits();
     setTheme(theme_index_);
 });
 theme_label_=label("THEME  ","eyebrow");bar->addWidget(theme_label_);
 theme_combo_=new QComboBox(bar);theme_combo_->addItems(kThemes);theme_combo_->setMinimumWidth(120);theme_combo_->setFixedHeight(30);theme_combo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);bar->addWidget(theme_combo_);
 connect(theme_combo_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int i){setTheme(i);});addToolBar(Qt::TopToolBarArea,bar);
}
void MainWindow::buildDashboard(){
 QVBoxLayout* body=nullptr;auto* page=contentPage(body,stack_);stack_->addWidget(page);
 auto* head=new QHBoxLayout;head->addWidget(label("CEFIRO / LIVE TELEMETRY","pageTitle"));head->addStretch();
 clear_btn_=new QPushButton("CLEAR CHARTS");clear_btn_->setObjectName("secondary");head->addWidget(clear_btn_);body->addLayout(head);
 body->addWidget(label("Attitude, angular rates and FMU measurements  ·  cursor hover / zoom controls on each scope","pageSubtitle"));
 dashboard_scroll_=new QScrollArea(page);
 dashboard_scroll_->setObjectName("dashboardScroll");
 dashboard_scroll_->setWidgetResizable(true);
 dashboard_scroll_->setFrameShape(QFrame::NoFrame);
 dashboard_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
 body->addWidget(dashboard_scroll_,1);
 auto* container=new QWidget(dashboard_scroll_);
 container->setObjectName("dashboardCanvas");
 dashboard_grid_=new QGridLayout(container);
 dashboard_grid_->setSpacing(12);
 dashboard_grid_->setContentsMargins(2,4,2,6);
 auto* grid=dashboard_grid_;
 dashboard_scroll_->setWidget(container);
 dashboard_scroll_->viewport()->installEventFilter(this);
 const QStringList titles={"ROLL / φ","PITCH / θ","YAW / ψ","ANGULAR RATES / p q r","LINEAR ACCELERATION / axyz","MAGNETIC FIELD / Bxyz","BAROMETER / PRESSURE","MAGNETOMETER / MAGNITUDE","DSHOT / HOST REQUEST"};
 const QStringList units={"deg","deg","deg","deg/s","m/s²","µT","hPa","µT","DShot"};
 for(int i=0;i<9;++i){QVBoxLayout* inner=nullptr;auto* panel=titledPanel(titles[i],inner,page);
    auto* metric=label(i<3?"-- deg":"--","metricValue");
    if(i<3)attitude_values_[size_t(i)]=metric;
    else if(i==3)rate_values_[0]=metric;
    else if(i==4)rate_values_[1]=metric;
    else if(i==5)rate_values_[2]=metric;
    else if(i==6)pressure_value_=metric;
    else if(i==7)magnitude_value_=metric;
    else if(i==8)dshot_value_=metric;
    inner->addWidget(metric);
    QStringList keys=i<3?QStringList{"SIM REF","SIM","REAL REF","REAL"}:
       i==3?QStringList{"SIM p","SIM q","SIM r","REAL p","REAL q","REAL r"}:
       i==4?QStringList{"SIM ax","SIM ay","SIM az","REAL ax","REAL ay","REAL az"}:
       i==5?QStringList{"SIM Bx","SIM By","SIM Bz","REAL Bx","REAL By","REAL Bz"}:
       i==6?QStringList{"SIM hPa","REAL hPa"}:
       i==7?QStringList{"SIM |B|","REAL |B|"}:
       QStringList{"SIM M1","SIM M2","SIM M3","SIM M4",
                   "REAL M1","REAL M2","REAL M3","REAL M4"};
    dash_[size_t(i)]=new ScopePlot("",units[i],keys,panel);dash_[size_t(i)]->setMinimumHeight(235);
    if(i<3){
       dash_[size_t(i)]->setDashed(0);
       dash_[size_t(i)]->setDashed(2);
       dash_[size_t(i)]->setYRange(i==1?-95:-185,i==1?95:185);
     }
    if(i==3)dash_[size_t(i)]->setYRange(-30,30);
    if(i==4)dash_[size_t(i)]->setYRange(-15,15);
    if(i==5)dash_[size_t(i)]->setYRange(-80,80);
     if(i==6)dash_[size_t(i)]->setYRange(700,1100);
     if(i==7)dash_[size_t(i)]->setYRange(0,120);
     if(i==3||i==4||i==5)for(int j=3;j<6;++j)dash_[size_t(i)]->setDashed(j);
 if(i==8){
     dash_[size_t(i)]->setYRange(0,2047);
     for(int motor=4;motor<8;++motor)dash_[size_t(i)]->setDashed(motor);
 }
    // Every dashboard scope follows only the series that actually have data.
    // Minimum spans prevent a single stable sensor from collapsing the Y axis.
    const double minimum_span[]={4.0,4.0,5.0,8.0,3.0,5.0,2.0,3.0,100.0};
    dash_[size_t(i)]->setAutoFitY(true,minimum_span[i]);
    inner->addWidget(dash_[size_t(i)],1);
    panel->setMinimumHeight(308);
    // Each dashboard panel occupies exactly one cell in a 2-column grid.
    // Pressure and altitude are independent panels with the same height.
    // The last two panels move to row 5 after the altitude insertion.
    const int row=i<=6 ? i/2 : 4;
    const int column=i<=6 ? i%2 : i-7;
    grid->addWidget(panel,row,column);
    dashboard_panels_.append(panel);
 }
 // Same structure as all other scopes: panel title, live metric and one plot.
 QVBoxLayout* altitude_inner=nullptr;
 auto* altitude_panel=titledPanel("RELATIVE ALTITUDE",altitude_inner,container);
 altitude_value_=label("-- m","metricValue");
 altitude_inner->addWidget(altitude_value_);
 altitude_plot_=new ScopePlot("","m",QStringList{"SIM altitude","REAL altitude"},altitude_panel);
 altitude_plot_->setMinimumHeight(235);
 altitude_plot_->setYRange(-20,20);
 altitude_plot_->setAutoFitY(true,2.0);
 altitude_plot_->setDashed(1);
 altitude_inner->addWidget(altitude_plot_,1);
 altitude_panel->setMinimumHeight(308);
 grid->addWidget(altitude_panel,3,1);
 dashboard_panels_.append(altitude_panel);
 // Requested RPM and measured RPM are separate scopes: never imply a DShot
 // request is measured rotational speed on physical hardware.
 const QStringList motor_keys={"SIM M1","SIM M2","SIM M3","SIM M4",
                               "REAL M1","REAL M2","REAL M3","REAL M4"};
 auto add_rpm_panel=[&](const QString& title,ScopePlot*& plot,QLabel*& metric,int column){
     QVBoxLayout* inner=nullptr;
     auto* panel=titledPanel(title,inner,container);
     metric=label("-- RPM","metricValue");
     inner->addWidget(metric);
     plot=new ScopePlot("","RPM",motor_keys,panel);
     plot->setMinimumHeight(235);
     plot->setYRange(0,12000);
     plot->setAutoFitY(true,300.0);
     for(int motor=4;motor<8;++motor)plot->setDashed(motor);
     inner->addWidget(plot,1);
     panel->setMinimumHeight(308);
     grid->addWidget(panel,5,column);
     dashboard_panels_.append(panel);
 };
 add_rpm_panel("RPM / REQUESTED REFERENCE",rpm_target_plot_,rpm_target_value_,0);
 add_rpm_panel("RPM / MEASURED FEEDBACK",rpm_feedback_plot_,rpm_feedback_value_,1);
 // Reading order stays consistent as the panel grid changes its columns.
 // Pressure is immediately followed by relative altitude.
 if(dashboard_panels_.size()==12){
   const QVector<int> order{0,1,2,3,4,5,6,9,7,8,10,11};
   const auto original=dashboard_panels_;
   for(int i=0;i<order.size();++i)dashboard_panels_[i]=original[order[i]];
 }
 // Initial placement is valid; defer responsive relayout until after show().
 connect(clear_btn_,&QPushButton::clicked,this,[this]{
    for(auto* p:dash_)p->clear();
    if(altitude_plot_)altitude_plot_->clear();
    if(rpm_target_plot_)rpm_target_plot_->clear();
    if(rpm_feedback_plot_)rpm_feedback_plot_->clear();
    for(auto& source:last_motor_group_s_)source.fill(-1.0);
    last_motor_telemetry_s_.fill(-1.0);
    if(dshot_value_)dshot_value_->setText("-- DShot");
    if(rpm_target_value_)rpm_target_value_->setText("-- RPM");
    if(rpm_feedback_value_)rpm_feedback_value_->setText("-- RPM");
    if(live_)live_->clear();
    for(auto& source:live_refs_)for(auto& axis:source)axis.clear();
    for(auto& source:live_meas_)for(auto& axis:source)axis.clear();
    stats_.reset();
 });
}
void MainWindow::buildControl() {
    QVBoxLayout* page_layout=nullptr;
    auto* page=contentPage(page_layout,stack_);
    stack_->addWidget(page);
    page_layout->setContentsMargins(0,0,0,0);
    control_scroll_=new QScrollArea(page);
    control_scroll_->setObjectName("controlScroll");
    control_scroll_->setWidgetResizable(true);
    control_scroll_->setFrameShape(QFrame::NoFrame);
    control_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    page_layout->addWidget(control_scroll_,1);
    auto* control_content=new QWidget(control_scroll_);
    control_content->setObjectName("controlCanvas");
    auto* body=new QVBoxLayout(control_content);
    body->setContentsMargins(10,9,10,10);
    body->setSpacing(7);
    control_scroll_->setWidget(control_content);
    control_scroll_->viewport()->installEventFilter(this);

    auto* header = new QHBoxLayout;
    auto* title = new QVBoxLayout;
    title->setSpacing(0);
    title->addWidget(label("CEFIRO / ATTITUDE OPERATIONS", "eyebrow"));
    title->addWidget(label("ATTITUDE CONTROL & EXPERIMENTS", "pageTitle"));
    title->addWidget(label("CEFIRO GUI v2.4  //  RESPONSIVE TRON ARES CONSOLE  ·  ATTITUDE COMMAND & ANALYSIS", "pageSubtitle"));
    header->addLayout(title);
    header->addStretch();
    record_btn_ = new QPushButton("DATA RECORDER", page);
    record_btn_->setObjectName("recordButton");
    header->addWidget(record_btn_);
    body->addLayout(header);

    // Independent columns: the attitude view and response scope get 64% width,
    // and the reference panel never forces a third full-height grid row.
    control_columns_=new QGridLayout;
    auto* columns=control_columns_;
    columns->setSpacing(10);
    body->addLayout(columns,1);
    auto* left_widget = new QWidget(page);
    auto* left = new QVBoxLayout(left_widget);
    left->setContentsMargins(0, 0, 0, 0);
    left->setSpacing(7);
    left_widget->setMinimumWidth(0);
    left_widget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
    left_widget->installEventFilter(this);
    auto* right_widget = new QWidget(page);
    auto* right = new QVBoxLayout(right_widget);
    right->setContentsMargins(0, 0, 0, 0);
    right->setSpacing(7);
    right_widget->setMinimumWidth(0);
    right_widget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
    control_left_=left_widget;
    control_right_=right_widget;
    columns->addWidget(left_widget,0,0);
    columns->addWidget(right_widget,0,1);
    columns->setColumnStretch(0,38);
    columns->setColumnStretch(1,62);

    QVBoxLayout* arch = nullptr;
    auto* architecture = titledPanel("CONTROL ARCHITECTURE", arch, left_widget);
    arch->setSpacing(4);
    auto* ctrl = new QHBoxLayout;
    ctrl->addWidget(label("STRATEGY", "eyebrow"));
    controller_ = new QComboBox(architecture);
    controller_->addItems({"OPEN LOOP", "PID", "LQR", "SWING-UP + LQR (SIM)", "CUSTOM"});
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
    gimbal_layout->addWidget(label("BALANCED ATTITUDE REFERENCE  /  DRAG THE COLORED RINGS", "eyebrow"));
    reference_gate_label_ = label("LOCKED · Waiting for controller state BALANCE: CAPTURED", "hint");
    reference_gate_label_->setWordWrap(true);
    gimbal_layout->addWidget(reference_gate_label_);
    command_pad_ = new AttitudeCommandPad(gimbal_tab);
    command_pad_->setMinimumHeight(175);
    gimbal_layout->addWidget(command_pad_, 1);
    auto* auto_fields = new QGridLayout;
    auto_fields->setHorizontalSpacing(4);
    auto_fields->setVerticalSpacing(2);
    for (int i=0; i<3; ++i) {
        const double limit = reference_limits_deg_[std::size_t(i)];
        auto_fields->addWidget(label(kAxisNames[i], "eyebrow"), 0, i);
        manual_[std::size_t(i)] = spin(-limit, limit, 0., 1., 2);
        auto_fields->addWidget(manual_[std::size_t(i)], 1, i);
        manual_labels_[std::size_t(i)] = label("0.00°", "hint");
        auto_fields->addWidget(manual_labels_[std::size_t(i)], 2, i);
    }
    gimbal_layout->addLayout(auto_fields);
    gimbal_layout->addWidget(label("MODEL-LIMITED SIM REFERENCES · yaw +/-180° spans 360° of headings (shortest turn) · 1° wheel", "hint"));
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
    trajectory_fields_grid_=new QGridLayout;
    auto* wavefields=trajectory_fields_grid_;
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
        auto* field_label=label(fieldnames[i],"eyebrow");
        wavefields->addWidget(field_label,row,col);
        wavefields->addWidget(*ptrs[std::size_t(i)],row,col+1);
        trajectory_labels_.append(field_label);
        trajectory_edits_.append(*ptrs[std::size_t(i)]);
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
    manual_layout->setSpacing(3);
    manual_layout->addWidget(label("DIRECT MOTOR COMMAND  /  FLEXIBLE MOTOR GROUP CONTROL", "eyebrow"));
    auto* motor_hint = label("NOMINAL RPM REQUESTS are mapped open-loop to DShot, NOT measured RPM. Apply the master command only to the motors you select below (2, 3 or 4 at a time), or leave it disabled to trim each motor independently. Direction is determined by physical phases.", "hint");
    motor_hint->setWordWrap(true);
    manual_layout->addWidget(motor_hint);

    auto* master_panel = new QFrame(manual_page);
    master_panel->setObjectName("panel");
    master_control_grid_=new QGridLayout(master_panel);
    auto* master_layout=master_control_grid_;
    master_layout->setContentsMargins(7, 7, 7, 7);
    master_layout->setHorizontalSpacing(6);
    master_layout->setVerticalSpacing(3);
    master_title_=label("MASTER COMMAND / SELECTED MOTORS","eyebrow");
    master_layout->addWidget(master_title_,0,0,1,5);
    link_all_motors_ = new QCheckBox("Apply master to selected motors", master_panel);
    link_all_motors_->setChecked(true);
    master_layout->addWidget(link_all_motors_, 1, 0, 1, 2);
    master_rpm_label_=label("MASTER NOMINAL RPM", "eyebrow");
    master_layout->addWidget(master_rpm_label_,1,2);
    master_motor_speed_ = spin(0.0, 12000.0, 0.0, 50.0, 0);
    master_layout->addWidget(master_motor_speed_, 1, 3);
    master_motor_direction_ = new QComboBox(master_panel);
    master_motor_direction_->addItems({"PHASES FIXED"});
    master_motor_direction_->setToolTip("Direction determined by physical motor phases.");
    master_motor_direction_->setEnabled(false);
    master_layout->addWidget(master_motor_direction_, 1, 4);
    master_select_widget_=new QWidget(master_panel);
    auto* select_row=new QHBoxLayout(master_select_widget_);
    select_row->setContentsMargins(0,0,0,0);
    select_row->setSpacing(5);
    master_select_label_=label("SYNC TARGETS", "eyebrow");
    select_row->addWidget(master_select_label_);
    for(int i=0;i<4;++i){
        motor_master_select_[size_t(i)]=new QCheckBox(QString("M%1").arg(i+1), master_select_widget_);
        motor_master_select_[size_t(i)]->setChecked(true);
        select_row->addWidget(motor_master_select_[size_t(i)]);
    }
    select_row->addStretch(1);
    master_select_all_btn_=new QPushButton("ALL", master_select_widget_);
    master_select_none_btn_=new QPushButton("NONE", master_select_widget_);
    master_select_all_btn_->setObjectName("secondary");
    master_select_none_btn_->setObjectName("secondary");
    master_select_all_btn_->setMaximumWidth(54);
    master_select_none_btn_->setMaximumWidth(62);
    select_row->addWidget(master_select_all_btn_);
    select_row->addWidget(master_select_none_btn_);
    master_layout->addWidget(master_select_widget_, 2, 0, 1, 5);
    master_motor_slider_ = new QSlider(Qt::Horizontal, master_panel);
    master_motor_slider_->setRange(0, 12000);
    master_motor_slider_->setTickPosition(QSlider::TicksBelow);
    master_motor_slider_->setTickInterval(1000);
    master_motor_slider_->setValue(0);
    master_motor_slider_->setObjectName("masterMotorSlider");
    master_layout->addWidget(master_motor_slider_, 3, 0, 1, 5);
    master_motor_label_ = label("0 nominal RPM · DShot cap: awaiting bridge", "hint");
    master_motor_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    master_layout->addWidget(master_motor_label_, 4, 0, 1, 5);
    master_layout->setColumnStretch(0, 1);
    master_layout->setColumnStretch(1, 1);
    master_layout->setColumnStretch(2, 0);
    master_layout->setColumnStretch(3, 0);
    master_layout->setColumnStretch(4, 0);
    manual_layout->addWidget(master_panel);

    motor_panels_grid_=new QGridLayout;
    auto* motors=motor_panels_grid_;
    motors->setSpacing(6);
    for (int i=0; i<4; ++i) {
        auto* motor_panel = new QFrame(manual_page);
        motor_panel->setObjectName("chartFrame");
        auto* mv = new QVBoxLayout(motor_panel);
        mv->setContentsMargins(6,4,6,4);
        mv->setSpacing(3);
        auto* name = label(QString("MOTOR %1").arg(i+1), "panelTitle");
        name->setAlignment(Qt::AlignCenter);
        mv->addWidget(name);
        motor_sliders_[std::size_t(i)] = new QSlider(Qt::Vertical, motor_panel);
        motor_sliders_[std::size_t(i)]->setRange(0, 12000);
        motor_sliders_[std::size_t(i)]->setTickPosition(QSlider::TicksLeft);
        motor_sliders_[std::size_t(i)]->setTickInterval(1000);
        motor_sliders_[std::size_t(i)]->setValue(0);
        motor_sliders_[std::size_t(i)]->setFixedHeight(175);
        motor_sliders_[std::size_t(i)]->setFixedWidth(34);
        motor_sliders_[std::size_t(i)]->setObjectName("motorAxisSlider");
        mv->addWidget(motor_sliders_[std::size_t(i)], 1, Qt::AlignHCenter);
        motor_speed_[std::size_t(i)] = spin(0.0, 12000.0, 0.0, 50.0, 0);
        mv->addWidget(motor_speed_[std::size_t(i)]);
        motor_direction_[std::size_t(i)] = new QComboBox(motor_panel);
        motor_direction_[std::size_t(i)]->addItems({"PHASES FIXED"});
        motor_direction_[std::size_t(i)]->setToolTip("Direction determined by physical motor phases.");
        motor_direction_[std::size_t(i)]->setEnabled(false);
        mv->addWidget(motor_direction_[std::size_t(i)]);
        motor_labels_[std::size_t(i)] = label("0 nominal RPM · DShot TX 0", "hint");
        motor_labels_[std::size_t(i)]->setAlignment(Qt::AlignCenter);
        mv->addWidget(motor_labels_[std::size_t(i)]);
        motors->addWidget(motor_panel,i/2,i%2);
        motor_panels_.append(motor_panel);
    }
    manual_layout->addLayout(motors,1);
    // Initial placement is valid; defer responsive relayout until after show().
    // Initial placement is valid; defer responsive relayout until after show().
    // Initial placement is valid; defer responsive relayout until after show().
    manual_layout->addWidget(label("Bench use: keep the cap low at first, remove propellers and secure the drone. DShot here carries open-loop requests only; physical direction control and guaranteed RPM feedback depend on the backend.", "hint"));
    command_source_stack_->addWidget(manual_page);
    ref_tabs_->addTab(reference_tab, "Command");

    auto* preview_tab = new QWidget(ref_tabs_);
    auto* pv = new QVBoxLayout(preview_tab);
    pv->setContentsMargins(4, 4, 4, 3);
    preview_ = new ScopePlot("REFERENCE PREVIEW", "deg", {"Reference"}, preview_tab);
    preview_->setMinimumHeight(115);
    // Preview starts with the original full attitude range, but its AUTO Y
    // toggle can be enabled by the user at any time and remains independent.
    preview_->setYRange(-190,190);
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
    gains_layout->addWidget(label("Balance LQR gains are generated from balance_sim.yaml on controller startup; GUI gain publication does not retune this controller.","hint"));
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
    live_->setAutoFitY(true,4.0);
    liveinner->addWidget(live_,1);
    right->addWidget(livepanel,4);
    // Initial placement is valid; defer responsive relayout until after show().
}
void MainWindow::buildSystem(){
 QVBoxLayout* outer=nullptr;
 auto* page=contentPage(outer,stack_);
 stack_->addWidget(page);
 outer->setContentsMargins(0,0,0,0);
 system_scroll_=new QScrollArea(page);
 system_scroll_->setObjectName("systemScroll");
 system_scroll_->setWidgetResizable(true);
 system_scroll_->setFrameShape(QFrame::NoFrame);
 system_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
 outer->addWidget(system_scroll_,1);
 auto* content=new QWidget(system_scroll_);
 content->setObjectName("systemCanvas");
 auto* root=new QVBoxLayout(content);
 root->setContentsMargins(12,11,12,12);
 root->setSpacing(10);
 system_scroll_->setWidget(content);
 system_scroll_->viewport()->installEventFilter(this);
 root->addWidget(label("CEFIRO / SYSTEM STATUS","pageTitle"));
 root->addWidget(label("Live ROS 2 connection, IMU orientation, sensor frames and controller feedback","pageSubtitle"));
 system_grid_=new QGridLayout;
 auto* g=system_grid_;
 g->setSpacing(10);
 root->addLayout(g);
 QVBoxLayout* sensors=nullptr;auto* sensor_panel=titledPanel("FMU SENSOR LINK",sensors,page);
 system_imu_=label("IMU: WAITING","metricValue");system_mag_=label("MAG: WAITING","metricValue");system_baro_=label("BARO: WAITING","metricValue");
 system_frame_=label("Frames: --","hint");system_angular_=label("Gyro: -- deg/s","metricValue");system_accel_=label("Accel: -- m/s²","metricValue");system_mag_value_=label("Mag: -- µT","metricValue");system_pressure_=label("Pressure: -- hPa","metricValue");system_altitude_=label("Z relative: -- m","metricValue");
 for(QWidget* w : QList<QWidget*>{system_imu_,system_mag_,system_baro_,hline(sensor_panel),system_frame_,system_angular_,system_accel_,system_mag_value_,system_pressure_,system_altitude_})sensors->addWidget(w);
 system_sensor_panel_=sensor_panel;g->addWidget(sensor_panel,0,0);
 QVBoxLayout* ros=nullptr;auto* ros_panel=titledPanel("ROS 2 INTERFACE",ros,page);
 system_controller_=label("Controller: NO FEEDBACK","metricValue");system_topics_=label("Topics initializing...","hint");system_topics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
 ros->addWidget(system_controller_);ros->addWidget(hline(ros_panel));ros->addWidget(system_topics_);ros->addStretch();system_ros_panel_=ros_panel;g->addWidget(ros_panel,0,1);
 // Initial placement is valid; defer responsive relayout until after show().
 auto* note=label("LQR = manual capture. SWING-UP + LQR (SIM) = automatic roll inversion and capture in Gazebo ONLY.\n"
 "Select SWING-UP + LQR (SIM), Output enabled and START TEST; monitor BALANCE status and SAFE STOP.","hint");root->addWidget(note);root->addStretch();
 system_topics_->setText(QString("IMU: %1\nMAG: %2\nBARO: %3\nREF: %4\n"
 "ENABLE: /cefiro/control/enable\nSTOP: /cefiro/control/emergency_stop\n"
 "STATUS: /cefiro/control/status\nPID: /cefiro/control/pid_gains\nLQR: /cefiro/control/lqr_gain")
 .arg(QString::fromStdString(bridge_.imuTopic()),QString::fromStdString(bridge_.magTopic()),QString::fromStdString(bridge_.baroTopic()),QString::fromStdString(bridge_.refTopic())));
}
void MainWindow::connectUi(){
 connect(controller_, QOverload<int>::of(&QComboBox::currentIndexChanged),
         this,[this](int){
             if(command_active_)stopOutput(false);
             resetEquilibriumReference();
             updateReferenceGate();
         });
 connect(manual_btn_,&QPushButton::clicked,this,[this]{setManual(true);});
 connect(automatic_btn_,&QPushButton::clicked,this,[this]{setManual(false);});
 command_pad_->setOnEdited([this](int axis, double value) {
     if (axis < 0 || axis > 2 || manual_mode_ || !reference_pad_unlocked_) return;
     value = std::clamp(value,-reference_limits_deg_[std::size_t(axis)],
                         reference_limits_deg_[std::size_t(axis)]);
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
   if (!manual_mode_ && reference_pad_unlocked_) {
       command_deg_[size_t(i)] = std::clamp(value,-reference_limits_deg_[size_t(i)],
                                          reference_limits_deg_[size_t(i)]);
       manual_dirty_ = true;
       if (command_active_ && publish_clock_.elapsed() > 45) publishCommand();
   }
 });}
 auto updateMotorLabel = [this](int idx) {
   if (idx < 0 || idx >= 4) return;
   motor_labels_[size_t(idx)]->setText(
       QString("Req %1 RPM · dir %2")
           .arg(f(motor_command_rpm_[size_t(idx)],0))
           .arg(motor_command_direction_[size_t(idx)] >= 0 ? "+1" : "-1"));
 };
 auto updateMasterLabel = [this]() {
   if (!master_motor_label_) return;
   const int dir = master_motor_direction_ && master_motor_direction_->currentIndex() == 0 ? 1 : -1;
   master_motor_label_->setText(QString("Req %1 RPM · dir %2")
       .arg(f(master_motor_speed_ ? master_motor_speed_->value() : 0.0,0))
       .arg(dir >= 0 ? "+1" : "-1"));
 };
 auto syncSelectedMotors = [this, updateMotorLabel](double rpm, int direction_index, bool sync_direction) {
   const bool apply_selected = link_all_motors_ && link_all_motors_->isChecked();
   for (int i = 0; i < 4; ++i) {
     const bool selected = !motor_master_select_[size_t(i)] || motor_master_select_[size_t(i)]->isChecked();
     if (apply_selected && !selected) continue;
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
 auto refreshMotorSelectionUi = [this]() {
   const bool use_master = link_all_motors_ && link_all_motors_->isChecked();
   QStringList selected_names;
   for (int i = 0; i < 4; ++i) {
     const bool selected = !motor_master_select_[size_t(i)] || motor_master_select_[size_t(i)]->isChecked();
     if (selected) selected_names << QString("M%1").arg(i + 1);
     if (motor_sliders_[size_t(i)]) motor_sliders_[size_t(i)]->setEnabled(manual_mode_ && !(use_master && selected));
     if (motor_speed_[size_t(i)]) motor_speed_[size_t(i)]->setEnabled(manual_mode_ && !(use_master && selected));
     if (motor_direction_[size_t(i)]) motor_direction_[size_t(i)]->setEnabled(false);
   }
   if (master_select_label_) {
     const QString suffix = selected_names.isEmpty() ? QString("none") : selected_names.join(", ");
     master_select_label_->setText(QString("SYNC TARGETS  [%1]").arg(suffix));
   }
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
 connect(master_motor_slider_, &QSlider::valueChanged, this, [this,syncSelectedMotors,updateMasterLabel](int value){
     QSignalBlocker block(master_motor_speed_);
     master_motor_speed_->setValue(value);
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncSelectedMotors(value, master_motor_direction_->currentIndex(), false);
     }
 });
 connect(master_motor_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this,syncSelectedMotors,updateMasterLabel](double value){
     QSignalBlocker block(master_motor_slider_);
     master_motor_slider_->setValue(int(std::round(value)));
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncSelectedMotors(value, master_motor_direction_->currentIndex(), false);
     }
 });
 connect(master_motor_direction_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this,syncSelectedMotors,updateMasterLabel](int idx){
     updateMasterLabel();
     if (link_all_motors_ && link_all_motors_->isChecked()) {
         syncSelectedMotors(master_motor_speed_->value(), idx, true);
     }
 });
 connect(link_all_motors_, &QCheckBox::toggled, this, [this,syncSelectedMotors,refreshMotorSelectionUi](bool on){
     refreshMotorSelectionUi();
     if (on) {
         syncSelectedMotors(master_motor_speed_->value(), master_motor_direction_->currentIndex(), true);
     }
 });
 for (int i = 0; i < 4; ++i) {
     if (motor_master_select_[size_t(i)]) {
         connect(motor_master_select_[size_t(i)], &QCheckBox::toggled, this, [this,syncSelectedMotors,refreshMotorSelectionUi](bool){
             refreshMotorSelectionUi();
             if (link_all_motors_ && link_all_motors_->isChecked()) {
                 syncSelectedMotors(master_motor_speed_->value(), master_motor_direction_->currentIndex(), true);
             }
         });
     }
 }
 if (master_select_all_btn_) connect(master_select_all_btn_, &QPushButton::clicked, this, [this,refreshMotorSelectionUi,syncSelectedMotors]{
     for (auto* box : motor_master_select_) if (box) { QSignalBlocker hold(box); box->setChecked(true); }
     refreshMotorSelectionUi();
     if (link_all_motors_ && link_all_motors_->isChecked()) syncSelectedMotors(master_motor_speed_->value(), master_motor_direction_->currentIndex(), true);
 });
 if (master_select_none_btn_) connect(master_select_none_btn_, &QPushButton::clicked, this, [this,refreshMotorSelectionUi]{
     for (auto* box : motor_master_select_) if (box) { QSignalBlocker hold(box); box->setChecked(false); }
     refreshMotorSelectionUi();
 });
 updateMasterLabel();
 refreshMotorSelectionUi();
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
 preview_->setSeriesData(0,points);
 preview_->setWindowSeconds(std::clamp(total,1.0,60.0));
 // Never override the plot's user-selected manual/auto vertical scale.
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
 for(std::size_t i=0;i<motor_sliders_.size();++i){
  const bool selected = motor_master_select_[i] && motor_master_select_[i]->isChecked();
  if(motor_sliders_[i]) motor_sliders_[i]->setEnabled(yes && !(linked && selected));
  if(motor_speed_[i]) motor_speed_[i]->setEnabled(yes && !(linked && selected));
 }
 for(auto* d:motor_direction_) if(d) d->setEnabled(false);
 if (master_motor_slider_) master_motor_slider_->setEnabled(yes);
 if (master_motor_speed_) master_motor_speed_->setEnabled(yes);
 if (master_motor_direction_) master_motor_direction_->setEnabled(false);
 if (link_all_motors_) link_all_motors_->setEnabled(yes);
 for(auto* box:motor_master_select_) if(box) box->setEnabled(yes);
 if(master_select_all_btn_) master_select_all_btn_->setEnabled(yes);
 if(master_select_none_btn_) master_select_none_btn_->setEnabled(yes);
 command_source_stack_->setCurrentIndex(yes ? 1 : 0);
 if (command_pad_) {
   command_pad_->setEnabledForEditing(!yes && reference_pad_unlocked_);
   command_pad_->setAngles({manual_[0]->value(),manual_[1]->value(),manual_[2]->value()});
 }
 if(wave_running_){waveform_timer_->stop();wave_running_=false;}
 if(!yes) for(int i=0;i<3;++i) command_deg_[size_t(i)] =
    std::clamp(manual_[size_t(i)]->value(),-reference_limits_deg_[size_t(i)],
               reference_limits_deg_[size_t(i)]);
 updateReferenceGate();
 if(command_active_){ if(manual_mode_) publishMotorCommand(); else publishCommand(); }
}
void MainWindow::resetEquilibriumReference(){
    reference_pad_unlocked_ = false;
    command_deg_ = {0.0,0.0,0.0};
    for (int i=0;i<3;++i) {
        QSignalBlocker block(manual_[size_t(i)]);
        manual_[size_t(i)]->setValue(0.0);
        manual_[size_t(i)]->setEnabled(false);
        manual_labels_[size_t(i)]->setText("0.00°");
    }
    if(command_pad_){
        command_pad_->setAngles({0.,0.,0.});
        command_pad_->setEnabledForEditing(false);
    }
    manual_dirty_ = false;
}
void MainWindow::applyReferenceLimits(const Vec3& requested){
    Vec3 limits{};
    for (std::size_t i=0; i<3; ++i) {
        const double cap = i==2 ? 180.0 : 30.0;
        limits[i] = std::clamp(requested[i], 0.5, cap);
    }
    bool changed=false;
    for (std::size_t i=0; i<3; ++i)
        changed |= std::abs(reference_limits_deg_[i]-limits[i])>0.01;
    if(!changed)return;
    reference_limits_deg_=limits;
    if(command_pad_)command_pad_->setLimits(limits);
    for (std::size_t i=0; i<3; ++i) {
        if(manual_[i]) {
            QSignalBlocker hold(manual_[i]);
            manual_[i]->setRange(-limits[i],limits[i]);
            manual_[i]->setValue(std::clamp(manual_[i]->value(),-limits[i],limits[i]));
            if(manual_labels_[i])manual_labels_[i]->setText(f(manual_[i]->value())+"°");
        }
        command_deg_[i]=std::clamp(command_deg_[i],-limits[i],limits[i]);
    }
    if(command_pad_)command_pad_->setAngles({manual_[0]->value(),manual_[1]->value(),manual_[2]->value()});
    if(reference_pad_unlocked_ && command_active_)publishCommand();
}
void MainWindow::refreshReferenceLimits(){
    if(bridge_.isReal()){
        // No nonlinear LQR/actuator certification for the physical plant.
        applyReferenceLimits({5.0,5.0,10.0});
        return;
    }
    Vec3 limits{};
    if(bridge_.simReferenceEnvelope(limits))applyReferenceLimits(limits);
}
void MainWindow::updateReferenceGate(){
    const bool is_lqr = controller_ && controller_->currentText().contains("LQR");
    // IMPORTANT: only an acknowledged CAPTURED is a proof of successful
    // state transition. An enabled button does NOT prove balance.
    const bool balanced = bridge_.feedbackFresh(1.2) &&
                          bridge_.controllerStatus()=="BALANCE: CAPTURED";
    const bool ready = is_lqr && !manual_mode_ && command_active_ &&
                       !e_stop_ && bridge_.imuFresh(0.5) && balanced;
    if(reference_pad_unlocked_ && !ready){
        resetEquilibriumReference();
        // Never leave a stale off-equilibrium reference after losing CAPTURED.
        if(command_active_ && !manual_mode_) publishCommand();
    }
    reference_pad_unlocked_=ready;
    if(command_pad_)command_pad_->setEnabledForEditing(ready);
    for(auto* m:manual_)if(m)m->setEnabled(ready);
    if(auto_command_tabs_ && is_lqr){
        auto_command_tabs_->setTabEnabled(1,false); // no waveform bypass for LQR
        if(auto_command_tabs_->currentIndex()!=0)auto_command_tabs_->setCurrentIndex(0);
    } else if(auto_command_tabs_)auto_command_tabs_->setTabEnabled(1,!manual_mode_);
    if(reference_gate_label_){
        if(ready) reference_gate_label_->setText(
            QString("UNLOCKED · BALANCE: CAPTURED · R +/- %1°, P +/- %2°, Y +/- %3° · SIM model estimate/fallback")
                .arg(f(reference_limits_deg_[0],1),f(reference_limits_deg_[1],1),f(reference_limits_deg_[2],1)));
        else reference_gate_label_->setText(
            QString("LOCKED · Wait for BALANCE: CAPTURED and valid IMU · Model/pilot limits: R +/- %1°, P +/- %2°, Y +/- %3°")
                .arg(f(reference_limits_deg_[0],1),f(reference_limits_deg_[1],1),f(reference_limits_deg_[2],1)));
    }
}
void MainWindow::startOutput(){
 if(bridge_.isReal() && !manual_mode_ && controller_->currentText().contains("SWING-UP")){
   QMessageBox::critical(this,"Simulation-only controller",
       "SWING-UP + LQR has not been validated on real motors. Select SIMULATION to run it.");
   return;
 }
 if(!output_enabled_->isChecked()){
  QMessageBox::information(this,"Output inhibited","Enable 'Output enabled' before sending controller commands.");return;
 }
 if(!manual_mode_ && controller_->currentText().contains("LQR") && !bridge_.magFresh()) {
  QMessageBox::warning(this,"Magnetometer unavailable", "CEFIRO balance capture requires the characterized MMC5983MA magnetometer topic."); return;
 }
 if(!manual_mode_ && !bridge_.imuFresh()){
  QMessageBox::warning(this,"IMU unavailable","No recent IMU data. Automatic reference generation is inhibited to avoid blind actuation.");return;
 }
 if (!manual_mode_ && controller_->currentText() == "SWING-UP + LQR (SIM)" && !bridge_.feedbackFresh()) {
   QMessageBox::critical(this, "Swing-up controller OFFLINE",
     "No live controller feedback on /cefiro/control/status.\n"
     "Start Gazebo with cefiro_balance, verify the selected ROS overlay, and retry.\n"
     "Automatic inversion is inhibited until the controller is online.");
   return;
 }
 if(!manual_mode_ && !bridge_.feedbackFresh()){
  const auto response=QMessageBox::warning(this,"Controller feedback unavailable",
    "No external controller has acknowledged the command interface.\n"
    "Start publishing references anyway? This does NOT verify any actuator is armed.",
    QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
  if(response!=QMessageBox::Yes)return;
 }
 if (!manual_mode_ && controller_->currentText() == "SWING-UP + LQR (SIM)") {
   const auto response=QMessageBox::warning(this,"SIMULATION ONLY: automatic inversion",
     "This experimental roll swing-up can spin the quadrotor through large angles.\n"
     "It is NOT validated for real ESCs or propellers. Launch Gazebo only, "
     "keep the simulated gimbal clear, and watch SAFE STOP.\n\n"
     "Start automatic inversion in SIMULATION?",
     QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
   if(response!=QMessageBox::Yes)return;
 }
 // Tell the balance node that manual RPM mode owns the actuator topic BEFORE enabling.
 // Avoid stale ATTITUDE/LQR mode causing dual publishers on the same motors.
 if (manual_mode_) bridge_.publishReference({0.0,0.0,0.0},"MANUAL","OPEN LOOP");
 resetEquilibriumReference();
 e_stop_=false;command_active_=true;
 if(plant_selector_)plant_selector_->setEnabled(false);
 bridge_.setEnabled(true);
 updateReferenceGate();
 if (manual_mode_) bridge_.setMotorArmed(true);
 activate_btn_->setText("COMMANDS ENABLED");
 if(auto_record_->isChecked()&&!recorder_active_)startRecording();
 if(manual_mode_){
   wave_running_=false;
   publishMotorCommand();
 } else if (controller_->currentText().contains("LQR")) {
   // Equilibrium-only reference. The simulation node performs swing-up if explicitly selected.
   command_deg_={0.0,0.0,0.0};
   for (int i=0;i<3;++i) {
     QSignalBlocker block(manual_[size_t(i)]);
     manual_[size_t(i)]->setValue(0.0);
   }
   command_pad_->setAngles({0.0,0.0,0.0});
   wave_running_=false;
   publishCommand();
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
    if(plant_selector_)plant_selector_->setEnabled(true);
    e_stop_ = emergency;
    resetEquilibriumReference();
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
 // Nonzero LQR references may only leave the GUI after confirmed CAPTURED.
 if(!manual_mode_ && controller_->currentText().contains("LQR")) {
    if (!reference_pad_unlocked_) command_deg_={0.0,0.0,0.0};
    else for(std::size_t i=0;i<3;++i)command_deg_[i]=
       std::clamp(command_deg_[i],-reference_limits_deg_[i],reference_limits_deg_[i]);
 }
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
 // The UI must not report a controller transition based on a reference publish alone.
 if (bridge_.feedbackFresh()) {
   applied_value_->setText("Controller: " + QString::fromStdString(bridge_.controllerStatus()));
 } else {
   applied_value_->setText("NO CONTROLLER FEEDBACK - check /cefiro_balance");
 }
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
void MainWindow::setMonitorAxis(int idx){
    if(idx<0||idx>2)return;
    monitor_axis_=idx;
    const auto source=bridge_.isReal()?1u:0u;
    live_->setSeriesData(0,live_refs_[source][size_t(idx)]);
    live_->setSeriesData(1,live_meas_[source][size_t(idx)]);
    // Keep the operator's AUTO Y preference when changing monitor axes.
}
void MainWindow::tick(){
 refreshReferenceLimits();
 updateReferenceGate();
 // A fresh reference is a GUI heartbeat for the simulation-only balance node.
 if(command_active_ && !manual_mode_ && controller_->currentText().contains("LQR") &&
    publish_clock_.elapsed()>150) publishCommand();
 const auto items=bridge_.drain();const bool fresh=bridge_.imuFresh();
 status_ros_->setText(fresh?"ROS: IMU ONLINE":"ROS: IMU WAITING / STALE");
 status_control_->setText(command_active_?"OUTPUT REQUEST: ENABLED":"CONTROL: DISABLED");
 if(bridge_.feedbackFresh())status_control_->setText("CTRL: "+QString::fromStdString(bridge_.controllerStatus()));
 if (command_active_ && !manual_mode_) {
   if (bridge_.feedbackFresh()) {
     applied_value_->setText("Controller: " + QString::fromStdString(bridge_.controllerStatus()));
   } else {
     applied_value_->setText("NO CONTROLLER FEEDBACK - swing-up inactive");
   }
 }
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
 // One fresh render per sensor and plant per UI frame, regardless of which
 // ROS callback arrived last. Prevents REAL suppressing SIM or vice versa.
 std::array<std::array<std::size_t,3>,2> last_event{};
 for (auto& source:last_event)source.fill(items.size());
 for(std::size_t i=0;i<items.size();++i){
     const auto& s=items[i];
     last_event[s.from_real ? 1 : 0][static_cast<std::size_t>(s.kind)]=i;
 }
 for(std::size_t i=0;i<items.size();++i){
    const auto& s=items[i];
    ++collected_;
    const bool selected=s.from_real==bridge_.isReal();
    if(selected && s.kind==SampleKind::Imu)stats_.add(s);
    if(selected && s.kind==SampleKind::Imu && s.orientation_valid) {
        for (int axis = 0; axis < 3; ++axis) {
            response_analyzers_[std::size_t(axis)].add(
                s.ros_time_s, s.attitude_deg[std::size_t(axis)]);
        }
    }
    if(selected && s.kind==SampleKind::Imu && recorder_active_)saveRecord(s);
    const bool last_in_source=last_event[s.from_real ? 1 : 0]
                                    [static_cast<std::size_t>(s.kind)]==i;
    onSample(s,last_in_source);
 }
 updateSourceConnections();
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
 // Motor monitoring is independent of the IMU and of the selected control plant.
 // Plot actual transport/plant messages, not the GUI slider value as feedback.
 const auto motor_events=bridge_.drainMotor();
 std::array<std::array<std::size_t,5>,2> latest_motor{};
 for(auto& source:latest_motor)source.fill(motor_events.size());
 for(std::size_t i=0;i<motor_events.size();++i){
     const auto& event=motor_events[i];
     latest_motor[event.from_real?1:0][static_cast<std::size_t>(event.kind)]=i;
 }
 const double motor_time=clock_.elapsed()/1000.0;
 for(std::size_t i=0;i<motor_events.size();++i){
     const auto& event=motor_events[i];
     if(latest_motor[event.from_real?1:0][static_cast<std::size_t>(event.kind)]!=i)continue;
     onMotorEvent(event,motor_time);
 }
 // The hardware status JSON is a fallback only when the dedicated host
 // commanded-DShot topic is absent. It is never RPM feedback.
 if(bridge_.hardwareStatusFresh()){
     const auto json=QJsonDocument::fromJson(QByteArray::fromStdString(bridge_.hardwareStatus()));
     if(json.isObject() && json.object().value("serial_connected").toBool(false) &&
        (last_motor_group_s_[1][2]<0.0 || motor_time-last_motor_group_s_[1][2]>0.6)){
         const auto codes=json.object().value("last_commanded_dshot").toArray();
         if(codes.size()==4 && motor_time-last_dshot_time_>=0.05){
             MotorEvent fallback;
             fallback.from_real=true;
             fallback.kind=MotorEventKind::HostCommandedDshot;
             bool valid=true;
             for(int motor=0;motor<4;++motor){
                 const int code=codes.at(motor).toInt(-1);
                 if(code!=0 && (code<48 || code>2047)){valid=false;break;}
                 fallback.dshot[size_t(motor)]=static_cast<std::uint16_t>(code);
             }
             if(valid){onMotorEvent(fallback,motor_time);last_dshot_time_=motor_time;}
         }
     }
 }
 expireMotorGroups(motor_time);
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
void MainWindow::clearMotorGroup(bool real,int group){
    const int source=real?1:0;
    ScopePlot* plot=(group==0)?rpm_target_plot_:(group==1?rpm_feedback_plot_:dash_[8]);
    if(plot)for(int motor=0;motor<4;++motor)plot->clearSeries(source*4+motor);
    last_motor_group_s_[size_t(source)][size_t(group)]=-1.0;
    if(real==bridge_.isReal()){
        auto* metric=(group==0)?rpm_target_value_:(group==1?rpm_feedback_value_:dshot_value_);
        if(metric)metric->setText(group==2?"-- DShot":"-- RPM");
    }
}
void MainWindow::expireMotorGroups(double time_s){
    constexpr double timeout_s=2.0;
    for(int source=0;source<2;++source){
        for(int group=0;group<3;++group){
            const double last=last_motor_group_s_[size_t(source)][size_t(group)];
            if(last>=0.0 && time_s-last>timeout_s)clearMotorGroup(source==1,group);
        }
    }
}
void MainWindow::onMotorEvent(const MotorEvent& event,double time_s){
    const int source=event.from_real?1:0;
    const bool selected=bridge_.isReal()==event.from_real;
    const int first=source*4;
    const auto format_motors=[](const auto& values,const QString& suffix){
        return QString("M1 %1 · M2 %2 · M3 %3 · M4 %4 %5")
            .arg(f(values[0],0)).arg(f(values[1],0))
            .arg(f(values[2],0)).arg(f(values[3],0)).arg(suffix);
    };
    const auto append_requested=[&](const std::array<double,4>& values){
        if(rpm_target_plot_){
            for(int m=0;m<4;++m)rpm_target_plot_->append(first+m,time_s,values[size_t(m)]);
        }
        last_motor_group_s_[size_t(source)][0]=time_s;
        if(selected && rpm_target_value_)rpm_target_value_->setText(format_motors(values,"RPM requested"));
    };
    const auto append_measured=[&](const std::array<double,4>& values){
        if(rpm_feedback_plot_){
            for(int m=0;m<4;++m)rpm_feedback_plot_->append(first+m,time_s,values[size_t(m)]);
        }
        last_motor_group_s_[size_t(source)][1]=time_s;
        if(selected && rpm_feedback_value_)rpm_feedback_value_->setText(format_motors(values,"RPM measured"));
    };
    const auto append_dshot=[&](const std::array<std::uint16_t,4>& codes,const QString& origin){
        for(int m=0;m<4;++m)dash_[8]->append(first+m,time_s,codes[size_t(m)]);
        last_motor_group_s_[size_t(source)][2]=time_s;
        if(selected && dshot_value_)dshot_value_->setText(format_motors(codes,origin));
    };
    switch(event.kind){
        case MotorEventKind::TargetRpm:
            append_requested(event.requested_rpm);
            break;
        case MotorEventKind::DirectDshot:
            // For raw DShot commands there is NO RPM reference; do not invent one.
            if(last_motor_group_s_[size_t(source)][0]>=0.0)
                clearMotorGroup(event.from_real,0);
            // Only show raw input in the absence of recent backend-applied DShot.
            if(last_motor_group_s_[size_t(source)][2]<0.0 ||
               time_s-last_motor_group_s_[size_t(source)][2]>0.6)
                append_dshot(event.dshot,"DShot input (unconfirmed)");
            break;
        case MotorEventKind::MeasuredRpm:
            // MotorTelemetry contains the same feedback with richer provenance.
            if(last_motor_telemetry_s_[size_t(source)]<0.0 ||
               time_s-last_motor_telemetry_s_[size_t(source)]>0.6)
                append_measured(event.measured_rpm);
            break;
        case MotorEventKind::Telemetry:
            last_motor_telemetry_s_[size_t(source)]=time_s;
            if(event.command_source==1)
                append_requested(event.requested_rpm);
            else if(event.command_source==2 && last_motor_group_s_[size_t(source)][0]>=0.0)
                clearMotorGroup(event.from_real,0);
            append_measured(event.measured_rpm);
            append_dshot(event.dshot,"DShot applied / telemetry");
            break;
        case MotorEventKind::HostCommandedDshot:
            append_dshot(event.dshot,"host DShot request");
            break;
    }
}
void MainWindow::onSample(const Telemetry& s,bool draw){
    const bool selected=s.from_real==bridge_.isReal();
    // Only IMU updates should drive the controller performance/status UI.
    if(selected && s.kind==SampleKind::Imu){
        last_sample_=s;
        have_sample_=true;
    }
    if(!draw)return;
    const double t=clock_.elapsed()/1000.0;
    const int src=s.from_real?1:0;
    if(s.kind==SampleKind::Imu){
        if(s.orientation_valid){
            for(int axis=0;axis<3;++axis){
                dash_[size_t(axis)]->append(src?2:0,t,s.setpoint_deg[axis]);
                dash_[size_t(axis)]->append(src?3:1,t,s.attitude_deg[axis]);
            }
        }
        for(int axis=0;axis<3;++axis){
            dash_[3]->append(src*3+axis,t,s.angular_rate_deg_s[axis]);
            dash_[4]->append(src*3+axis,t,s.accel_m_s2[axis]);
        }
        if(!selected)return;
        for(int axis=0;axis<3;++axis){
            attitude_values_[size_t(axis)]->setText(
                s.orientation_valid ? f(s.attitude_deg[axis],2)+" deg":"NO ORIENTATION");
        }
        rate_values_[0]->setText(QString("p %1   q %2   r %3 deg/s")
            .arg(f(s.angular_rate_deg_s[0]),f(s.angular_rate_deg_s[1]),f(s.angular_rate_deg_s[2])));
        rate_values_[1]->setText(QString("ax %1   ay %2   az %3 m/s²")
            .arg(f(s.accel_m_s2[0]),f(s.accel_m_s2[1]),f(s.accel_m_s2[2])));
        view_->setAttitude(s.orientation,s.orientation_valid);
        if(s.orientation_valid){
            for(int axis=0;axis<3;++axis){
                auto& ref=live_refs_[size_t(src)][size_t(axis)];
                auto& measured=live_meas_[size_t(src)][size_t(axis)];
                ref.append(QPointF(t,s.setpoint_deg[axis]));
                measured.append(QPointF(t,s.attitude_deg[axis]));
                while(ref.size()>20000 || (!ref.isEmpty() && ref.first().x()<t-600.0))ref.removeFirst();
                while(measured.size()>20000 || (!measured.isEmpty() && measured.first().x()<t-600.0))measured.removeFirst();
            }
            live_->append(0,t,s.setpoint_deg[monitor_axis_]);
            live_->append(1,t,s.attitude_deg[monitor_axis_]);
        }
    }else if(s.kind==SampleKind::Magnetometer && s.magnetometer_valid){
        double magnitude_sq=0.0;
        for(int axis=0;axis<3;++axis){
            const double component=s.mag_uT[size_t(axis)];
            dash_[5]->append(src*3+axis,t,component); // Tesla -> microtesla in RosBridge.
            magnitude_sq+=component*component;
        }
        const double field_magnitude=std::sqrt(magnitude_sq);
        dash_[7]->append(src,t,field_magnitude);
        if(selected && magnitude_value_)
            magnitude_value_->setText(f(field_magnitude,2)+" µT");
        if(selected){
            rate_values_[2]->setText(QString("Bx %1   By %2   Bz %3 µT")
                .arg(f(s.mag_uT[0]),f(s.mag_uT[1]),f(s.mag_uT[2])));
        }
    }else if(s.kind==SampleKind::Barometer && s.barometer_valid){
        dash_[6]->append(src,t,s.pressure_hPa);
        if(altitude_plot_)altitude_plot_->append(src,t,s.relative_alt_m);
        if(selected){
            if(pressure_value_)pressure_value_->setText(f(s.pressure_hPa,2)+" hPa");
            if(altitude_value_)altitude_value_->setText(f(s.relative_alt_m,2)+" m");
        }
    }
}

void MainWindow::setConnectionBadge(QLabel* badge,const QString& name,bool connected){
    if(!badge)return;
    badge->setText(QString("● %1  %2").arg(name,connected?"CONNECTED":"DISCONNECTED"));
    badge->setStyleSheet(connected
        ? "QLabel { color: #36df91; font-weight: 700; padding: 2px 7px; }"
        : "QLabel { color: #ff6576; font-weight: 700; padding: 2px 7px; }");
}

void MainWindow::clearSensorHistory(bool real,SampleKind sensor){
    const int source=real?1:0;
    if(sensor==SampleKind::Imu){
        for(int axis=0;axis<3;++axis){
            dash_[size_t(axis)]->clearSeries(source?2:0);
            dash_[size_t(axis)]->clearSeries(source?3:1);
        }
        for(int axis=0;axis<3;++axis){
            dash_[3]->clearSeries(source*3+axis);
            dash_[4]->clearSeries(source*3+axis);
            live_refs_[size_t(source)][size_t(axis)].clear();
            live_meas_[size_t(source)][size_t(axis)].clear();
        }
        if(real==bridge_.isReal())resetSelectedFeedback();
    }else if(sensor==SampleKind::Magnetometer){
        for(int axis=0;axis<3;++axis)dash_[5]->clearSeries(source*3+axis);
        dash_[7]->clearSeries(source);
        if(real==bridge_.isReal()){
            rate_values_[2]->setText("MAG: DISCONNECTED");
            if(magnitude_value_)magnitude_value_->setText("-- µT");
        }
    }else if(sensor==SampleKind::Barometer){
        dash_[6]->clearSeries(source);
        if(altitude_plot_)altitude_plot_->clearSeries(source);
        if(real==bridge_.isReal()){
            if(pressure_value_)pressure_value_->setText("-- hPa");
            if(altitude_value_)altitude_value_->setText("-- m");
        }
    }
}

void MainWindow::clearSourceHistory(bool real){
    clearSensorHistory(real,SampleKind::Imu);
    clearSensorHistory(real,SampleKind::Magnetometer);
    clearSensorHistory(real,SampleKind::Barometer);
    for(int group=0;group<3;++group)clearMotorGroup(real,group);
    last_motor_telemetry_s_[real?1u:0u]=-1.0;
    if(real)last_dshot_time_=-1.0;
    bridge_.resetTelemetry(real);
}

void MainWindow::resetSelectedFeedback(){
    have_sample_=false;
    stats_.reset();
    last_sample_=Telemetry{};
    for(auto& analyzer:response_analyzers_)analyzer=StepResponseAnalyzer{};
    for(auto* l:attitude_values_)if(l)l->setText("-- deg");
    for(auto* l:rate_values_)if(l)l->setText("--");
    if(pressure_value_)pressure_value_->setText("-- hPa");
    if(altitude_value_)altitude_value_->setText("-- m");
    if(magnitude_value_)magnitude_value_->setText("-- µT");
    if(metric_value_)metric_value_->setText("RMSE --  |  MAE --  |  PEAK --");
    if(view_)view_->setAttitude(Quaternion{},false);
    if(live_)setMonitorAxis(monitor_axis_);
    for(auto* l:rmse_values_)if(l)l->setText("--");
}

void MainWindow::updateSourceConnections(){
    for(int index=0;index<2;++index){
        const bool real=index==1;
        const bool imu=bridge_.imuFreshFor(real);
        const bool mag=bridge_.magFreshFor(real);
        const bool baro=bridge_.baroFreshFor(real);
        bool host_serial_connected=false;
        if(real && bridge_.hardwareStatusFresh()){
            const auto status=QJsonDocument::fromJson(
                QByteArray::fromStdString(bridge_.hardwareStatus()));
            host_serial_connected=status.isObject() &&
                status.object().value("serial_connected").toBool(false);
        }
        const bool connected=imu||mag||baro||host_serial_connected||bridge_.motorFreshFor(real);
        if(imu_online_[size_t(index)] && !imu)clearSensorHistory(real,SampleKind::Imu);
        if(mag_online_[size_t(index)] && !mag)clearSensorHistory(real,SampleKind::Magnetometer);
        if(baro_online_[size_t(index)] && !baro)clearSensorHistory(real,SampleKind::Barometer);
        if(source_online_[size_t(index)] && !connected)clearSourceHistory(real);
        if(source_online_[size_t(index)]!=connected){
            setConnectionBadge(real?status_real_connection_:status_sim_connection_,
                               real?"REAL":"SIM",connected);
        }
        source_online_[size_t(index)]=connected;
        imu_online_[size_t(index)]=imu;
        mag_online_[size_t(index)]=mag;
        baro_online_[size_t(index)]=baro;
    }
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
 system_mag_value_->setText(bridge_.magFresh()
  ? QString("Mag: [%1, %2, %3] µT").arg(f(s.mag_uT[0]),f(s.mag_uT[1]),f(s.mag_uT[2]))
  : "Mag: -- µT (STALE)");
 system_pressure_->setText(bridge_.baroFresh()
  ? QString("Pressure: %1 hPa").arg(f(s.pressure_hPa,3))
  : "Pressure: -- hPa (STALE)");
 system_altitude_->setText(bridge_.baroFresh()
  ? QString("Z relative: %1 m (pressure-derived)").arg(f(s.relative_alt_m,3))
  : "Z relative: -- m (STALE)");
}
bool MainWindow::eventFilter(QObject* watched,QEvent* event){
    // Qt sends Resize events synchronously during layout negotiation. Altering
    // that same layout from the event filter can recursively re-enter Qt.
    if(event->type()==QEvent::Resize && ui_ready_){
        const bool tracked =
            (dashboard_scroll_ && watched==dashboard_scroll_->viewport()) ||
            (control_scroll_ && watched==control_scroll_->viewport()) ||
            (system_scroll_ && watched==system_scroll_->viewport()) ||
            (control_left_ && watched==control_left_);
        if(tracked)scheduleResponsiveLayout();
    }
    return QMainWindow::eventFilter(watched,event);
}
void MainWindow::resizeEvent(QResizeEvent* event){
    QMainWindow::resizeEvent(event);
    if(ui_ready_)scheduleResponsiveLayout();
}
void MainWindow::scheduleResponsiveLayout(){
    // Emergency diagnostic fallback: use the initial static (two-column)
    // layout without moving QWidget instances at runtime.
    if(qEnvironmentVariableIsSet("CEFIRO_GUI_DISABLE_REFLOW"))return;
    if(!ui_ready_ || shutting_down_ || !responsive_timer_)return;
    // Coalesce multiple QWidget/viewport Resize notifications into one pass.
    // Do not restart the timer forever while the user drags window borders.
    if(!responsive_timer_->isActive())responsive_timer_->start();
}
void MainWindow::applyResponsiveLayout(){
    if(!ui_ready_ || shutting_down_ || reflow_in_progress_)return;
    reflow_in_progress_=true;
    if(dashboard_scroll_)reflowDashboard(dashboard_scroll_->viewport()->width());
    if(control_scroll_)reflowControl(control_scroll_->viewport()->width());
    if(system_scroll_)reflowSystem(system_scroll_->viewport()->width());
    if(control_left_){
        const int w=control_left_->width();
        reflowMotorControls(w);
        reflowMasterControl(w);
        reflowTrajectoryFields(w);
    }
    updateCompactChrome(width());
    reflow_in_progress_=false;
}
void MainWindow::reflowDashboard(int available_width){
    if(!dashboard_grid_||dashboard_panels_.isEmpty())return;
    // Three columns only on genuinely wide desktop monitors.
    const int cols=dashboard_columns_==3
        ? (available_width<2040?(available_width<890?1:2):3)
        : dashboard_columns_==2
        ? (available_width>=2160?3:(available_width<890?1:2))
        : (available_width>=2160?3:(available_width>=960?2:1));
    if(dashboard_columns_==cols)return;
    dashboard_columns_=cols;
    for(QWidget* panel:dashboard_panels_)dashboard_grid_->removeWidget(panel);
    for(int i=0;i<dashboard_panels_.size();++i){
        QWidget* panel=dashboard_panels_[i];
        dashboard_grid_->addWidget(panel,i/cols,i%cols);
        const int width_hint=available_width/cols;
        const int card_height=width_hint<530?303:width_hint<850?355:395;
        panel->setMinimumHeight(card_height);
        // ScopePlot has no Q_OBJECT. Qt's findChildren<ScopePlot*>() would
        // match *any* QFrame subclass via ScopePlot's inherited metaobject.
        // Check the C++ runtime type before calling ScopePlot methods.
        for(QWidget* child:panel->findChildren<QWidget*>()){
            if(auto* scope=dynamic_cast<ScopePlot*>(child)){
                scope->setMinimumHeight(std::max(190,card_height-85));
            }
        }
    }
    for(int c=0;c<3;++c)dashboard_grid_->setColumnStretch(c,c<cols?1:0);
    for(int r=0;r<dashboard_panels_.size();++r)dashboard_grid_->setRowStretch(r,0);
    dashboard_grid_->invalidate();
}
void MainWindow::reflowControl(int available_width){
    if(!control_columns_||!control_left_||!control_right_)return;
    const int mode=control_columns_mode_==2 ? (available_width<1020?1:2) : (available_width>1080?2:1);
    if(control_columns_mode_==mode)return;
    control_columns_mode_=mode;
    control_columns_->removeWidget(control_left_);
    control_columns_->removeWidget(control_right_);
    if(mode==1){
        control_columns_->addWidget(control_left_,0,0);
        control_columns_->addWidget(control_right_,1,0);
        control_columns_->setColumnStretch(0,1);
        control_columns_->setColumnStretch(1,0);
        control_columns_->setRowStretch(0,0);
        control_columns_->setRowStretch(1,0);
    }else{
        control_columns_->addWidget(control_left_,0,0);
        control_columns_->addWidget(control_right_,0,1);
        control_columns_->setColumnStretch(0,38);
        control_columns_->setColumnStretch(1,62);
        control_columns_->setRowStretch(1,0);
    }
    control_columns_->invalidate();
}
void MainWindow::reflowSystem(int available_width){
    if(!system_grid_||!system_sensor_panel_||!system_ros_panel_)return;
    const int mode=system_columns_mode_==2 ? (available_width<820?1:2) : (available_width>880?2:1);
    if(system_columns_mode_==mode)return;
    system_columns_mode_=mode;
    system_grid_->removeWidget(system_sensor_panel_);
    system_grid_->removeWidget(system_ros_panel_);
    if(mode==1){
        system_grid_->addWidget(system_sensor_panel_,0,0);
        system_grid_->addWidget(system_ros_panel_,1,0);
        system_grid_->setColumnStretch(0,1);
        system_grid_->setColumnStretch(1,0);
    }else{
        system_grid_->addWidget(system_sensor_panel_,0,0);
        system_grid_->addWidget(system_ros_panel_,0,1);
        system_grid_->setColumnStretch(0,1);
        system_grid_->setColumnStretch(1,1);
    }
    system_grid_->invalidate();
}
void MainWindow::reflowMotorControls(int available_width){
    if(!motor_panels_grid_||motor_panels_.size()!=4)return;
    const int cols=motor_columns_mode_==4
        ? (available_width<760?(available_width<420?1:2):4)
        : motor_columns_mode_==2
        ? (available_width>820?4:(available_width<420?1:2))
        : (available_width>820?4:(available_width>470?2:1));
    if(motor_columns_mode_==cols)return;
    motor_columns_mode_=cols;
    for(QWidget* panel:motor_panels_)motor_panels_grid_->removeWidget(panel);
    for(int i=0;i<motor_panels_.size();++i){
        motor_panels_grid_->addWidget(motor_panels_[i],i/cols,i%cols);
    }
    for(int c=0;c<4;++c)motor_panels_grid_->setColumnStretch(c,c<cols?1:0);
    motor_panels_grid_->invalidate();
}
void MainWindow::reflowMasterControl(int available_width){
    if(!master_control_grid_||!master_title_||!master_motor_label_||!master_select_widget_)return;
    const int mode=master_columns_mode_==2 ? (available_width<545?1:2) : (available_width>590?2:1);
    if(master_columns_mode_==mode)return;
    master_columns_mode_=mode;
    for(QWidget* widget:{static_cast<QWidget*>(master_title_),
                         static_cast<QWidget*>(link_all_motors_),
                         static_cast<QWidget*>(master_rpm_label_),
                         static_cast<QWidget*>(master_motor_speed_),
                         static_cast<QWidget*>(master_select_widget_),
                         static_cast<QWidget*>(master_motor_direction_),
                         static_cast<QWidget*>(master_motor_slider_),
                         static_cast<QWidget*>(master_motor_label_)}){
        master_control_grid_->removeWidget(widget);
    }
    if(mode==1){
        master_control_grid_->addWidget(master_title_,0,0,1,2);
        master_control_grid_->addWidget(link_all_motors_,1,0,1,2);
        master_control_grid_->addWidget(master_select_widget_,2,0,1,2);
        master_control_grid_->addWidget(master_rpm_label_,3,0);
        master_control_grid_->addWidget(master_motor_speed_,3,1);
        master_control_grid_->addWidget(master_motor_direction_,4,0,1,2);
        master_control_grid_->addWidget(master_motor_slider_,5,0,1,2);
        master_control_grid_->addWidget(master_motor_label_,6,0,1,2);
    }else{
        master_control_grid_->addWidget(master_title_,0,0,1,4);
        master_control_grid_->addWidget(link_all_motors_,1,0,1,2);
        master_control_grid_->addWidget(master_rpm_label_,1,2);
        master_control_grid_->addWidget(master_motor_speed_,1,3);
        master_control_grid_->addWidget(master_select_widget_,2,0,1,4);
        master_control_grid_->addWidget(master_motor_direction_,3,0,1,4);
        master_control_grid_->addWidget(master_motor_slider_,4,0,1,4);
        master_control_grid_->addWidget(master_motor_label_,5,0,1,4);
    }
    master_control_grid_->invalidate();
}
void MainWindow::reflowTrajectoryFields(int available_width){
    if(!trajectory_fields_grid_||trajectory_edits_.size()!=13||trajectory_labels_.size()!=13)return;
    const int cols=trajectory_columns_mode_==2 ? (available_width<590?1:2) : (available_width>645?2:1);
    if(trajectory_columns_mode_==cols)return;
    trajectory_columns_mode_=cols;
    for(int i=0;i<13;++i){
        trajectory_fields_grid_->removeWidget(trajectory_labels_[i]);
        trajectory_fields_grid_->removeWidget(trajectory_edits_[i]);
    }
    for(int i=0;i<13;++i){
        const int row=i/cols;
        const int col=(i%cols)*2;
        trajectory_fields_grid_->addWidget(trajectory_labels_[i],row,col);
        trajectory_fields_grid_->addWidget(trajectory_edits_[i],row,col+1);
    }
    trajectory_fields_grid_->setColumnStretch(1,1);
    trajectory_fields_grid_->setColumnStretch(3,cols==2?1:0);
    trajectory_fields_grid_->invalidate();
}
void MainWindow::updateCompactChrome(int available_width){
    const bool compact=available_width<1130;
    const bool narrow=available_width<790;
    if(brand_label_)brand_label_->setVisible(theme_index_==0 && available_width>=1350);
    if(control_target_label_)control_target_label_->setVisible(!compact);
    if(theme_label_)theme_label_->setVisible(!compact);
    if(plant_selector_){
        plant_selector_->setMinimumWidth(narrow?108:130);
        plant_selector_->setMaximumWidth(narrow?145:210);
    }
    if(theme_combo_){
        theme_combo_->setMinimumWidth(narrow?100:120);
        theme_combo_->setMaximumWidth(narrow?125:210);
    }
    if(nav_){
        nav_->setTabText(0,narrow?"LIVE":"Dashboard");
        nav_->setTabText(1,narrow?"CONTROL":"Signal Generator");
        nav_->setTabText(2,"System");
    }
    if(status_sim_connection_)status_sim_connection_->setMinimumWidth(narrow?94:126);
    if(status_real_connection_)status_real_connection_->setMinimumWidth(narrow?94:126);
    if(status_rec_)status_rec_->setVisible(!compact);
    if(status_control_)status_control_->setVisible(!narrow);
    if(status_samples_)status_samples_->setVisible(!compact);
    if(status_ros_)status_ros_->setVisible(!narrow);
    for(auto* sep:statusBar()->findChildren<QLabel*>("statusDivider"))sep->setVisible(!compact);
}
void MainWindow::setTheme(int index){
    if (index < 0 || index >= int(kThemes.size())) {
        return;
    }
    theme_index_ = index;
 QString file=QString(":/theme/%1.qss").arg(QStringList{"tron_ares","matlab","light","dark","tron_evolution","cyberpunk"}[index]);
 QFile qss(file);if(qss.open(QIODevice::ReadOnly|QIODevice::Text))qApp->setStyleSheet(QString::fromUtf8(qss.readAll()));
 const auto& t=kTheme[size_t(index)];
 startupTrace("Theme palette loaded");
 const QList<QColor> colors=index==0 ? QList<QColor>{
     QColor("#ff334d"),QColor("#ff9b77"),QColor("#ffd1bf"),QColor("#eb5676"),
     QColor("#ffa398"),QColor("#d72b3c"),QColor("#ffe2cb"),QColor("#f16b4c")}
     : QList<QColor>{QColor(t.red),QColor(t.blue),QColor(t.teal),QColor("#f8b869")};
 if(brand_label_)brand_label_->setVisible(index==0&&width()>=1350);
 // AresPanel and AresPage intentionally do not declare Q_OBJECT.
 // Their inherited Qt metaobjects belong to QFrame/QWidget. In particular,
 // findChildren<AresPage*>() matches arbitrary QWidgets and then tries to
 // call AresPage methods on those objects (undefined behavior / SIGSEGV).
 // dynamic_cast checks the actual C++ type and prevents invalid downcasts.
 for(QWidget* widget:findChildren<QWidget*>()){
     if(auto* panel=dynamic_cast<AresPanel*>(widget)){
         panel->setAresEnabled(index==0);
     }else if(auto* page=dynamic_cast<AresPage*>(widget)){
         page->setAresEnabled(index==0);
     }
 }
 // Optional glow uses an offscreen Qt effect buffer. Keep this off on
 // affected desktop drivers; the red/black QSS remains fully operational.
 if(qEnvironmentVariableIsSet("CEFIRO_GUI_ENABLE_ARES_FX"))
 for(auto* title:findChildren<QLabel*>("pageTitle")){
     if(index==0){
         if(!title->graphicsEffect()){
             auto* glow=new QGraphicsDropShadowEffect(title);
             glow->setBlurRadius(19);
             glow->setOffset(0,0);
             glow->setColor(QColor(245,22,48,128));
             title->setGraphicsEffect(glow);
         }
     }else if(title->graphicsEffect())title->setGraphicsEffect(nullptr);
 }
 startupTrace("Theme widget types verified");
 if(command_pad_)command_pad_->setAresTheme(index==0);
 for(auto* plot:dash_)if(plot)plot->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
  if(altitude_plot_)altitude_plot_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
  if(rpm_target_plot_)rpm_target_plot_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
  if(rpm_feedback_plot_)rpm_feedback_plot_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(preview_)preview_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(live_)live_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(view_)view_->setTheme(QColor(t.bg),QColor(t.fg),QColor(t.red),QColor(t.blue));
 startupTrace("Theme charts and attitude updated");
 if(plant_selector_){
     const QString accent=bridge_.isReal()?QString("#ffb57d"):t.red;
     plant_selector_->setStyleSheet(QString(
         "QComboBox#plantSelector { background:%1; color:%2; border:1px solid %2; "
         "border-radius:8px; padding:7px 12px; font-weight:800; min-height:19px; } "
         "QComboBox#plantSelector:hover { border:2px solid %2; } "
         "QComboBox#plantSelector:disabled { color:%3; border-color:%3; }")
         .arg(t.bg,accent,t.grid));
 }
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
