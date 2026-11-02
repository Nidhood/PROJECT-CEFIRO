#include "cefiro_gui/main_window.hpp"
#include "cefiro_gui/scope_plot.hpp"
#include "cefiro_gui/attitude_view.hpp"
#include <QAction>
#include <QApplication>
#include <QCheckBox>
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
 setWindowTitle("CEFIRO | ATTITUDE CONTROL HMI");setMinimumSize(1280,820);resize(1600,930);
 clock_.start();publish_clock_.start();
 stack_=new QStackedWidget(this);setCentralWidget(stack_);
 buildToolbar();buildDashboard();buildControl();buildSystem();
 connectUi();setTheme(0);
 nav_->setCurrentIndex(0);stack_->setCurrentIndex(0);
 bridge_.start();refresh_=new QTimer(this);refresh_->setInterval(33);refresh_->setTimerType(Qt::CoarseTimer);
 connect(refresh_,&QTimer::timeout,this,[this]{tick();});refresh_->start();
 waveform_timer_=new QTimer(this);waveform_timer_->setInterval(20);waveform_timer_->setTimerType(Qt::PreciseTimer);
 connect(waveform_timer_,&QTimer::timeout,this,[this]{tickWaveform();});
 statusBar()->setSizeGripEnabled(false);
 status_ros_=label("ROS: WAITING");status_rec_=label("REC: IDLE");status_control_=label("CONTROL: DISABLED");status_samples_=label("SAMPLES: 0");
 for(auto* l:{status_ros_,status_rec_,status_control_,status_samples_}){statusBar()->addPermanentWidget(l);statusBar()->addPermanentWidget(label(" | "));}
}
MainWindow::~MainWindow(){shutdown();}
void MainWindow::shutdown(){if(shutting_down_)return;shutting_down_=true;
 if(refresh_)refresh_->stop();if(waveform_timer_)waveform_timer_->stop();wave_running_=false;
 if(command_active_)bridge_.emergencyStop();command_active_=false;
 stopRecording();bridge_.stop();
}
void MainWindow::buildToolbar(){
 auto* bar=new QToolBar(this);bar->setObjectName("topbar");bar->setMovable(false);bar->setFloatable(false);
 nav_=new QTabBar(bar);nav_->setObjectName("topTabs");nav_->setExpanding(false);nav_->setDrawBase(false);
 for(const QString& name:QStringList{"Dashboard","Signal Generator","System"})nav_->addTab(name);
 connect(nav_,&QTabBar::currentChanged,this,[this](int i){stack_->setCurrentIndex(i);control_tab_active_=(i==1);
   for(auto* p:dash_)if(p)p->setActive(i==0);if(preview_)preview_->setActive(i==1);if(live_)live_->setActive(i==1);});
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
    dash_[size_t(i)]=new ScopePlot("",units[i],keys,panel);dash_[size_t(i)]->setMinimumHeight(235);
    if(i<3){dash_[size_t(i)]->setDashed(0);dash_[size_t(i)]->setYRange(i==1?-95:-185,i==1?95:185);}
    if(i==3)dash_[size_t(i)]->setYRange(-30,30);
    if(i==4)dash_[size_t(i)]->setYRange(-15,15);
    if(i==5)dash_[size_t(i)]->setYRange(-80,80);
    inner->addWidget(dash_[size_t(i)],1);grid->addWidget(panel,i/2,i%2);
 }
 grid->setColumnStretch(0,1);grid->setColumnStretch(1,1);
 connect(clear_btn_,&QPushButton::clicked,this,[this]{for(auto* p:dash_)p->clear();if(live_)live_->clear();stats_.reset();});
}
void MainWindow::buildControl(){
 QVBoxLayout* body=nullptr;auto* page=contentPage(body,stack_);stack_->addWidget(page);
 auto* header=new QHBoxLayout;auto* title=new QVBoxLayout;
 title->addWidget(label("CEFIRO / CONTROL DECK","eyebrow"));title->addWidget(label("REFERENCE & CONTROL","pageTitle"));
 title->addWidget(label("External ROS 2 controller interface · manual setpoints · automatic excitation · measured attitude","pageSubtitle"));
 header->addLayout(title);header->addStretch();record_btn_=new QPushButton("DATA RECORDER",page);record_btn_->setObjectName("recordButton");header->addWidget(record_btn_);body->addLayout(header);
 auto* scroll=new QScrollArea(page);scroll->setWidgetResizable(true);scroll->setFrameShape(QFrame::NoFrame);body->addWidget(scroll,1);
 auto* container=new QWidget(scroll);scroll->setWidget(container);auto* grid=new QGridLayout(container);grid->setSpacing(8);grid->setContentsMargins(0,0,0,2);
 QVBoxLayout* arch=nullptr;auto* architecture=titledPanel("CONTROL ARCHITECTURE",arch,container);
 auto* ctrl=new QGridLayout;ctrl->addWidget(label("STRATEGY","eyebrow"),0,0);ctrl->addWidget(label("CONTROL MODE","eyebrow"),0,1);
 controller_=new QComboBox(architecture);controller_->addItems({"OPEN LOOP","PID","LQR","CUSTOM"});
 ctrl->addWidget(controller_,1,0);ctrl->addWidget(label("ATTITUDE · R/P/Y","metricValue"),1,1);arch->addLayout(ctrl);
 auto* source=new QHBoxLayout;automatic_btn_=new QPushButton("AUTOMATIC");manual_btn_=new QPushButton("MANUAL");
 for(auto* b:{automatic_btn_,manual_btn_}){b->setCheckable(true);b->setObjectName("sourceModeButton");source->addWidget(b,1);}automatic_btn_->setChecked(true);arch->addLayout(source);
 auto* actions=new QHBoxLayout;activate_btn_=new QPushButton("APPLY / ACTIVATE");activate_btn_->setObjectName("primaryAction");
 deactivate_btn_=new QPushButton("DEACTIVATE");deactivate_btn_->setObjectName("secondary");
 actions->addWidget(activate_btn_,2);actions->addWidget(deactivate_btn_,1);arch->addLayout(actions);
 applied_value_=label("Command output: DISABLED","metricValue");arch->addWidget(applied_value_);
 arch->addWidget(label("Strategy selection publishes a request. The external controller must acknowledge and implement it.","hint"));
 grid->addWidget(architecture,0,0);
 QVBoxLayout* ref=nullptr;auto* ref_panel=titledPanel("REFERENCE COMMAND",ref,container);
 ref_tabs_=new QTabWidget(ref_panel);ref_tabs_->setObjectName("referenceTabs");
 // MANUAL and AUTOMATIC share the reference panel and keep the original integrated tab approach.
 auto* settings=new QWidget(ref_tabs_);auto* settings_layout=new QVBoxLayout(settings);settings_layout->setContentsMargins(6,7,6,5);
 auto* manual_group=new QGroupBox("MANUAL AXES",settings);auto* mg=new QGridLayout(manual_group);
 for(int i=0;i<3;++i){mg->addWidget(label(kAxisNames[i]+" [deg]","eyebrow"),i,0);
    const double limit=i==1?89.0:180.0;manual_[size_t(i)]=spin(-limit,limit,0.,1.,2);mg->addWidget(manual_[size_t(i)],i,1);
    manual_labels_[size_t(i)]=label("0.00°","metricValue");mg->addWidget(manual_labels_[size_t(i)],i,2);
 }
 settings_layout->addWidget(manual_group);
 auto* generator=new QGroupBox("AUTOMATIC SIGNAL",settings);auto* gf=new QFormLayout(generator);
 axis_=new QComboBox(generator);axis_->addItems(kAxisNames);waveform_=new QComboBox(generator);waveform_->addItems(kTypes);
 gf->addRow("Axis",axis_);gf->addRow("Waveform",waveform_);
 auto* wavefields=new QGridLayout();
 const QStringList fieldnames={"Amplitude [deg]","Offset [deg]","Frequency [Hz]","End frequency [Hz]","Phase [deg]","Delay [s]","Duration [s]","Initial [deg]","Final [deg]","Rise time [s]","Pulse width [s]","Duty cycle [%]","Tau [s]"};
 std::array<QDoubleSpinBox**,13> ptrs={&amplitude_,&offset_,&frequency_,&end_frequency_,&phase_,&delay_,&duration_,&initial_,&final_,&rise_,&pulse_,&duty_,&tau_};
 const double default_values[]={10,0,.25,2,0,0,10,0,10,2,.5,50,1};
 for(int i=0;i<13;++i){const double lo=i==2||i==3?.001:(i==5||i==6||i==9||i==10||i==12?0.001:(i==11?0:-180));
    const double hi=(i==2||i==3?20:(i==5||i==6||i==9||i==10||i==12?3600:(i==11?100:180)));
    *ptrs[size_t(i)]=spin(lo,hi,default_values[i],i==2||i==3?.05:.5,3);
    auto* l=label(fieldnames[i],"eyebrow");const int col=i%2;const int row=i/2;
    wavefields->addWidget(l,row*2,col);wavefields->addWidget(*ptrs[size_t(i)],row*2+1,col);
 }
 gf->addRow(wavefields);repeat_=new QCheckBox("Repeat signal");gf->addRow(repeat_);settings_layout->addWidget(generator);settings_layout->addStretch();
 ref_tabs_->addTab(settings,"Parameters");
 auto* preview_tab=new QWidget(ref_tabs_);auto* pv=new QVBoxLayout(preview_tab);preview_=new ScopePlot("REFERENCE PREVIEW","deg",{"Reference"},preview_tab);preview_->setMinimumHeight(320);pv->addWidget(preview_);pv->addWidget(label("Preview uses exactly the same generator as the transmitted command.","hint"));ref_tabs_->addTab(preview_tab,"Reference Preview");
 auto* topology_tab=new QWidget(ref_tabs_);auto* tp=new QVBoxLayout(topology_tab);tp->setSpacing(16);
 for(const QString& text:QStringList{"REFERENCE COMMAND [roll, pitch, yaw]","↓   Vector3Stamped · radians   ↓","EXTERNAL ATTITUDE CONTROLLER [PID / LQR / CUSTOM]","↓   Actuator commands · external implementation   ↓","CEFIRO + GIMBAL / Gazebo Harmonic","↓   FMU sensors   ↓","IMU ORIENTATION · MAGNETOMETER · BAROMETER","↓   ROS 2 telemetry → GUI / recorder"}){
     auto* l=label(text,"topologyLine");l->setAlignment(Qt::AlignCenter);tp->addWidget(l);
 }tp->addStretch();ref_tabs_->addTab(topology_tab,"Control Topology");
 ref->addWidget(ref_tabs_,1);
 auto* options=new QHBoxLayout;auto_record_=new QCheckBox("Auto record",ref_panel);output_enabled_=new QCheckBox("Output enabled",ref_panel);options->addWidget(auto_record_);options->addWidget(output_enabled_);options->addStretch();ref->addLayout(options);
 auto* commands=new QHBoxLayout;start_btn_=new QPushButton("START TEST");start_btn_->setObjectName("primaryAction");stop_btn_=new QPushButton("SAFE STOP");stop_btn_->setObjectName("secondary");emergency_btn_=new QPushButton("EMERGENCY ZERO");emergency_btn_->setObjectName("emergencyButton");
 for(auto* b:{start_btn_,stop_btn_,emergency_btn_})commands->addWidget(b);ref->addLayout(commands);
 command_value_=label("Reference: R 0.00° · P 0.00° · Y 0.00°","metricValue");ref->addWidget(command_value_);
 grid->addWidget(ref_panel,1,0,2,1);
 QVBoxLayout* visual=nullptr;auto* visual_panel=titledPanel("ATTITUDE / IMU FRAME",visual,container);
 view_=new AttitudeView(visual_panel);visual->addWidget(view_,1);
 auto* readouts=new QGridLayout;for(int i=0;i<3;++i){readouts->addWidget(label(kAxisNames[i],"eyebrow"),0,i);rmse_values_[size_t(i)]=label("-- ° / RMSE --","metricValue");readouts->addWidget(rmse_values_[size_t(i)],1,i);}visual->addLayout(readouts);grid->addWidget(visual_panel,0,1);
 QVBoxLayout* liveinner=nullptr;auto* livepanel=titledPanel("LIVE ATTITUDE RESPONSE",liveinner,container);
 auto* ml=new QHBoxLayout;ml->addWidget(label("MONITOR AXIS","eyebrow"));monitor_combo_=new QComboBox;monitor_combo_->addItems(kAxisNames);ml->addWidget(monitor_combo_);ml->addStretch();metric_value_=label("RMSE --°  |  MAE --°  |  PEAK --°","metricValue");ml->addWidget(metric_value_);liveinner->addLayout(ml);
 live_=new ScopePlot("REFERENCE vs MEASURED","deg",{"Reference","Measured"},livepanel);live_->setMinimumHeight(285);live_->setDashed(0);live_->setYRange(-60,60);liveinner->addWidget(live_,1);
 grid->addWidget(livepanel,1,1);
 QVBoxLayout* gainsinner=nullptr;auto* gains=titledPanel("CONTROLLER PROFILES / EXPERIMENT CONFIGURATION",gainsinner,container);
 auto* gain_tabs=new QTabWidget(gains);pid_gains_=new QTableWidget(3,3,gain_tabs);pid_gains_->setHorizontalHeaderLabels({"Kp","Ki","Kd"});pid_gains_->setVerticalHeaderLabels(kAxisNames);pid_gains_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
 for(int r=0;r<3;++r)for(int c=0;c<3;++c)pid_gains_->setItem(r,c,new QTableWidgetItem(c==0?"0.500":"0.000"));
 gain_tabs->addTab(pid_gains_,"PID gains");lqr_gains_=new QTableWidget(3,6,gain_tabs);
 lqr_gains_->setHorizontalHeaderLabels({"φ","θ","ψ","p","q","r"});lqr_gains_->setVerticalHeaderLabels(kAxisNames);lqr_gains_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
 for(int r=0;r<3;++r)for(int c=0;c<6;++c)lqr_gains_->setItem(r,c,new QTableWidgetItem("0.000"));gain_tabs->addTab(lqr_gains_,"LQR / K 3×6");
 gainsinner->addWidget(gain_tabs);auto* send=new QPushButton("PUBLISH CONTROLLER PARAMETERS");send->setObjectName("secondary");gainsinner->addWidget(send);connect(send,&QPushButton::clicked,this,[this]{sendGains();});
 gainsinner->addWidget(label("Published gain arrays are a controller interface. They do not modify an external controller without a compatible subscriber.","hint"));grid->addWidget(gains,2,1);
 grid->setColumnStretch(0,5);grid->setColumnStretch(1,6);
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
 for(int i=0;i<3;++i){connect(manual_[size_t(i)],QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this,i](double value){
   manual_labels_[size_t(i)]->setText(f(value)+"°");
   if(manual_mode_){command_deg_[size_t(i)]=value;if(command_active_&&publish_clock_.elapsed()>45)publishCommand();}
 });}
 connect(activate_btn_,&QPushButton::clicked,this,[this]{startOutput();});
 connect(deactivate_btn_,&QPushButton::clicked,this,[this]{stopOutput(false);});
 connect(start_btn_,&QPushButton::clicked,this,[this]{startOutput();});
 connect(stop_btn_,&QPushButton::clicked,this,[this]{stopOutput(false);});
 connect(emergency_btn_,&QPushButton::clicked,this,[this]{stopOutput(true);});
 connect(output_enabled_,&QCheckBox::toggled,this,[this](bool on){if(!on && command_active_)stopOutput(false);});
 connect(record_btn_,&QPushButton::clicked,this,[this]{if(recorder_active_)stopRecording();else startRecording();});
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
void MainWindow::setManual(bool yes){manual_mode_=yes;manual_btn_->setChecked(yes);automatic_btn_->setChecked(!yes);
 waveform_->setEnabled(!yes);axis_->setEnabled(!yes);
 for(auto* d:{amplitude_,offset_,frequency_,end_frequency_,phase_,delay_,duration_,initial_,final_,rise_,pulse_,duty_,tau_})d->setEnabled(!yes);
 for(auto* m:manual_)m->setEnabled(yes);
 if(wave_running_){waveform_timer_->stop();wave_running_=false;}
 if(yes)for(int i=0;i<3;++i)command_deg_[size_t(i)]=manual_[size_t(i)]->value();
 if(command_active_)publishCommand();
}
void MainWindow::startOutput(){
 if(!output_enabled_->isChecked()){
  QMessageBox::information(this,"Output inhibited","Enable 'Output enabled' before sending controller commands.");return;
 }
 if(!bridge_.imuFresh()){
  QMessageBox::warning(this,"IMU unavailable","No recent IMU data. Reference generation is inhibited to avoid blind actuation.");return;
 }
 if(!bridge_.feedbackFresh()){
  const auto response=QMessageBox::warning(this,"Controller feedback unavailable",
    "No external controller has acknowledged the command interface.\n"
    "Start publishing references anyway? This does NOT verify any actuator is armed.",
    QMessageBox::Yes|QMessageBox::No,QMessageBox::No);
  if(response!=QMessageBox::Yes)return;
 }
 e_stop_=false;command_active_=true;bridge_.setEnabled(true);activate_btn_->setText("COMMANDS ENABLED");
 if(auto_record_->isChecked()&&!recorder_active_)startRecording();
 if(!manual_mode_){waveform_clock_.start();wave_running_=true;waveform_timer_->start();tickWaveform();}
 else{wave_running_=false;for(int i=0;i<3;++i)command_deg_[size_t(i)]=manual_[size_t(i)]->value();publishCommand();}
}
void MainWindow::stopOutput(bool emergency){
 if(waveform_timer_)waveform_timer_->stop();wave_running_=false;command_active_=false;e_stop_=emergency;
 command_deg_={0.,0.,0.};
 bridge_.publishReference(command_deg_,"ATTITUDE",controller_->currentText().toStdString());
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
 publish_clock_.restart();bridge_.publishReference(command_deg_,"ATTITUDE",controller_->currentText().toStdString());
 command_value_->setText(QString("Reference: R %1° · P %2° · Y %3°").arg(f(command_deg_[0]),f(command_deg_[1]),f(command_deg_[2])));
 applied_value_->setText("Reference request published · awaiting controller feedback");
}
void MainWindow::setMonitorAxis(int idx){if(idx<0||idx>2)return;monitor_axis_=idx;live_->clear();live_->setYRange(idx==1?-95.0:-185.0,idx==1?95.0:185.0);}
void MainWindow::tick(){
 const auto items=bridge_.drain();const bool fresh=bridge_.imuFresh();
 status_ros_->setText(fresh?"ROS: IMU ONLINE":"ROS: IMU WAITING / STALE");
 status_control_->setText(command_active_?"OUTPUT REQUEST: ENABLED":"CONTROL: DISABLED");
 if(bridge_.feedbackFresh())status_control_->setText("CTRL: "+QString::fromStdString(bridge_.controllerStatus()));
 for(size_t i=0;i<items.size();++i){const auto& s=items[i];++collected_;stats_.add(s);
    if(recorder_active_)saveRecord(s);
    onSample(s,i+1==items.size());
 }
 status_samples_->setText(QString("SAMPLES: %1").arg(collected_));
 if(have_sample_){for(int i=0;i<3;++i){
   rmse_values_[size_t(i)]->setText(QString("%1° / RMSE %2°").arg(f(last_sample_.attitude_deg[i],1)).arg(f(stats_.rmse(i),2)));}
   metric_value_->setText(QString("RMSE %1°  |  MAE %2°  |  PEAK %3°")
   .arg(f(stats_.rmse(monitor_axis_)),f(stats_.mae(monitor_axis_)),f(stats_.peak[monitor_axis_])));
   updateSystem(last_sample_);
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
 if(index<0||index>=int(kThemes.size()))return;theme_index_=index;
 QString file=QString(":/theme/%1.qss").arg(QStringList{"tron_ares","matlab","light","dark","tron_evolution","cyberpunk"}[index]);
 QFile qss(file);if(qss.open(QIODevice::ReadOnly|QIODevice::Text))qApp->setStyleSheet(QString::fromUtf8(qss.readAll()));
 const auto& t=kTheme[size_t(index)];
 const QList<QColor> colors={QColor(t.red),QColor(t.blue),QColor(t.teal),QColor("#f8b869")};
 for(auto* plot:dash_)if(plot)plot->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(preview_)preview_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(live_)live_->reTheme(QColor(t.fg),QColor(t.bg),QColor(t.grid),colors);
 if(view_)view_->setTheme(QColor(t.bg),QColor(t.fg),QColor(t.red),QColor(t.blue));
}
void MainWindow::startRecording(){
 if(recorder_active_)return;
 const QString file=QFileDialog::getSaveFileName(this,"Record CEFIRO experiment",
   QDir::homePath()+"/cefiro_attitude_"+QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")+".csv","CSV Files (*.csv)");
 if(file.isEmpty())return;
 recording_path_=file;recording_file_.setFileName(file);
 if(!recording_file_.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Truncate)){
   QMessageBox::critical(this,"Recorder",recording_file_.errorString());return;
 }
 recording_stream_.setDevice(&recording_file_);
 recording_stream_<<"ros_time_s,sequence,roll_deg,pitch_deg,yaw_deg,ref_roll_deg,ref_pitch_deg,ref_yaw_deg,"
 "error_roll_deg,error_pitch_deg,error_yaw_deg,p_deg_s,q_deg_s,r_deg_s,ax_m_s2,ay_m_s2,az_m_s2,"
 "Bx_uT,By_uT,Bz_uT,pressure_hPa,relative_alt_m,orientation_valid,mag_valid,baro_valid,strategy,source\n";
 recording_stats_.reset();recorder_active_=true;record_btn_->setText("STOP RECORDING");status_rec_->setText("REC: ACTIVE");
}
void MainWindow::stopRecording(){
 if(!recorder_active_)return;
 recorder_active_=false;recording_stream_.flush();recording_file_.close();
 record_btn_->setText("DATA RECORDER");status_rec_->setText("REC: IDLE");exportSummary();
}
void MainWindow::saveRecord(const Telemetry& s){
 recording_stats_.add(s);
 auto& out=recording_stream_;out.setRealNumberNotation(QTextStream::FixedNotation);out.setRealNumberPrecision(7);
 out<<s.ros_time_s<<','<<s.sequence;
 for(double v:s.attitude_deg)out<<','<<v;
 for(double v:s.setpoint_deg)out<<','<<v;
 for(int i=0;i<3;++i)out<<','<<(s.orientation_valid?angleErrorDeg(s.setpoint_deg[i],s.attitude_deg[i]):0.0);
 for(double v:s.angular_rate_deg_s)out<<','<<v;
 for(double v:s.accel_m_s2)out<<','<<v;
 for(double v:s.mag_uT)out<<','<<v;
 out<<','<<s.pressure_hPa<<','<<s.relative_alt_m<<','<<int(s.orientation_valid)
 <<','<<int(s.magnetometer_valid)<<','<<int(s.barometer_valid)<<','<<controller_->currentText()<<','<<(manual_mode_?"MANUAL":"AUTOMATIC")<<'\n';
 if(s.sequence%128==0)out.flush();
}
void MainWindow::exportSummary(){
 const QString name=QFileInfo(recording_path_).absolutePath()+"/"+QFileInfo(recording_path_).completeBaseName()+"_summary.txt";
 QFile file(name);if(!file.open(QIODevice::WriteOnly|QIODevice::Text|QIODevice::Truncate))return;
 QTextStream out(&file);out<<"CEFIRO ATTITUDE EXPERIMENT SUMMARY\n";
 out<<"CSV: "<<recording_path_<<"\n";
 out<<"Controller selection: "<<controller_->currentText()<<"\nSource: "<<(manual_mode_?"MANUAL":"AUTOMATIC")<<"\n";
 out<<"Note: controller acknowledgement and actual actuation must be verified separately.\n";
 out<<"Recorded orientation samples: "<<recording_stats_.count<<"\n";
 for(int i=0;i<3;++i)out<<kAxisNames[i]<<" RMSE="<<f(recording_stats_.rmse(i))<<"deg MAE="<<f(recording_stats_.mae(i))<<"deg Peak="<<f(recording_stats_.peak[i])<<"deg\n";
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
