#include "cefiro_gui/scope_plot.hpp"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLegend>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QTimer>
#include <QToolButton>
#include <QToolTip>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
namespace cefiro::gui {
ScopePlot::ScopePlot(const QString& title,const QString& units,const QStringList& series,QWidget* parent):QFrame(parent){
    setObjectName("chartFrame");setMinimumHeight(170);
    auto* root=new QVBoxLayout(this);root->setContentsMargins(4,2,4,4);root->setSpacing(2);
    auto* header=new QHBoxLayout;header->setSpacing(3);
    auto* name=new QLabel(title,this);name->setObjectName("chartTitle");header->addWidget(name);header->addStretch();
    auto make=[&](const QString& label,const QString& help){auto* b=new QToolButton(this);b->setObjectName("chartToolButton");b->setText(label);b->setToolTip(help);b->setFixedHeight(22);header->addWidget(b);return b;};
    auto* t_minus=make("T−","Zoom time in");time_label_=new QLabel("15 s",this);time_label_->setObjectName("chartWindowLabel");header->addWidget(time_label_);
    auto* t_plus=make("T+","Zoom time out");auto* y_minus=make("Y−","Zoom vertically in");auto* y_plus=make("Y+","Zoom vertically out");auto* fit=make("FIT","Fit visible values");
    root->addLayout(header);
    chart_=new QChart;chart_->setAnimationOptions(QChart::NoAnimation);chart_->setMargins(QMargins(0,0,0,0));
    chart_->legend()->setVisible(series.size()>1);chart_->legend()->setAlignment(Qt::AlignTop);
    x_=new QValueAxis(chart_);y_=new QValueAxis(chart_);
    x_->setTitleText("t [s]");x_->setLabelFormat("%.1f");x_->setTickCount(5);
    y_->setTitleText(units);y_->setLabelFormat("%.3g");y_->setTickCount(5);
    chart_->addAxis(x_,Qt::AlignBottom);chart_->addAxis(y_,Qt::AlignLeft);
    for(const auto& text:series){auto* line=new QLineSeries(chart_);line->setName(text);chart_->addSeries(line);line->attachAxis(x_);line->attachAxis(y_);data_.push_back({{},line,false});}
    view_=new QChartView(chart_,this);view_->setObjectName("scopeChartView");view_->setRenderHint(QPainter::Antialiasing,false);
    view_->viewport()->setMouseTracking(true);view_->viewport()->installEventFilter(this);root->addWidget(view_,1);
    connect(t_minus,&QToolButton::clicked,this,[this]{zoomTime(0.75);});
    connect(t_plus,&QToolButton::clicked,this,[this]{zoomTime(4.0/3.0);});
    connect(y_minus,&QToolButton::clicked,this,[this]{zoomY(.8);});
    connect(y_plus,&QToolButton::clicked,this,[this]{zoomY(1.25);});
    connect(fit,&QToolButton::clicked,this,[this]{fitY();});
    refresh_timer_=new QTimer(this);refresh_timer_->setInterval(50);refresh_timer_->setTimerType(Qt::CoarseTimer);
    connect(refresh_timer_,&QTimer::timeout,this,[this]{refresh();});refresh_timer_->start();
    setYRange(-30,30);x_->setRange(0,window_s_);
    reTheme(QColor("#e8eef4"),QColor("#070a0f"),QColor("#213549"),{QColor("#FF5468"),QColor("#69B5F0"),QColor("#00D8C6"),QColor("#F8B869")});
}
ScopePlot::~ScopePlot(){refresh_timer_->stop();view_->viewport()->removeEventFilter(this);}
void ScopePlot::append(int i,double t,double v){
    if(i<0||i>=int(data_.size())||!std::isfinite(t)||!std::isfinite(v))return;
    auto& pts=data_[size_t(i)].points;pts.emplace_back(t,v);
    while(!pts.empty()&&(pts.front().x()<t-retention_s_||pts.size()>5000))pts.pop_front();dirty_=true;
}
void ScopePlot::setSeriesData(int i,const QList<QPointF>& pts){
    if(i<0||i>=int(data_.size()))return;
    auto& p=data_[size_t(i)].points;p.clear();for(const auto& v:pts)if(std::isfinite(v.x())&&std::isfinite(v.y()))p.push_back(v);dirty_=true;
}
void ScopePlot::clear(){for(auto& s:data_){s.points.clear();s.line->clear();}x_->setRange(0,window_s_);dirty_=false;}
void ScopePlot::setWindowSeconds(double s){window_s_=std::clamp(s,1.0,60.0);labelTime();dirty_=true;}
void ScopePlot::labelTime(){time_label_->setText(QString::number(window_s_,'f',window_s_<10?1:0)+" s");}
void ScopePlot::setYRange(double lo,double hi){if(hi<=lo)return;y_lo_=lo;y_hi_=hi;auto_y_=false;y_->setRange(lo,hi);}
void ScopePlot::fitY(){double lo=std::numeric_limits<double>::infinity(),hi=-lo;const double left=x_->min(),right=x_->max();
 for(const auto& s:data_)for(const auto& p:s.points)if(p.x()>=left&&p.x()<=right){lo=std::min(lo,p.y());hi=std::max(hi,p.y());}
 if(!std::isfinite(lo))return;const double pad=std::max(1.0,(hi-lo)*.12);setYRange(lo-pad,hi+pad);
}
void ScopePlot::setDashed(int i,bool b){if(i>=0&&i<int(data_.size())){data_[size_t(i)].dashed=b;reTheme(fg_,bg_,grid_,colors_);}}
void ScopePlot::setActive(bool b){active_=b;if(b)refresh_timer_->start();else refresh_timer_->stop();}
void ScopePlot::zoomTime(double f){setWindowSeconds(window_s_*f);}
void ScopePlot::zoomY(double f){const double m=(y_lo_+y_hi_)*.5,d=(y_hi_-y_lo_)*f*.5;setYRange(m-d,m+d);}
void ScopePlot::refresh(){if(!dirty_||!active_)return;dirty_=false;
 double max_t=0.0;bool any=false;for(const auto& s:data_)if(!s.points.empty()){any=true;max_t=std::max(max_t,s.points.back().x());}if(!any)return;
 x_->setRange(std::max(0.0,max_t-window_s_),std::max(window_s_,max_t));
 for(auto& s:data_){QList<QPointF> pts;pts.reserve(int(s.points.size()));for(const auto& p:s.points)if(p.x()>=x_->min()-1.0)pts.push_back(p);s.line->replace(pts);}
 if(auto_y_)fitY();
}
void ScopePlot::reTheme(const QColor& fg,const QColor& bg,const QColor& grid,const QList<QColor>& colors){
 fg_=fg;bg_=bg;grid_=grid;colors_=colors;chart_->setBackgroundBrush(bg);chart_->setPlotAreaBackgroundBrush(bg);chart_->setPlotAreaBackgroundVisible(true);
 chart_->legend()->setLabelColor(fg);chart_->setBackgroundPen(QPen(Qt::NoPen));
 for(auto* axis:{x_,y_}){axis->setLabelsColor(fg);axis->setTitleBrush(fg);axis->setGridLineColor(grid);axis->setLinePenColor(grid);}
 for(size_t i=0;i<data_.size();++i){QPen p(colors[int(i)%colors.size()],1.8);if(data_[i].dashed)p.setStyle(Qt::DashLine);p.setCosmetic(true);data_[i].line->setPen(p);}
}
bool ScopePlot::eventFilter(QObject* obj,QEvent* evt){
 if(obj!=view_->viewport())return QFrame::eventFilter(obj,evt);
 if(evt->type()==QEvent::Wheel){auto* e=static_cast<QWheelEvent*>(evt);zoomTime(e->angleDelta().y()>0?.75:4.0/3.0);return true;}
 if(evt->type()==QEvent::MouseMove){const auto* e=static_cast<QMouseEvent*>(evt);auto* s=data_.empty()?nullptr:data_.front().line;if(!s)return false;
    const QPointF chart_pos=chart_->mapFromScene(view_->mapToScene(e->pos()));
    const QPointF coord=chart_->mapToValue(chart_pos,s);
    if(!std::isfinite(coord.x()))return false;
    QString tooltip=QString("t = %1 s").arg(coord.x(),0,'f',2);
    for(const auto& series:data_){const auto& pts=series.points;if(pts.empty())continue;
        auto iter=std::lower_bound(pts.begin(),pts.end(),coord.x(),[](const QPointF& a,double t){return a.x()<t;});
        if(iter==pts.end())iter=std::prev(pts.end());
        tooltip+=QString("\n%1: %2").arg(series.line->name()).arg(iter->y(),0,'f',3);
    }
    QToolTip::showText(e->globalPosition().toPoint(),tooltip,view_);return false;
 }
 if(evt->type()==QEvent::Leave)QToolTip::hideText();return QFrame::eventFilter(obj,evt);
}
}
