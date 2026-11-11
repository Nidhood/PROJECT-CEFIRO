#include "cefiro_gui/ares_chrome.hpp"

#include <QLinearGradient>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QPolygonF>
#include <QtGlobal>
#include <algorithm>
#include <cmath>

namespace cefiro::gui {

AresPanel::AresPanel(QWidget* parent):QFrame(parent){
    setAttribute(Qt::WA_StyledBackground,true);
}
void AresPanel::setAresEnabled(bool enabled){
    if(ares_enabled_==enabled)return;
    ares_enabled_=enabled;
    update();
}
void AresPanel::paintEvent(QPaintEvent* event){
    QFrame::paintEvent(event);
    // Safe default for older Qt6/NVIDIA stacks: keep Tron Ares QSS styling,
    // but do not run custom QPainter overlay code unless opted in.
    if(!qEnvironmentVariableIsSet("CEFIRO_GUI_ENABLE_ARES_FX") ||
       !ares_enabled_ || width()<40 || height()<40)return;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing,true);
    const QRectF outer=QRectF(rect()).adjusted(1.0,1.0,-2.0,-2.0);
    // Layered strokes approximate a subtle emissive glass-panel edge.
    for(int i=4;i>=1;--i){
        painter.setPen(QPen(QColor(255,26,53,i==1?115:11),double(i)));
        painter.setBrush(Qt::NoBrush);
        painter.drawRoundedRect(outer.adjusted(i,i,-i,-i),5.0,5.0);
    }
    const int w=width(),h=height();
    QLinearGradient top(0,0,w,0);
    top.setColorAt(0.0,QColor(255,39,55,180));
    top.setColorAt(0.42,QColor(255,45,70,60));
    top.setColorAt(0.85,QColor(255,30,60,20));
    top.setColorAt(1.0,QColor(255,45,55,165));
    painter.setPen(QPen(QBrush(top),1.6));
    painter.drawLine(QPointF(10,1.5),QPointF(w-10,1.5));
    painter.setPen(QPen(QColor(255,67,73,235),2.1));
    constexpr qreal offset=5.0;
    constexpr qreal reach=18.0;
    const QPointF corners[]={QPointF(offset,offset),QPointF(w-offset,offset),
                             QPointF(offset,h-offset),QPointF(w-offset,h-offset)};
    painter.drawLine(corners[0],corners[0]+QPointF(reach,0));
    painter.drawLine(corners[0],corners[0]+QPointF(0,reach));
    painter.drawLine(corners[1],corners[1]+QPointF(-reach,0));
    painter.drawLine(corners[1],corners[1]+QPointF(0,reach));
    painter.drawLine(corners[2],corners[2]+QPointF(reach,0));
    painter.drawLine(corners[2],corners[2]+QPointF(0,-reach));
    painter.drawLine(corners[3],corners[3]+QPointF(-reach,0));
    painter.drawLine(corners[3],corners[3]+QPointF(0,-reach));
    painter.setPen(QPen(QColor(255,72,83,90),1));
    painter.drawLine(QPointF(w-63,18),QPointF(w-20,18));
    for(int i=0;i<4;++i){
        painter.drawLine(QPointF(w-22-i*9,12),QPointF(w-26-i*9,21));
    }
}

AresPage::AresPage(QWidget* parent):QWidget(parent){
    setAttribute(Qt::WA_StyledBackground,true);
}
void AresPage::setAresEnabled(bool enabled){
    if(ares_enabled_==enabled)return;
    ares_enabled_=enabled;
    update();
}
void AresPage::paintEvent(QPaintEvent* event){
    QWidget::paintEvent(event);
    if(!qEnvironmentVariableIsSet("CEFIRO_GUI_ENABLE_ARES_FX") ||
       !ares_enabled_ || width()<100 || height()<90)return;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing,false);
    const qreal width_px=width(), height_px=height();
    const qreal horizon=height_px*0.55;
    // A very low-opacity perspective grid hints at the Ares terminal
    // atmosphere without competing with instruments or chart values.
    for(int i=1;i<=9;++i){
        const double fraction=double(i)/9.0;
        const qreal y=horizon+(height_px-horizon)*fraction*fraction;
        painter.setPen(QColor(240,21,44,int(10+24*fraction)));
        painter.drawLine(QPointF(0,y),QPointF(width_px,y));
    }
    painter.setPen(QColor(238,18,48,22));
    for(int i=-6;i<=6;++i){
        const qreal bottom_x=width_px*(0.5+double(i)/5.0);
        painter.drawLine(QPointF(width_px*0.5,horizon),QPointF(bottom_x,height_px));
    }
    painter.setPen(QColor(248,29,53,70));
    painter.drawLine(QPointF(0,1),QPointF(std::min<qreal>(width_px*.31,380.0),1));
}

} // namespace cefiro::gui
