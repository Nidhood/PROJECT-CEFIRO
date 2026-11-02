#include "cefiro_gui/attitude_view.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <QVector3D>
#include <cmath>
namespace cefiro::gui {
AttitudeView::AttitudeView(QWidget* p):QWidget(p){setMinimumSize(290,245);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);}
void AttitudeView::setAttitude(const Quaternion& q,bool v){q_=q;valid_=v;update();}
void AttitudeView::setTheme(const QColor& bg,const QColor& fg,const QColor& red,const QColor& blue){bg_=bg;fg_=fg;accent_=red;secondary_=blue;update();}
void AttitudeView::paintEvent(QPaintEvent*){
    QPainter p(this);p.setRenderHint(QPainter::Antialiasing);p.fillRect(rect(),bg_);
    const qreal sc=std::min(width(),height())*.37;const QPointF center(width()*.50,height()*.52);
    auto rotate=[&](const QVector3D& v)->QVector3D {
        const QVector3D u(float(q_.x),float(q_.y),float(q_.z));
        return v+2.f*float(q_.w)*QVector3D::crossProduct(u,v)+2.f*QVector3D::crossProduct(u,QVector3D::crossProduct(u,v));
    };
    auto project=[&](const QVector3D& v)->QPointF{
        const auto a=rotate(v);
        // Fixed axonometric camera with no flight dynamics.
        const double sx=.82*a.x()+.57*a.y();
        const double sy=.27*a.x()-.39*a.y()-.88*a.z();
        return center+QPointF(sc*sx,sc*sy);
    };
    const QColor muted=fg_.darker(250);
    p.setPen(QPen(muted,1,Qt::DashLine));
    for(int r=1;r<=3;++r){QRectF e(center.x()-r*sc/3.5,center.y()-r*sc/4.0,r*sc/1.75,r*sc/2.0);p.drawEllipse(e);}
    p.setPen(QPen(fg_.darker(180),1,Qt::DashLine));p.drawLine(QPointF(0,center.y()),QPointF(width(),center.y()));
    p.drawLine(QPointF(center.x(),0),QPointF(center.x(),height()));
    const QPointF o=project({0,0,0});
    const QVector3D ends[3]={{1.45f,0,0},{0,1.45f,0},{0,0,1.3f}};
    const QColor axiscolors[3]={accent_,secondary_,QColor("#00d8c6")};
    const QString axisnames[3]={"X / ROLL","Y / PITCH","Z / YAW"};
    for(int k=0;k<3;++k){const QPointF end=project(ends[k]);p.setPen(QPen(axiscolors[k],2));p.drawLine(o,end);p.drawText(end+QPointF(5,-5),axisnames[k]);}
    if(!valid_){p.setPen(fg_);p.setFont(QFont("Sans",11,QFont::Bold));p.drawText(rect(),Qt::AlignCenter,"WAITING FOR IMU ORIENTATION");return;}
    const QVector3D motors[4]={{-.72f,-.72f,0},{.72f,-.72f,0},{.72f,.72f,0},{-.72f,.72f,0}};
    const QColor arms[2]={secondary_,accent_};
    for(int i=0;i<2;++i){p.setPen(QPen(arms[i],8,Qt::SolidLine,Qt::RoundCap));p.drawLine(project(motors[i]),project(motors[i+2]));}
    // Low-vertex airframe model: motor housings, rotors, and nose marker.
    for(int i=0;i<4;++i){const QPointF c=project(motors[i]);p.setBrush(QColor("#152a3a"));p.setPen(QPen(fg_,1.8));p.drawEllipse(c,9,9);
        p.setPen(QPen(i%2?accent_:secondary_,2.4));p.drawEllipse(c,16,7);p.drawLine(c+QPointF(-18,0),c+QPointF(18,0));
    }
    p.setBrush(QColor("#273949"));p.setPen(QPen(fg_,2));p.drawRoundedRect(QRectF(o-QPointF(13,9),QSizeF(26,18)),4,4);
    const QPointF nose=project({.3f,0,.08f});p.setBrush(accent_);p.setPen(Qt::NoPen);p.drawEllipse(nose,3.5,3.5);
    p.setPen(fg_);p.setFont(QFont("Sans",9));
    const auto e=q_.eulerDeg();p.drawText(QRectF(8,height()-28,width()-16,20),Qt::AlignRight,
      QString("R %1°    P %2°    Y %3°").arg(e[0],0,'f',1).arg(e[1],0,'f',1).arg(e[2],0,'f',1));
}
}
