#include "cefiro_gui/attitude_command_pad.hpp"
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace cefiro::gui {
namespace {
constexpr std::array<const char*,3> kNames = {"ROLL", "PITCH", "YAW"};
constexpr std::array<const char*,3> kColors = {"#ff5252", "#4ee58c", "#448cff"};
constexpr double kHalfCircle = 180.0;
}

AttitudeCommandPad::AttitudeCommandPad(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(170);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setToolTip("Drag a colored ring to select roll/pitch/yaw. Reference limits "
               "are obtained from the SIM controller's CAD/actuator estimate. "
               "Yaw +/-180 deg covers all headings; this is not a 360 deg spin command. "
               "Wheel: 1 deg; double-click: zero. Enabled only after BALANCE: CAPTURED.");
}
void AttitudeCommandPad::setAngles(const Vec3& angles) { angles_ = angles; update(); }
void AttitudeCommandPad::setLimits(const Vec3& limits) {
    for (std::size_t i=0; i<3; ++i) {
        if (!std::isfinite(limits[i]) || limits[i] <= 0.0) return;
    }
    limits_ = limits;
    for (std::size_t i=0; i<3; ++i)
        angles_[i] = std::clamp(angles_[i], -limits_[i], limits_[i]);
    update();
}
void AttitudeCommandPad::setEnabledForEditing(bool enabled) {
    editing_enabled_ = enabled;
    if (!enabled) { dragging_ = false; active_axis_ = -1; }
    setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
    update();
}
void AttitudeCommandPad::setOnEdited(std::function<void(int,double)> callback) {
    on_edited_ = std::move(callback);
}
QPointF AttitudeCommandPad::center() const {
    return QPointF(width() * 0.47, height() * 0.50);
}
double AttitudeCommandPad::radiusStep() const {
    return std::clamp(std::min(width() * 0.73, height() * 0.90) / 6.0, 17.0, 45.0);
}
int AttitudeCommandPad::ringAt(const QPointF& position) const {
    const double dist = std::hypot(position.x() - center().x(), position.y() - center().y());
    const double step = radiusStep();
    int best_axis = -1;
    double best_distance = step * 0.62;
    for (int axis = 0; axis < 3; ++axis) {
        const double radius = step * (3 - axis);
        const double delta = std::abs(dist - radius);
        if (delta < best_distance) { best_distance = delta; best_axis = axis; }
    }
    return best_axis;
}
void AttitudeCommandPad::editAt(const QPointF& pos) {
    if (active_axis_ < 0 || !editing_enabled_) return;
    const QPointF delta = pos - center();
    // Use the ENTIRE circle for the SAFE reference range:
    // top = 0, right = +limit/2, left = -limit/2,
    // bottom clockwise = +limit, bottom counter-clockwise = -limit.
    // The single unavoidable +/- endpoint seam is at 6 o'clock.
    const double bearing = std::atan2(delta.x(), -delta.y()) * rad2deg;
    const double limit = limits_[std::size_t(active_axis_)];
    double angle = std::clamp(bearing / kHalfCircle * limit, -limit, limit);
    // Crossing the bottom seam must NOT jump directly from +limit to -limit.
    const double previous = angles_[std::size_t(active_axis_)];
    if (previous > .8*limit && angle < -.8*limit) angle = limit;
    if (previous < -.8*limit && angle > .8*limit) angle = -limit;
    if (std::abs(angle - angles_[std::size_t(active_axis_)]) < 0.02) return;
    angles_[std::size_t(active_axis_)] = angle;
    if (on_edited_) on_edited_(active_axis_, angle);
    update();
}
void AttitudeCommandPad::mousePressEvent(QMouseEvent* event) {
    if (editing_enabled_ && event->button() == Qt::LeftButton) {
        active_axis_ = ringAt(event->position());
        if (active_axis_ >= 0) { dragging_ = true; editAt(event->position()); event->accept(); return; }
    }
    QWidget::mousePressEvent(event);
}
void AttitudeCommandPad::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_) { editAt(event->position()); event->accept(); return; }
    if (editing_enabled_) setCursor(ringAt(event->position()) >= 0 ? Qt::PointingHandCursor : Qt::ArrowCursor);
    QWidget::mouseMoveEvent(event);
}
void AttitudeCommandPad::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) { dragging_ = false; event->accept(); return; }
    QWidget::mouseReleaseEvent(event);
}
void AttitudeCommandPad::mouseDoubleClickEvent(QMouseEvent* event) {
    const int axis = ringAt(event->position());
    if (editing_enabled_ && axis >= 0) {
        active_axis_ = axis;
        angles_[std::size_t(axis)] = 0.0;
        if (on_edited_) on_edited_(axis, 0.0);
        update(); event->accept(); return;
    }
    QWidget::mouseDoubleClickEvent(event);
}
void AttitudeCommandPad::wheelEvent(QWheelEvent* event) {
    if (!editing_enabled_) { event->ignore(); return; }
    const int axis = ringAt(event->position());
    if (axis < 0 || event->angleDelta().y() == 0) { event->ignore(); return; }
    active_axis_ = axis;
    const double lo = -limits_[std::size_t(axis)];
    const double hi =  limits_[std::size_t(axis)];
    const double updated = std::clamp(angles_[std::size_t(axis)] +
                  event->angleDelta().y() / 120.0, lo, hi);
    angles_[std::size_t(axis)] = updated;
    if (on_edited_) on_edited_(axis, updated);
    update(); event->accept();
}
void AttitudeCommandPad::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), Qt::transparent);
    const QPointF c = center();
    const double step = radiusStep();
    const double alpha = editing_enabled_ ? 1.0 : 0.42;
    p.setPen(QPen(QColor("#46586a"),1,Qt::DashLine));
    p.drawLine(QPointF(c.x(),c.y()-step*3.45),QPointF(c.x(),c.y()+step*3.45));
    p.drawLine(QPointF(c.x()-step*3.45,c.y()),QPointF(c.x()+step*3.45,c.y()));
    const std::array<double,3> a = {angles_[0], angles_[1], angles_[2]};
    for(int i=0;i<3;++i) {
        const QColor color(kColors[std::size_t(i)]);
        const double r = step*(3-i);
        p.setOpacity(alpha);
        p.setBrush(Qt::NoBrush);
        // A recessed rim and highlight give the concentric controller the
        // depth cues of a three-gimbal instrument without altering its hit zones.
        p.setPen(QPen(QColor("#121c2b"), 7));
        p.drawEllipse(c + QPointF(1.5, 2.0), r, r);
        p.setPen(QPen(QColor("#43556a"), 3));
        p.drawEllipse(c, r, r);
        p.setPen(QPen(color,3,Qt::SolidLine,Qt::RoundCap));
        const QRectF arc(c.x()-r,c.y()-r,2*r,2*r);
        const double start_angle = 90.0;
        const double sweep = std::clamp(a[std::size_t(i)] / limits_[std::size_t(i)] * kHalfCircle,
                                        -kHalfCircle,kHalfCircle);
        p.drawArc(arc,int(start_angle*16),int(-sweep*16));
        const double theta = (90.0-sweep)*deg2rad;
        const QPointF needle(c.x()+r*std::cos(theta),c.y()-r*std::sin(theta));
        p.setPen(QPen(color,2));
        p.drawLine(c,needle);
        p.setBrush(color);
        p.drawEllipse(needle,4,4);
        p.setOpacity(1.0);
    }
    // The small quadrotor silhouette identifies the common rotation center.
    // It is a control instrument, never the measured IMU attitude view.
    p.setOpacity(alpha);
    p.setPen(QPen(QColor("#a7c5e2"), 3, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(c + QPointF(-11, -11), c + QPointF(11, 11));
    p.drawLine(c + QPointF(-11, 11), c + QPointF(11, -11));
    for (const QPointF offset : {QPointF(-12,-12), QPointF(12,-12),
                                 QPointF(-12,12), QPointF(12,12)}) {
        p.setPen(QPen(QColor("#85a8c8"), 1));
        p.setBrush(QColor("#0b1420"));
        p.drawEllipse(c + offset, 4.5, 4.5);
    }
    p.setPen(QColor("#d4e5f5"));
    p.setBrush(QColor("#152638"));
    p.drawRoundedRect(QRectF(c.x()-7,c.y()-7,14,14),2.0,2.0);
    p.setOpacity(1.0);
    QFont font=p.font();font.setPixelSize(10);font.setBold(true);p.setFont(font);
    const int x=std::max(2,width()-89);
    const int h=std::min(27,std::max(17,height()/5));
    for(int i=0;i<3;++i){
        p.setPen(QColor(kColors[std::size_t(i)]));
        p.drawText(QRect(x, int(c.y())+(i-1)*h-10, 87,22),
                   Qt::AlignLeft|Qt::AlignVCenter,
                   QString("%1 %2°").arg(kNames[std::size_t(i)])
                                     .arg(angles_[std::size_t(i)],0,'f',1));
    }
}
}
