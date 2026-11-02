#include "cefiro_gui/attitude_view.hpp"
#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <QPolygonF>
#include <QVector3D>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace cefiro::gui {
namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
struct Projected {
    QPointF xy;
    double depth{0};
};
struct Face {
    QPolygonF polygon;
    double depth{0};
    QColor fill;
    QColor line;
};
}

AttitudeView::AttitudeView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(290, 245);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);
    setToolTip("Drag to orbit the camera · Double-click to reset · Fixed auto-fit scale");
}
void AttitudeView::setAttitude(const Quaternion& quaternion, bool valid) {
    q_ = quaternion;
    valid_ = valid;
    update();
}
void AttitudeView::setTheme(const QColor& bg, const QColor& fg,
                            const QColor& accent, const QColor& secondary) {
    bg_ = bg;
    fg_ = fg;
    accent_ = accent;
    secondary_ = secondary;
    update();
}
void AttitudeView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        drag_origin_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}
void AttitudeView::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_) {
        const QPoint delta = event->pos() - drag_origin_;
        drag_origin_ = event->pos();
        azimuth_ = std::remainder(azimuth_ + delta.x() * 0.009, 2.0 * kPi);
        elevation_ = std::clamp(elevation_ - delta.y() * 0.009, -1.36, 1.36);
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}
void AttitudeView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = false;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}
void AttitudeView::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        azimuth_ = -0.80;
        elevation_ = 0.52;
        update();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}
void AttitudeView::wheelEvent(QWheelEvent* event) {
    // Deliberately consume wheel events. Model scale is always fitted to panel.
    event->accept();
}

void AttitudeView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), bg_);
    const double usable_width = std::max(80, width() - 130);
    const double usable_height = std::max(80, height() - 90);
    const double scale = std::min(usable_width, usable_height) / 3.55;
    const QPointF origin(width() * 0.5, height() * 0.51);

    // Camera coordinates: horizontal orbit and elevation above world XY.
    const float ca = float(std::cos(azimuth_));
    const float sa = float(std::sin(azimuth_));
    const float ce = float(std::cos(elevation_));
    const float se = float(std::sin(elevation_));
    const QVector3D right(ca, -sa, 0.f);
    const QVector3D up(-sa * se, -ca * se, ce);
    const QVector3D forward(sa * ce, ca * ce, se);
    auto bodyToWorld = [this](const QVector3D& v) {
        if (!valid_) return v;
        const QVector3D u(float(q_.x), float(q_.y), float(q_.z));
        return v + 2.f * float(q_.w) * QVector3D::crossProduct(u, v)
                 + 2.f * QVector3D::crossProduct(u, QVector3D::crossProduct(u, v));
    };
    auto projectWorld = [&](const QVector3D& world) -> Projected {
        return {origin + QPointF(scale * QVector3D::dotProduct(world, right),
                                 -scale * QVector3D::dotProduct(world, up)),
                QVector3D::dotProduct(world, forward)};
    };
    auto projectBody = [&](const QVector3D& body) { return projectWorld(bodyToWorld(body)); };

    // Fixed-size world reference grid, independent of attitude and camera orbit.
    painter.setPen(QPen(fg_.darker(350), 1, Qt::DashLine));
    const std::array<QVector3D, 4> grid = {{{-1.32f, -1.32f, -0.23f},
                                             {1.32f, -1.32f, -0.23f},
                                             {1.32f, 1.32f, -0.23f},
                                             {-1.32f, 1.32f, -0.23f}}};
    for (int i = 0; i < 4; ++i) {
        painter.drawLine(projectWorld(grid[i]).xy,
                         projectWorld(grid[(i + 1) % 4]).xy);
    }
    painter.setPen(QPen(fg_.darker(280), 1, Qt::DotLine));
    painter.drawLine(projectWorld({-1.32f, 0.f, -0.23f}).xy,
                     projectWorld({1.32f, 0.f, -0.23f}).xy);
    painter.drawLine(projectWorld({0.f, -1.32f, -0.23f}).xy,
                     projectWorld({0.f, 1.32f, -0.23f}).xy);

    if (!valid_) {
        painter.setPen(fg_);
        painter.setFont(QFont("Sans", 10, QFont::Bold));
        painter.drawText(rect(), Qt::AlignCenter, "WAITING FOR IMU ORIENTATION");
    } else {
        const std::array<QVector3D, 4> motors = {{{-0.72f, -0.72f, 0.0f},
                                                  {0.72f, -0.72f, 0.0f},
                                                  {0.72f, 0.72f, 0.0f},
                                                  {-0.72f, 0.72f, 0.0f}}};
        // Render arms as thick projected 3D struts, with a subtle edge highlight.
        for (int i = 0; i < 2; ++i) {
            const QPointF a = projectBody(motors[i]).xy;
            const QPointF b = projectBody(motors[i + 2]).xy;
            painter.setPen(QPen(QColor("#102431"), 12, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(a, b);
            painter.setPen(QPen(i == 0 ? secondary_ : accent_, 7,
                                Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(a, b);
        }

        std::vector<Face> faces;
        faces.reserve(36);
        // A six-faced avionics enclosure gives the airframe visible 3D volume.
        const std::array<QVector3D, 8> cube = {{{-.22f, -.16f, -.10f},
                                               {.22f, -.16f, -.10f},
                                               {.22f, .16f, -.10f},
                                               {-.22f, .16f, -.10f},
                                               {-.22f, -.16f, .12f},
                                               {.22f, -.16f, .12f},
                                               {.22f, .16f, .12f},
                                               {-.22f, .16f, .12f}}};
        const std::array<std::array<int, 4>, 6> box_sides = {{{{0, 1, 2, 3}},
            {{4, 5, 6, 7}}, {{0, 1, 5, 4}}, {{1, 2, 6, 5}},
            {{2, 3, 7, 6}}, {{3, 0, 4, 7}}}};
        const std::array<QColor, 6> box_colors = {{QColor("#182c3a"),
            QColor("#2b4658"), QColor("#203747"), QColor("#142733"),
            QColor("#274153"), QColor("#344b5c")}};
        for (std::size_t i = 0; i < box_sides.size(); ++i) {
            Face face;
            face.fill = box_colors[i];
            face.line = fg_.darker(155);
            for (int vertex : box_sides[i]) {
                const auto pt = projectBody(cube[std::size_t(vertex)]);
                face.polygon << pt.xy;
                face.depth += pt.depth / 4.0;
            }
            faces.push_back(std::move(face));
        }
        // Circular propeller planes and raised motor caps are real oriented
        // polygons, not flat circles pasted over the 2D screen.
        for (std::size_t i = 0; i < motors.size(); ++i) {
            const QColor ring_color = (i % 2 == 0) ? secondary_ : accent_;
            Face rotor;
            rotor.fill = ring_color;
            rotor.fill.setAlpha(52);
            rotor.line = ring_color;
            for (int k = 0; k < 24; ++k) {
                const double theta = (2.0 * kPi * k) / 24.0;
                const QVector3D v = motors[i] + QVector3D(
                    float(.235 * std::cos(theta)), float(.235 * std::sin(theta)), .105f);
                const auto pt = projectBody(v);
                rotor.polygon << pt.xy;
                rotor.depth += pt.depth / 24.0;
            }
            faces.push_back(std::move(rotor));
            Face cap;
            cap.fill = QColor("#172b3c");
            cap.line = fg_.darker(125);
            for (int k = 0; k < 16; ++k) {
                const double theta = 2.0 * kPi * k / 16.0;
                const auto pt = projectBody(motors[i] + QVector3D(
                    float(.105 * std::cos(theta)), float(.105 * std::sin(theta)), .13f));
                cap.polygon << pt.xy;
                cap.depth += pt.depth / 16.0;
            }
            faces.push_back(std::move(cap));
        }
        std::stable_sort(faces.begin(), faces.end(),
            [](const Face& a, const Face& b) { return a.depth < b.depth; });
        for (const auto& face : faces) {
            painter.setBrush(face.fill);
            painter.setPen(QPen(face.line, 1.5));
            painter.drawPolygon(face.polygon);
        }
        // Nose marking: body +X indicates the drone's forward direction.
        const QPointF nose = projectBody({.24f, 0.f, .14f}).xy;
        painter.setBrush(QColor("#ffe59a"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(nose, 4, 4);
    }

    // Three colored BODY axes (conventional RGB) change with IMU quaternion.
    const std::array<QVector3D, 3> axes = {{{1.40f, 0.f, 0.f},
                                            {0.f, 1.40f, 0.f},
                                            {0.f, 0.f, 1.30f}}};
    const std::array<QColor, 3> colors = {{QColor("#ff5252"),
                                           QColor("#4ee58c"),
                                           QColor("#448cff")}};
    const std::array<QString, 3> labels = {{"X / ROLL", "Y / PITCH", "Z / YAW"}};
    const QPointF center = projectBody({0.f, 0.f, 0.f}).xy;
    painter.setFont(QFont("Sans", 9, QFont::Bold));
    for (int i = 0; i < 3; ++i) {
        const QPointF end = projectBody(axes[std::size_t(i)]).xy;
        painter.setPen(QPen(colors[std::size_t(i)], 2.8));
        painter.drawLine(center, end);
        painter.setBrush(colors[std::size_t(i)]);
        painter.drawEllipse(end, 3.0, 3.0);
        painter.setPen(colors[std::size_t(i)]);
        const QPointF direction = end - center;
        const QPointF label_at = end + QPointF(direction.x() >= 0 ? 6 : -72,
                                              direction.y() >= 0 ? 14 : -6);
        painter.drawText(QRectF(label_at, QSizeF(88, 20)), labels[std::size_t(i)]);
    }

    painter.setFont(QFont("Sans", 9));
    painter.setPen(fg_.darker(145));
    painter.drawText(QRectF(10, 7, width() - 20, 19), Qt::AlignLeft,
                     "DRAG TO ORBIT  ·  DOUBLE-CLICK TO RESET");
    painter.drawText(QRectF(10, height() - 25, width() / 2.0, 19), Qt::AlignLeft,
                     QString("CAM  %1° / %2°")
                         .arg(azimuth_ * 180.0 / kPi, 0, 'f', 0)
                         .arg(elevation_ * 180.0 / kPi, 0, 'f', 0));
    if (valid_) {
        const auto angles = q_.eulerDeg();
        painter.setPen(fg_);
        painter.drawText(QRectF(15, height() - 25, width() - 25, 19), Qt::AlignRight,
            QString("R %1°   P %2°   Y %3°")
                .arg(angles[0], 0, 'f', 1)
                .arg(angles[1], 0, 'f', 1)
                .arg(angles[2], 0, 'f', 1));
    }
}

}  // namespace cefiro::gui
