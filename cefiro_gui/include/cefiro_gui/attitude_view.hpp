#pragma once
#include "cefiro_gui/models.hpp"
#include <QColor>
#include <QPoint>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

namespace cefiro::gui {

// Lightweight true-3D orthographic renderer: no OpenGL / Gazebo dependencies.
// Mouse movement changes the camera only; IMU attitude changes the airframe.
class AttitudeView final : public QWidget {
public:
    explicit AttitudeView(QWidget* parent = nullptr);
    void setAttitude(const Quaternion& quaternion, bool valid);
    void setTheme(const QColor& bg, const QColor& fg,
                  const QColor& accent, const QColor& secondary);
protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
private:
    Quaternion q_{};
    bool valid_{false};
    bool dragging_{false};
    QPoint drag_origin_{};
    double azimuth_{-0.80};
    double elevation_{0.52};
    QColor bg_{"#070a0f"};
    QColor fg_{"#e8eef4"};
    QColor accent_{"#ff5468"};
    QColor secondary_{"#69b5f0"};
};

}  // namespace cefiro::gui
