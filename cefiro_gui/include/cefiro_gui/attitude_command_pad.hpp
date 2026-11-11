#pragma once

#include "cefiro_gui/models.hpp"
#include <QWidget>
#include <QColor>
#include <functional>

class QMouseEvent;
class QWheelEvent;

namespace cefiro::gui {

// A compact interactive attitude reference instrument. It never drives actuators
// directly; the owner decides whether an edited reference may be published.
class AttitudeCommandPad final : public QWidget {
public:
    explicit AttitudeCommandPad(QWidget* parent = nullptr);
    void setAngles(const Vec3& angles);
    void setAresTheme(bool enabled);
    void setLimits(const Vec3& limits);
    void setEnabledForEditing(bool enabled);
    void setOnEdited(std::function<void(int, double)> callback);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void editAt(const QPointF& pos);
    int ringAt(const QPointF& pos) const;
    double radiusStep() const;
    QPointF center() const;
    Vec3 angles_{};
    Vec3 limits_{{5.0,5.0,10.0}};
    bool ares_theme_{false};
    bool editing_enabled_{false};
    bool dragging_{false};
    int active_axis_{-1};
    std::function<void(int, double)> on_edited_;
};

} // namespace cefiro::gui
