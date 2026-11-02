#pragma once
#include "cefiro_gui/models.hpp"
#include <QWidget>
#include <QColor>
namespace cefiro::gui {
class AttitudeView final : public QWidget {
public:
    explicit AttitudeView(QWidget* parent=nullptr);
    void setAttitude(const Quaternion& quaternion,bool valid);
    void setTheme(const QColor& bg,const QColor& fg,const QColor& accent,const QColor& secondary);
protected:
    void paintEvent(QPaintEvent*) override;
private:
    Quaternion q_{};
    bool valid_{false};
    QColor bg_{"#070a0f"},fg_{"#e8eef4"},accent_{"#ff5468"},secondary_{"#69b5f0"};
};
}
