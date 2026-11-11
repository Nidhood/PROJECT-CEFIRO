#pragma once

#include <QFrame>
#include <QWidget>

class QPaintEvent;

namespace cefiro::gui {

// Lightweight neon chrome rendered directly into the frame. No expensive
// whole-widget blur is applied to frequently updating telemetry charts.
class AresPanel final : public QFrame {
public:
    explicit AresPanel(QWidget* parent = nullptr);
    void setAresEnabled(bool enabled);
protected:
    void paintEvent(QPaintEvent* event) override;
private:
    bool ares_enabled_{false};
};

// Static subdued perspective grid below the actual controls.
class AresPage final : public QWidget {
public:
    explicit AresPage(QWidget* parent = nullptr);
    void setAresEnabled(bool enabled);
protected:
    void paintEvent(QPaintEvent* event) override;
private:
    bool ares_enabled_{false};
};

}  // namespace cefiro::gui
