#pragma once
#include <QFrame>
#include <QElapsedTimer>
#include <QPoint>
#include <QPointF>
#include <QColor>
#include <QStringList>
#include <deque>
#include <vector>
class QChart;
class QChartView;
class QLabel;
class QLineSeries;
class QTimer;
class QToolButton;
class QValueAxis;
class QGraphicsLineItem;
class QGraphicsEllipseItem;
namespace cefiro::gui {
class ScopePlot final : public QFrame {
public:
    explicit ScopePlot(const QString& title,const QString& units,const QStringList& series,QWidget* parent=nullptr);
    ~ScopePlot() override;
    void append(int index,double t,double value);
    void setSeriesData(int index,const QList<QPointF>& points);
    void clear();
    void clearSeries(int index);
    void setWindowSeconds(double seconds);
    void setYRange(double lo,double hi);
    void fitY();
    void setAutoFitY(bool enabled=true,double minimumSpan=2.0);
    void setDashed(int index,bool yes=true);
    void setActive(bool enabled);
    void reTheme(const QColor& foreground,const QColor& background,const QColor& grid,const QList<QColor>& colors);
protected:
    bool eventFilter(QObject* object,QEvent* event) override;
private:
    void refresh();
    void syncSeriesVisibility();
    void zoomTime(double factor);
    void zoomY(double factor);
    void labelTime();
    void updateCursor(const QPoint& position);
    void hideCursor();
    void positionHoverLabel(const QPoint& position);
    struct SeriesData {std::deque<QPointF> points;QLineSeries* line{nullptr};bool dashed{false};bool rendered{true};};
    std::vector<SeriesData> data_;
    QChart* chart_{nullptr};QChartView* view_{nullptr};QValueAxis* x_{nullptr};QValueAxis* y_{nullptr};
    QTimer* refresh_timer_{nullptr};QLabel* time_label_{nullptr};
    QToolButton* auto_fit_button_{nullptr};
    double window_s_{15.0},retention_s_{600.0},y_lo_{-30.0},y_hi_{30.0};
    double minimum_y_span_{2.0};
    bool dirty_{true},auto_y_{false},active_{true};
    QColor fg_,bg_,grid_;QList<QColor> colors_;
    QGraphicsLineItem* cursor_line_{nullptr};
    std::vector<QGraphicsEllipseItem*> cursor_points_;
    QLabel* hover_label_{nullptr};
    QElapsedTimer hover_timer_;
    QPoint hover_position_{};
    bool hovering_{false};
};
}
