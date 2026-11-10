#include "cefiro_gui/scope_plot.hpp"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QEvent>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QLabel>
#include <QLegend>
#include <QtCharts/QLegendMarker>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QSignalBlocker>
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
    auto* t_plus=make("T+","Zoom time out");
    auto* y_minus=make("Y−","Zoom vertically in");
    auto* y_plus=make("Y+","Zoom vertically out");
    auto* fit=make("FIT","Fit the visible data once without changing auto-fit mode");
    auto_fit_button_=make("AUTO Y: OFF","Follow the visible samples with automatic vertical scaling (independent for this plot)");
    auto_fit_button_->setObjectName("chartAutoFitButton");
    auto_fit_button_->setCheckable(true);
    auto_fit_button_->setChecked(false);
    auto_fit_button_->setMinimumWidth(91);
    root->addLayout(header);
    chart_=new QChart;chart_->setAnimationOptions(QChart::NoAnimation);chart_->setMargins(QMargins(0,0,0,0));
    chart_->legend()->setVisible(series.size()>1);chart_->legend()->setAlignment(Qt::AlignTop);
    x_=new QValueAxis(chart_);y_=new QValueAxis(chart_);
    x_->setTitleText("t [s]");x_->setLabelFormat("%.1f");x_->setTickCount(5);
    y_->setTitleText(units);
    y_->setLabelFormat(units == "hPa" ? "%.2f" : units == "DShot" ? "%.0f" : "%.2f");
    y_->setTickCount(5);
    chart_->addAxis(x_,Qt::AlignBottom);chart_->addAxis(y_,Qt::AlignLeft);
    for(const auto& text:series){auto* line=new QLineSeries(chart_);line->setName(text);chart_->addSeries(line);line->attachAxis(x_);line->attachAxis(y_);data_.push_back({{},line,false});}
    view_=new QChartView(chart_,this);view_->setObjectName("scopeChartView");view_->setRenderHint(QPainter::Antialiasing,false);
    view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view_->setCacheMode(QGraphicsView::CacheNone);
    view_->setMouseTracking(true);
    view_->viewport()->setMouseTracking(true);
    view_->viewport()->installEventFilter(this);
    root->addWidget(view_,1);
    // A viewport child follows every mouse movement. QToolTip::showText is
    // intentionally not used: native tooltips may stay in place on X11 / Wayland.
    hover_label_ = new QLabel(view_->viewport());
    hover_label_->setObjectName("scopeHoverLabel");
    hover_label_->setTextFormat(Qt::RichText);
    hover_label_->setAttribute(Qt::WA_TransparentForMouseEvents);
    hover_label_->setAttribute(Qt::WA_ShowWithoutActivating);
    hover_label_->setMargin(8);
    hover_label_->setWordWrap(true);
    hover_label_->setMaximumWidth(310);
    hover_label_->setMinimumWidth(118);
    hover_label_->setStyleSheet("QLabel#scopeHoverLabel { background-color: #101b2a; "
                               "color: #e8eef4; border: 1px solid #496077; "
                               "border-radius: 5px; padding: 2px; }");
    hover_label_->hide();
    cursor_line_ = chart_->scene()->addLine(QLineF(), QPen(QColor("#7990a5"), 1, Qt::DashLine));
    cursor_line_->setZValue(100);
    cursor_line_->hide();
    for (std::size_t i = 0; i < data_.size(); ++i) {
        auto* marker = chart_->scene()->addEllipse(-4, -4, 8, 8,
                                                   QPen(Qt::NoPen), QBrush(Qt::white));
        marker->setZValue(101);
        marker->hide();
        cursor_points_.push_back(marker);
    }
    hover_timer_.start();
    connect(t_minus,&QToolButton::clicked,this,[this]{zoomTime(0.75);});
    connect(t_plus,&QToolButton::clicked,this,[this]{zoomTime(4.0/3.0);});
    connect(y_minus,&QToolButton::clicked,this,[this]{zoomY(.8);});
    connect(y_plus,&QToolButton::clicked,this,[this]{zoomY(1.25);});
    // FIT is a one-shot operation; it must not silently re-enable AUTO Y.
    connect(fit,&QToolButton::clicked,this,[this]{fitY();});
    connect(auto_fit_button_,&QToolButton::toggled,this,[this](bool checked){
        setAutoFitY(checked,minimum_y_span_);
    });
    refresh_timer_=new QTimer(this);refresh_timer_->setInterval(50);refresh_timer_->setTimerType(Qt::CoarseTimer);
    connect(refresh_timer_,&QTimer::timeout,this,[this]{refresh();});refresh_timer_->start();
    setYRange(-30,30);x_->setRange(0,window_s_);
    syncSeriesVisibility();
    reTheme(QColor("#e8eef4"),QColor("#070a0f"),QColor("#213549"),{QColor("#FF5468"),QColor("#69B5F0"),QColor("#00D8C6"),QColor("#F8B869")});
}
ScopePlot::~ScopePlot(){
    refresh_timer_->stop();
    view_->viewport()->removeEventFilter(this);
    hideCursor();
    if (auto* scene = chart_->scene()) {
        if (cursor_line_) { scene->removeItem(cursor_line_); delete cursor_line_; cursor_line_ = nullptr; }
        for (auto* item : cursor_points_) {
            scene->removeItem(item);
            delete item;
        }
    }
}
void ScopePlot::append(int index,double t,double value){
    if(index<0 || index>=static_cast<int>(data_.size()) ||
       !std::isfinite(t) || !std::isfinite(value)) return;
    auto& points=data_[static_cast<std::size_t>(index)].points;
    points.emplace_back(t,value);
    while(!points.empty() &&
          (points.front().x()<t-retention_s_ || points.size()>20000)) {
        points.pop_front();
    }
    dirty_=true;
}
void ScopePlot::setSeriesData(int index,const QList<QPointF>& points){
    if(index<0 || index>=static_cast<int>(data_.size())) return;
    auto& buffer=data_[static_cast<std::size_t>(index)].points;
    buffer.clear();
    for(const auto& point:points){
        if(std::isfinite(point.x()) && std::isfinite(point.y())){
            buffer.push_back(point);
        }
    }
    syncSeriesVisibility();
    dirty_=true;
}
void ScopePlot::clear(){
    hideCursor();
    for(auto& series:data_){
        series.points.clear();
        series.line->clear();
    }
    syncSeriesVisibility();
    x_->setRange(0,window_s_);
    dirty_=true;
}
void ScopePlot::clearSeries(int index){
    if(index<0 || index>=static_cast<int>(data_.size())) return;
    auto& series=data_[static_cast<std::size_t>(index)];
    series.points.clear();
    series.line->clear();
    hideCursor();
    syncSeriesVisibility();
    dirty_=true;
}
void ScopePlot::setWindowSeconds(double seconds){
    window_s_=std::clamp(seconds,1.0,60.0);
    labelTime();
    dirty_=true;
}
void ScopePlot::labelTime(){
    time_label_->setText(QString::number(window_s_,'f',window_s_<10?1:0)+" s");
}
void ScopePlot::setYRange(double lo,double hi){
    if(!std::isfinite(lo) || !std::isfinite(hi) || hi<=lo) return;
    y_lo_=lo;
    y_hi_=hi;
    y_->setRange(lo,hi);
    // A manual axis range disables auto-fitting for THIS plot only.
    setAutoFitY(false,minimum_y_span_);
}
void ScopePlot::setAutoFitY(bool enabled,double minimumSpan){
    if(std::isfinite(minimumSpan) && minimumSpan>0.0){
        minimum_y_span_=minimumSpan;
    }
    auto_y_=enabled;
    if(auto_fit_button_){
        // API calls (e.g. initial dashboard configuration) must update the
        // visible toggle without emitting another toggled() callback.
        const QSignalBlocker block(auto_fit_button_);
        auto_fit_button_->setChecked(enabled);
        auto_fit_button_->setText(enabled?"AUTO Y: ON":"AUTO Y: OFF");
        auto_fit_button_->setToolTip(enabled
            ? "AUTO FIT ON: vertical scale follows live data. Y−/Y+ switches to manual."
            : "AUTO FIT OFF: vertical scale is fixed. FIT adjusts once; AUTO Y turns tracking on.");
    }
    dirty_=true;
}
void ScopePlot::fitY(){
    // Only online series with visible samples are allowed to affect the range.
    const double left=x_->min();
    const double right=x_->max();
    double low=std::numeric_limits<double>::infinity();
    double high=-std::numeric_limits<double>::infinity();
    for(const auto& series:data_){
        if(!series.line->isVisible()) continue;
        for(const auto& point:series.points){
            if(point.x()<left || point.x()>right) continue;
            low=std::min(low,point.y());
            high=std::max(high,point.y());
        }
    }
    if(!std::isfinite(low) || !std::isfinite(high)) return;
    const double span=std::max(high-low,minimum_y_span_);
    const double center=low+0.5*(high-low);
    const double pad=span*0.12;
    const double new_lo=center-0.5*span-pad;
    const double new_hi=center+0.5*span+pad;
    // Avoid constant axis recreation when the values change only in rounding.
    const double epsilon=std::max(1.e-8,span*0.005);
    if(std::abs(new_lo-y_lo_)>epsilon || std::abs(new_hi-y_hi_)>epsilon){
        y_lo_=new_lo;
        y_hi_=new_hi;
        y_->setRange(new_lo,new_hi);
    }
}
void ScopePlot::setDashed(int index,bool enabled){
    if(index>=0 && index<static_cast<int>(data_.size())){
        data_[static_cast<std::size_t>(index)].dashed=enabled;
        reTheme(fg_,bg_,grid_,colors_);
    }
}
void ScopePlot::setActive(bool enabled){
    active_=enabled;
    if(enabled){
        dirty_=true;
        refresh_timer_->start();
    }else{
        refresh_timer_->stop();
        hideCursor();
    }
}
void ScopePlot::zoomTime(double factor){
    setWindowSeconds(window_s_*factor);
}
void ScopePlot::zoomY(double factor){
    const double mid=(y_lo_+y_hi_)*0.5;
    const double half=(y_hi_-y_lo_)*factor*0.5;
    setYRange(mid-half,mid+half); // Manual zoom disables automatic Y fit.
}
void ScopePlot::syncSeriesVisibility(){
    int visible_count=0;
    for(auto& series:data_){
        const bool has_samples=!series.points.empty();
        if(series.rendered!=has_samples){
            series.rendered=has_samples;
            series.line->setVisible(has_samples);
            // Qt Charts retains legend markers for invisible series.
            // Toggle markers only on connectivity / data-state transitions.
            for(auto* marker:chart_->legend()->markers(series.line)){
                marker->setVisible(has_samples);
            }
        }
        if(has_samples) ++visible_count;
    }
    if(chart_->legend()->isVisible()!=(visible_count>0)){
        chart_->legend()->setVisible(visible_count>0);
    }
}
void ScopePlot::refresh(){
    if(!dirty_ || !active_)return;
    dirty_=false;
    syncSeriesVisibility();
    double latest=0.0;
    bool has_samples=false;
    for(const auto& series:data_){
        if(!series.line->isVisible()) continue;
        if(!series.points.empty()){
            has_samples=true;
            latest=std::max(latest,series.points.back().x());
        }
    }
    if(!has_samples){
        x_->setRange(0.0,window_s_);
        hideCursor();
        return;
    }
    const double left=std::max(0.0,latest-window_s_);
    const double right=std::max(window_s_,latest);
    x_->setRange(left,right);
    for(auto& series:data_){
        if(!series.line->isVisible())continue;
        QList<QPointF> points;
        points.reserve(static_cast<int>(series.points.size()));
        for(const auto& point:series.points){
            if(point.x()>=x_->min()-1.0)points.push_back(point);
        }
        series.line->replace(points);
        // A single topic publication should still be visibly marked.
        const bool single_point=points.size()==1;
        if(series.line->pointsVisible()!=single_point){
            series.line->setPointsVisible(single_point);
        }
    }
    if(auto_y_)fitY();
    if(hovering_)updateCursor(hover_position_);
}
void ScopePlot::reTheme(const QColor& fg,const QColor& bg,const QColor& grid,const QList<QColor>& colors){
 fg_=fg;bg_=bg;grid_=grid;colors_=colors;chart_->setBackgroundBrush(bg);chart_->setPlotAreaBackgroundBrush(bg);chart_->setPlotAreaBackgroundVisible(true);
 chart_->legend()->setLabelColor(fg);chart_->setBackgroundPen(QPen(Qt::NoPen));
 for(auto* axis:{x_,y_}){axis->setLabelsColor(fg);axis->setTitleBrush(fg);axis->setGridLineColor(grid);axis->setLinePenColor(grid);}
 for(size_t i=0;i<data_.size();++i){QPen p(colors[int(i)%colors.size()],1.8);if(data_[i].dashed)p.setStyle(Qt::DashLine);p.setCosmetic(true);data_[i].line->setPen(p);}
 if(cursor_line_)cursor_line_->setPen(QPen(fg_.darker(145),1,Qt::DashLine));
 if (hover_label_) {
     hover_label_->setStyleSheet(QString(
         "QLabel#scopeHoverLabel { background-color: %1; color: %2; "
         "border: 1px solid %3; border-radius: 5px; padding: 2px; }")
         .arg(bg_.lighter(145).name(),fg_.name(),grid_.lighter(170).name()));
 }
}
void ScopePlot::hideCursor() {
    hovering_ = false;
    if (cursor_line_) {
        cursor_line_->setLine(QLineF());
        cursor_line_->hide();
    }
    for (auto* marker : cursor_points_) marker->hide();
    if (hover_label_) hover_label_->hide();
    if (auto* scene = chart_ ? chart_->scene() : nullptr) {
        scene->update();
    }
    if (view_ && view_->viewport()) {
        view_->viewport()->update();
        view_->viewport()->repaint();
    }
}

void ScopePlot::updateCursor(const QPoint& pos) {
    if (!active_ || data_.empty()) { hideCursor(); return; }
    const QPointF scene_pos = view_->mapToScene(pos);
    const QPointF chart_pos = chart_->mapFromScene(scene_pos);
    const QRectF plot_area = chart_->plotArea();
    if (!plot_area.contains(chart_pos)) { hideCursor(); return; }
    const QPointF value_pos = chart_->mapToValue(chart_pos, data_.front().line);
    const double t = value_pos.x();
    if (!std::isfinite(t)) { hideCursor(); return; }

    QString tooltip = QString("<b>t = %1 s</b>").arg(t, 0, 'f', 3);
    bool has_values = false;
    for (std::size_t i = 0; i < data_.size(); ++i) {
        auto* marker = cursor_points_[i];
        const auto& points = data_[i].points;
        if(!data_[i].line->isVisible()) { marker->hide(); continue; }
        if (points.empty() || t < points.front().x() || t > points.back().x()) {
            marker->hide();
            continue;
        }
        const auto upper = std::lower_bound(points.begin(), points.end(), t,
            [](const QPointF& item, double x) { return item.x() < x; });
        double y = 0.0;
        if (upper == points.end()) {
            y = points.back().y();
        } else if (upper == points.begin()) {
            y = upper->y();
        } else {
            const auto lower = std::prev(upper);
            const double dt = upper->x() - lower->x();
            const double alpha = dt > 1e-12 ? (t - lower->x()) / dt : 0.0;
            y = lower->y() + std::clamp(alpha, 0.0, 1.0) * (upper->y() - lower->y());
        }
        if (!std::isfinite(y)) { marker->hide(); continue; }
        const QPointF marker_scene = chart_->mapToScene(
            chart_->mapToPosition(QPointF(t, y), data_[i].line));
        marker->setRect(marker_scene.x() - 4, marker_scene.y() - 4, 8, 8);
        const QColor color = colors_.isEmpty() ? QColor("#69b5f0")
            : colors_[int(i) % colors_.size()];
        marker->setBrush(QBrush(color));
        marker->setPen(QPen(bg_, 1.2));
        marker->setVisible(plot_area.contains(chart_->mapFromScene(marker_scene)));
        tooltip += QString("<br><span style=\"color:%1\">●</span> %2: <b>%3</b>")
            .arg(color.name(), data_[i].line->name())
            .arg(y, 0, 'f', 3);
        has_values = true;
    }
    if (!has_values) {
        tooltip += "<br><i>No sample at this timestamp</i>";
        for (auto* marker : cursor_points_) marker->hide();
    }
    const QPointF top = chart_->mapToScene(QPointF(chart_pos.x(), plot_area.top()));
    const QPointF bottom = chart_->mapToScene(QPointF(chart_pos.x(), plot_area.bottom()));
    cursor_line_->setLine(QLineF(top, bottom));
    cursor_line_->show();
    hovering_ = true;
    hover_position_ = pos;
    if (hover_label_) {
        hover_label_->setText(tooltip);
        hover_label_->adjustSize();
        positionHoverLabel(pos);
        hover_label_->show();
        hover_label_->raise();
    }
    if (view_ && view_->viewport()) view_->viewport()->update();
}

void ScopePlot::positionHoverLabel(const QPoint& position) {
    if (!hover_label_) return;
    // Coordinates refer to QChartView's viewport, never the window or chart
    // scene. Keep the popup adjacent to the cursor and flip at panel edges.
    const QSize box = hover_label_->size();
    const int view_width = view_->viewport()->width();
    const int view_height = view_->viewport()->height();
    constexpr int offset = 14;
    constexpr int margin = 3;
    int left = position.x() + offset;
    int top = position.y() + offset;
    if (left + box.width() + margin > view_width) {
        left = position.x() - box.width() - offset;
    }
    if (top + box.height() + margin > view_height) {
        top = position.y() - box.height() - offset;
    }
    left = std::clamp(left, margin, std::max(margin, view_width - box.width() - margin));
    top = std::clamp(top, margin, std::max(margin, view_height - box.height() - margin));
    hover_label_->move(left, top);
}

bool ScopePlot::eventFilter(QObject* object, QEvent* event) {
    if (object != view_->viewport()) return QFrame::eventFilter(object, event);
    if (event->type() == QEvent::Wheel) {
        const auto* wheel = static_cast<QWheelEvent*>(event);
        if (wheel->angleDelta().y() != 0) {
            zoomTime(wheel->angleDelta().y() > 0 ? 0.75 : 4.0 / 3.0);
        }
        return true;
    }
    if (event->type() == QEvent::MouseMove) {
        const auto* move = static_cast<QMouseEvent*>(event);
        hover_position_ = move->pos();
        // Reposition the overlay on EVERY mouse event, not on the chart's
        // 50 ms refresh timer. Numerical interpolation is limited to ~30 Hz.
        if (hover_label_ && hover_label_->isVisible()) {
            positionHoverLabel(hover_position_);
        }
        if (!hovering_ || !hover_timer_.isValid() || hover_timer_.elapsed() >= 16) {
            hover_timer_.restart();
            updateCursor(hover_position_);
        }
    } else if (event->type() == QEvent::Leave) {
        hideCursor();
    } else if (event->type() == QEvent::Resize && hovering_) {
        updateCursor(hover_position_);
    }
    return QFrame::eventFilter(object, event);
}
}  // namespace cefiro::gui
