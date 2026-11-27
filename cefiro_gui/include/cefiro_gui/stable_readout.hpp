#pragma once
#include <QLabel>
#include <QFontMetrics>
#include <QSizePolicy>
#include <QStringList>
#include <algorithm>

namespace cefiro::gui {
// Readout geometry must be a function of typography, never of live values.
// Fixed-height + no wrapping prevents valueChanged/timer layout oscillations.
inline void reserveReadout(QLabel* readout, int rows, bool fixed_width=false,
                    const QStringList& templates={}) {
    if(!readout)return;
    readout->ensurePolished();
    readout->setTextFormat(Qt::PlainText);
    readout->setWordWrap(false);
    readout->setIndent(0);
    readout->setMargin(0);
    readout->setSizePolicy(fixed_width?QSizePolicy::Fixed:QSizePolicy::Ignored,
                           QSizePolicy::Fixed);
    const QFontMetrics fm(readout->font());
    const auto margins=readout->contentsMargins();
    const int height=fm.height()+(rows-1)*fm.lineSpacing()+margins.top()+margins.bottom()+4;
    readout->setFixedHeight(height);
    if(fixed_width){
        int width=0;
        for(const auto& text:templates)width=std::max(width,fm.horizontalAdvance(text));
        readout->setFixedWidth(width+margins.left()+margins.right()+12);
    }
}
inline void setReadoutText(QLabel* readout,const QString& text,const QString& detail=QString()){
    if(!readout)return;
    // Avoid redundant text updates and the resulting layout invalidations.
    if(readout->text()!=text)readout->setText(text);
    const QString tooltip=detail.isEmpty()?text:text+"\n"+detail;
    if(readout->toolTip()!=tooltip)readout->setToolTip(tooltip);
}
} // namespace cefiro::gui
