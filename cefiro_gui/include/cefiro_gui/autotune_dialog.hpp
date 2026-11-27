#pragma once
#include "cefiro_gui/autotune.hpp"
class QWidget;
namespace cefiro::gui {
void showAutoTuneDialog(QWidget* parent);
std::optional<autotune::Candidate> loadShadowCandidate(QWidget* parent);
}
