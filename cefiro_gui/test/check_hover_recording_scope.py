"""Regression for the V15 recording prefix scope error (static check)."""
from pathlib import Path
s = (Path(__file__).resolve().parents[1] / "src/main_window.cpp").read_text()
legacy = s.split("bool shadowRequested=false;", 1)[1].split("connect(close,", 1)[0]
assert 'config.prefix="cefiro_swing_lqr_experiment";' in legacy
assert "hover?" not in legacy and "hover ?" not in legacy
local = s.split("void MainWindow::startEmbeddedRoll(){", 1)[1].split("void MainWindow::serviceEmbeddedRoll(){", 1)[0]
assert local.index("const bool hover=") < local.index('config.prefix=hover?')
assert 'config.prefix=hover?"cefiro_hover_lqr_experiment":"cefiro_swing_lqr_experiment";' in local
print("Recording prefix scopes: PASS")
