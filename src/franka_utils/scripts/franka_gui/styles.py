"""QSS styles for the Franka Panda control GUI."""

DARK_STYLE = """
QMainWindow, QWidget {
    background: #0b1220;
    color: #e5edf8;
    font-family: 'Inter', 'DejaVu Sans';
    font-size: 10pt;
}
QFrame#Header {
    background: #101b2d;
    border: 1px solid #243650;
    border-radius: 12px;
}
QLabel#Title {
    font-size: 19pt;
    font-weight: 700;
    color: #f4f8ff;
}
QLabel#Subtitle {
    color: #91a7c6;
    font-size: 9pt;
}
QGroupBox {
    border: 1px solid #263a56;
    border-radius: 12px;
    margin-top: 12px;
    padding: 14px 10px 10px 10px;
    font-weight: 600;
    color: #c9dcf7;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 5px;
}
QPushButton {
    background: #17263b;
    border: 1px solid #304865;
    border-radius: 8px;
    padding: 9px 12px;
    color: #e5edf8;
}
QPushButton:hover {
    background: #203a59;
    border-color: #398bff;
}
QPushButton:pressed {
    background: #0f6bce;
}
QPushButton#Primary {
    background: #1677f9;
    border: 1px solid #328cff;
    font-weight: 700;
}
QPushButton#Primary:hover {
    background: #328cff;
}
QPushButton#Danger {
    background: #3a1c2a;
    border: 1px solid #7f2c45;
    color: #ffb8c6;
}
QPushButton#Success {
    background: #12382f;
    border: 1px solid #1e785e;
    color: #8af0c4;
}
QSlider::groove:horizontal {
    background: #24364e;
    height: 6px;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #2187ff;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #8bcaff;
    width: 15px;
    margin: -5px 0;
    border-radius: 7px;
    border: 2px solid #1677f9;
}
QDoubleSpinBox, QComboBox, QLineEdit {
    background: #101d30;
    border: 1px solid #30445f;
    border-radius: 6px;
    padding: 6px;
    selection-background-color: #1677f9;
}
QComboBox QAbstractItemView {
    background: #101d30;
    selection-background-color: #1677f9;
}
QTabWidget::pane {
    border: 1px solid #263a56;
    border-radius: 10px;
    top: -1px;
}
QTabBar::tab {
    background: #101b2d;
    border: 1px solid #263a56;
    padding: 10px 16px;
    margin-right: 4px;
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
}
QTabBar::tab:selected {
    background: #17365b;
    color: #ffffff;
    border-bottom-color: #2187ff;
}
QPlainTextEdit {
    background: #080e18;
    color: #b8cbe4;
    border: 1px solid #263a56;
    border-radius: 8px;
    font-family: monospace;
}
QProgressBar {
    background: #142239;
    border: 0;
    border-radius: 4px;
    height: 8px;
}
QProgressBar::chunk {
    background: #2187ff;
    border-radius: 4px;
}
"""