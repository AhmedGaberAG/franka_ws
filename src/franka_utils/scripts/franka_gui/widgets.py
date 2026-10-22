"""Custom Qt widgets for the Franka Panda control GUI."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QSlider

from .constants import ARM_JOINTS


class JointRow(QWidget):
    """A single row with joint name, slider, value display, and limits."""

    valueChanged = Signal(int, float)

    def __init__(self, index, name, low, high, parent=None):
        super().__init__(parent)
        self.index, self.name, self.low, self.high = index, name, low, high
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setHorizontalSpacing(10)

        self.name_label = QLabel(name.replace("panda_", ""))
        self.name_label.setMinimumWidth(100)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(round(low * 1000), round(high * 1000))
        self.slider.setValue(round(max(low, min(high, 0.0)) * 1000))

        self.value_label = QLabel("0.000 rad")
        self.value_label.setMinimumWidth(95)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.limit_label = QLabel(f"{low:.2f}  …  {high:.2f}")
        self.limit_label.setStyleSheet("color:#748ba9; font-size:8pt;")
        self.limit_label.setMinimumWidth(112)

        layout.addWidget(self.name_label, 0, 0)
        layout.addWidget(self.slider, 0, 1)
        layout.addWidget(self.value_label, 0, 2)
        layout.addWidget(self.limit_label, 0, 3)
        layout.setColumnStretch(1, 1)

        self.slider.valueChanged.connect(self._update_value)

    def _update_value(self, value):
        self.value_label.setText(f"{value / 1000.0:+.3f} rad")
        self.valueChanged.emit(self.index, value / 1000.0)

    def set_position(self, value):
        value = max(self.low, min(self.high, float(value)))
        self.slider.setValue(round(value * 1000))

    def value(self):
        return self.slider.value() / 1000.0