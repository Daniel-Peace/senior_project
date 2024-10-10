from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class TitleLabel(QLabel):
    def __init__(self, text="", parent=None):
        # instantiate parent object
        super().__init__(text, parent)

        # setting default properties
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 14, QFont.Bold))

    def updateText(self, text):
        self.setText(text)

    def updateColor(self, color):
        self.setStyleSheet(f"color: {color};")