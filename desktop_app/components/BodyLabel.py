from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class BodyLabel(QLabel):
    def __init__(self, text="", parent=None):
        # instantiate parent object
        super().__init__(text, parent)

        # setting default properties
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 14))

    def updateText(self, text):
        self.setText(text)