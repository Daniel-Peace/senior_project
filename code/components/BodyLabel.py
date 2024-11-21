# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class creates a label with custom styling for the body text of a widget or element
#
# -------------------------------------------------------------------------------------------

from PyQt5.QtCore       import Qt
from PyQt5.QtGui        import QFont
from PyQt5.QtWidgets    import QLabel

class BodyLabel(QLabel):
    # constructor
    def __init__(self, text="", parent=None):
        # instantiate parent object
        super().__init__(text, parent)

        # setting default properties
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 20))

    # updates the text of the label
    def updateText(self, text):
        self.setText(text)

    # updates the color of the text in the label
    def updateTextColor(self, color):
        self.setStyleSheet(f"color: {color};")