# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class creates custom label to be used for titles of sections and widgets
#
# -------------------------------------------------------------------------------------------

from PyQt5.QtCore       import Qt
from PyQt5.QtGui        import QFont
from PyQt5.QtWidgets    import QLabel

# This is a customized version of the QLabel for use as a title
class TitleLabel(QLabel):
    # constructor
    def __init__(self, text="", parent=None):
        # instantiate parent object
        super().__init__(text, parent)

        # setting default properties
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 20, QFont.Bold))

    # updates the text of the label
    def updateText(self, text):
        self.setText(text)

    # updates the color of the text in the label
    def updateTextColor(self, color):
        self.setStyleSheet(f"color: {color};")