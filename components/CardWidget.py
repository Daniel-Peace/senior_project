from PyQt5.QtWidgets    import QWidget, QVBoxLayout
from PyQt5.QtCore       import Qt
from components         import TitleLabel, BodyLabel

class CardWidget(QWidget):
    def __init__(self, title, body, parent = None):
        super().__init__(parent)

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 15px;')
        
        # creating and adding title label
        self.titleLabel = TitleLabel(title)
        mainLayout.addWidget(self.titleLabel)
        self.titleLabel.setMaximumHeight(60)
        self.titleLabel.setMinimumHeight(60)

        # creating and adding title label
        self.bodyLabel = BodyLabel(body)
        mainLayout.addWidget(self.bodyLabel)

    # updates the title label
    def updateTitle(self, title):
        self.titleLabel.updateText(title)

    # updates the body label
    def updateBody(self, title):
        self.bodyLabel.updateText(title)

    # updates the background color of the card
    def updateBackgroundColor(self, color):
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {color};"'border-radius: 15px;')
    
    # updates the color of all text in the card
    def updateTextColor(self, color):
        self.titleLabel.updateColor(color)
        self.bodyLabel.updateColor(color)