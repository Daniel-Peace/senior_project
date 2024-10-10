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
        self.setStyleSheet('background-color: #2F343E;''border-radius: 5px;')

        # creating and adding title label
        self.titleLabel = TitleLabel(title)
        # self.titleLabel.setStyleSheet("background-color: transparent;")
        mainLayout.addWidget(self.titleLabel)

        # creating and adding title label
        self.bodyLabel = BodyLabel(body)
        # self.bodyLabel.setStyleSheet("background-color: transparent;")
        mainLayout.addWidget(self.bodyLabel)

    # updates the title label
    def updateTitle(self, title):
        self.titleLabel.updateText(title)

    # updates the body label
    def updateBody(self, title):
        self.bodyLabel.updateText(title)