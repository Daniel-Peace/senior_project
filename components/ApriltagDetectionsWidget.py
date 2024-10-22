from PyQt5.QtWidgets    import QWidget, QVBoxLayout
from PyQt5.QtCore       import Qt
from components         import TitleLabel, BodyLabel
from PyQt5.QtGui        import QFont

# this class creates a list widget that has a title and a list object
class ApriltagDetectionsWidget(QWidget):
    # constructor
    def __init__(self):
        super().__init__()

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 15px;')

        # creating and adding titleLabel
        self.title = TitleLabel("Current AprilTag Detections")
        self.title.setMaximumHeight(60)
        self.title.setMinimumHeight(60)
        mainLayout.addWidget(self.title)

        self.valueWidget = QWidget()
        self.valueWidget.setStyleSheet(f"color: #ADB2BD;background-color: #42474f;padding: 10px;border-radius: 15px;")
        self.valueLayout = QVBoxLayout()
        self.valueWidget.setLayout(self.valueLayout)

       # creating and adding title label
        self.bodyLabel = BodyLabel("Tag ID: --")
        self.valueLayout.addWidget(self.bodyLabel)

        mainLayout.addWidget(self.valueWidget)

    # updates the body label
    def updateBody(self, title):
        self.bodyLabel.updateText(title)