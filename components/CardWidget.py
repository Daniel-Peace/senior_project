
from components         import TitleLabel, BodyLabel
from PyQt5.QtCore       import Qt
from PyQt5.QtWidgets    import QWidget, QVBoxLayout

# this class is a custom card widget that contains a title and a body of text
class CardWidget(QWidget):
    # constructor
    def __init__(self, title, body, parent = None):
        # instantiate parent object
        super().__init__(parent)

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting default properties
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 15px;')
        
        # creating and adding title label
        self.titleLabel = TitleLabel(title)
        self.titleLabel.setMaximumHeight(60)
        self.titleLabel.setMinimumHeight(60)
        mainLayout.addWidget(self.titleLabel)        

        # creating sub-widget to hold body label
        self.bodyWidget = QWidget()
        self.bodyWidget.setStyleSheet(f"color: #ADB2BD;background-color: #42474f;padding: 10px;border-radius: 15px;")
        
        # creating setting main layout of sub-widget
        self.bodyLayout = QVBoxLayout()
        self.bodyWidget.setLayout(self.bodyLayout)

        # creating and adding body label to sub-widget
        self.bodyLabel = BodyLabel(body)
        self.bodyLayout.addWidget(self.bodyLabel)

        # adding sub-widget to main widget
        mainLayout.addWidget(self.bodyWidget)

    # updates the title label
    def updateTitleText(self, title):
        self.titleLabel.updateText(title)

    # updates the body label
    def updateBodyText(self, title):
        self.bodyLabel.updateText(title)

    # updates the background color of the card
    def updateBodyBackgroundColor(self, color):
        self.bodyWidget.setAttribute(Qt.WA_StyledBackground, True)
        self.bodyWidget.setStyleSheet(f"background-color: {color};"'border-radius: 15px;')
    
    # updates the color of all text in the card
    def updateBodyTextColor(self, color):
        self.bodyLabel.updateColor(color)