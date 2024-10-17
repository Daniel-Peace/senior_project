from PyQt5.QtWidgets    import QWidget, QVBoxLayout
from PyQt5.QtCore       import Qt
from components         import TitleLabel, ReportList

# this class creates a list widget that has a title and a list object
class ReportListWidget(QWidget):
    # constructor
    def __init__(self):
        super().__init__()

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 5px;')

        # creating and adding titleLabel
        self.title = TitleLabel("Reports")
        self.title.setMaximumHeight(40)
        self.setMinimumWidth(300)
        mainLayout.addWidget(self.title)

        # creating and adding editText
        self.list = ReportList(self)
        mainLayout.addWidget(self.list)