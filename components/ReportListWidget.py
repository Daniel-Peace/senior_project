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
        self.setStyleSheet('background-color: #2F343E;''border-radius: 15px;')

        # creating and adding titleLabel
        self.title = TitleLabel("Reports")
        self.title.setMaximumHeight(60)
        self.title.setMinimumHeight(60)
        # self.setMinimumWidth(300)
        mainLayout.addWidget(self.title)

        # creating and adding editText
        self.list = ReportList(self)
        self.list.setStyleSheet(f"color: #ADB2BD;background-color: #42474f;padding: 30px;border-radius: 15px;")
        mainLayout.addWidget(self.list)