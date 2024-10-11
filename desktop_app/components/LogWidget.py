from PyQt5.QtWidgets    import QWidget, QVBoxLayout, QTextEdit
from PyQt5.QtCore       import Qt
from PyQt5.QtGui        import QFont
from components         import TitleLabel

class LogWidget(QWidget):
    def __init__(self):
        super().__init__()


        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 5px;')

        # creating and adding titleLabel
        self.title = TitleLabel("Logs\n------------------------")
        self.title.setMaximumHeight(30)
        self.setMinimumWidth(300)
        mainLayout.addWidget(self.title)

        # creating and adding editText
        self.textBox = QTextEdit(self)
        self.textBox.setReadOnly(True)
        self.textBox.setStyleSheet(f"color: #ADB2BD;")
        self.textBox.setFont(QFont("Monospace", 12))
        mainLayout.addWidget(self.textBox)

        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")
        self.appendMessage("test log showing a lot of data regarding nothing since I just need to test how this works. Checking 0 real quick")


    def appendMessage(self,msg):
        self.textBox.append('<font color="#80ABE4">[-] </font>' + msg + '<br>')
        self.textBox.verticalScrollBar().setValue(self.textBox.verticalScrollBar().maximum())