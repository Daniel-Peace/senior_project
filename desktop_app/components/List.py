import sys
from PyQt5.QtWidgets    import QListWidget
from PyQt5.QtWidgets    import QListWidgetItem
from PyQt5.QtGui        import QFont
from PyQt5.QtCore       import Qt

class List(QListWidget):
    def __init__(self, parent = None):
        super().__init__(parent)
        
        # setting default properties
        #self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 14, QFont.Bold))

    def add_item(self, string):
        self.addItem(QListWidgetItem(string))