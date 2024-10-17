from PyQt5.QtWidgets    import QListWidget, QListWidgetItem
from PyQt5.QtGui        import QFont
from PyQt5.QtCore       import Qt
from casualty           import Casualty

# this class creates a list of report names with casualty objects attached as data
class ReportList(QListWidget):
    def __init__(self, parent = None):
        super().__init__(parent)
        
        # setting default properties
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 14))

    # adds a report to the list by adding the report name to the list and attaching the casualty data
    def addItemToList(self, name:str, casualty:Casualty):
        item = QListWidgetItem(name)
        item.setData(Qt.UserRole, casualty)
        self.addItem(item)