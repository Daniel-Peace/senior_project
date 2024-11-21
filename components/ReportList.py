# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class provides a custom list for holder report names and attached data
#
# -------------------------------------------------------------------------------------------

from casualty           import Casualty
from PyQt5.QtCore       import Qt
from PyQt5.QtGui        import QFont, QColor
from PyQt5.QtWidgets    import QListWidget, QListWidgetItem

# this class creates a list of report names with casualty objects attached as data
class ReportList(QListWidget):
    # constructor
    def __init__(self, parent = None):
        # instantiate parent object
        super().__init__(parent)
        
        # setting default properties
        self.setStyleSheet(f"color: #ADB2BD;")
        self.setFont(QFont("Arial", 20))

    # this function adds the name of the report to the list and attaches the report data to the item
    def addItemToList(self, name:str, casualty:Casualty, color:str):
        item = QListWidgetItem(name)
        item.setData(Qt.UserRole, casualty)
        item.setForeground(QColor(color))
        self.addItem(item)
        item.setSelected(True)

    # this function clears the current selection
    def clearItemSelection(self):
        self.clearSelection