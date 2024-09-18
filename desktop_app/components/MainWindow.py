from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton

from components import CustomLabel
from components import VideoDisplay
from components import CardWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('NumPy Array to PyQt5 Example')
        self.setGeometry(100, 100, 800, 600)

        # Create a central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create the main vertical layout
        vbox_layout = QVBoxLayout(central_widget)

        # Create and add the GraphicsDisplay widget
        self.graphics_display = VideoDisplay(self)
        vbox_layout.addWidget(self.graphics_display)

        # Create a horizontal layout
        hbox_layout = QHBoxLayout()



        # Add widgets to the horizontal layout
        self.card1 = CardWidget("AprilTag ID", "NA")
        hbox_layout.addWidget(self.card1)

        # Add the horizontal layout to the vertical layout
        vbox_layout.addLayout(hbox_layout)

        self.setStyleSheet("""
            QMainWindow {
                background-color: #282C32;  /* Light gray background */
            }
        """)
