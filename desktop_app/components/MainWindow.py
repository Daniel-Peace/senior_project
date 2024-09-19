from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

from components import CustomLabel
from components import VideoDisplay
from components import CardWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS Monitor')
        self.setGeometry(100, 100, 800, 600)

        # Create a central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)


        # Create the main vertical layout
        vbox_layout_left = QVBoxLayout()

        # Create and add the GraphicsDisplay widget
        self.graphics_display = VideoDisplay(self)
        vbox_layout_left.addWidget(self.graphics_display)

        # Create a horizontal layout
        hbox_layout = QHBoxLayout()

        # Add widgets to the horizontal layout
        self.card1 = CardWidget("AprilTag ID", "NA")
        hbox_layout.addWidget(self.card1)
        self.card2 = CardWidget("AprilTag ID", "NA")
        hbox_layout.addWidget(self.card2)
        self.card3 = CardWidget("AprilTag ID", "NA")
        hbox_layout.addWidget(self.card3)
        self.card4 = CardWidget("AprilTag ID", "NA")
        hbox_layout.addWidget(self.card4)

        # Add the horizontal layout to the vertical layout
        vbox_layout_left.addLayout(hbox_layout)

        # Adding vbox layout to main hbox
        main_layout.addLayout(vbox_layout_left)



        # Create the main vertical layout
        vbox_layout_right = QVBoxLayout()

        main_card_layout = QVBoxLayout()
        main_card_layout.setContentsMargins(0,0,0,0)

        card_container = QWidget()
        inner_card_layout = QVBoxLayout()
        inner_card_layout.setContentsMargins(0,0,0,0)
        card_container.setLayout(inner_card_layout)

        title = QLabel()
        title.setText("Current Casualty Info")
        font = QFont("Roboto Mono", 12, QFont.Bold)
        title.setFont(font)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"color: white;")
        inner_card_layout.addWidget(title)

        row_1 = QHBoxLayout()
        label_1_title = QLabel()
        label_1_title.setText("Casualty ID:")
        font = QFont("Roboto Mono", 12)
        label_1_title.setFont(font)
        label_1_title.setAlignment(Qt.AlignLeft)
        label_1_title.setStyleSheet(f"color: white;")
        row_1.addWidget(label_1_title)

        label_1_description = QLabel()
        label_1_description.setText("NA")
        font = QFont("Roboto Mono", 12)
        label_1_description.setFont(font)
        label_1_description.setAlignment(Qt.AlignHCenter)
        label_1_description.setStyleSheet(f"color: white;")
        row_1.addWidget(label_1_description)
        inner_card_layout.addLayout(row_1)

        row_2 = QHBoxLayout()
        label_2_title = QLabel()
        label_2_title.setText("Is coherent:")
        font = QFont("Roboto Mono", 12)
        label_2_title.setFont(font)
        label_2_title.setAlignment(Qt.AlignLeft)
        label_2_title.setStyleSheet(f"color: white;")
        row_2.addWidget(label_2_title)

        label_2_description = QLabel()
        label_2_description.setText("NA")
        font = QFont("Roboto Mono", 12)
        label_2_description.setFont(font)
        label_2_description.setAlignment(Qt.AlignHCenter)
        label_2_description.setStyleSheet(f"color: white;")
        row_2.addWidget(label_2_description)
        inner_card_layout.addLayout(row_2)

        inner_card_layout.addStretch()

        main_card_layout.addWidget(card_container)

        vbox_layout_right.addLayout(main_card_layout)
        
        

        # Styling the card container
        card_container.setStyleSheet("""
            QWidget {
                background-color: #2F343E;                    
                border-radius: 5px;
                padding: 5px;
            }
        """)

        # # Create a horizontal layout
        # hbox_layout_right = QHBoxLayout()

        # # Add widgets to the horizontal layout
        # self.card2 = CardWidget("Test", "NA")
        # hbox_layout_right.addWidget(self.card2)

        # # Add the horizontal layout to the vertical layout
        # vbox_layout_right.addLayout(hbox_layout_right)

        # Adding vbox layout to main hbox
        main_layout.addLayout(vbox_layout_right)

        

        

        

        self.setStyleSheet("""
            QMainWindow {
                background-color: #282C32;  /* Light gray background */
            }
        """)
