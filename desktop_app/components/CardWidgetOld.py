from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt

from components import CustomLabel
from components import TitleLabel
from components import List

class CardWidgetOld(QWidget):
    def __init__(self, title, description, parent=None):
        super().__init__(parent)

        # creating main layout for card
        main_layout = QVBoxLayout()

        # setting layout defaults
        main_layout.setContentsMargins(0, 0, 0, 0)

        # setting main layout of
        self.setLayout(main_layout)

        # Create a container widget for card content
        card_container = QWidget()
        card_layout = QVBoxLayout()
        # card_layout.setContentsMargins(10, 10, 10, 10)  # Margins inside the card container
        card_container.setLayout(card_layout)
        
        # Title
        self.title_label = TitleLabel(title)
        card_layout.addWidget(self.title_label)

        # Description
        self.description_label = CustomLabel(description)
        self.description_label.setWordWrap(True)
        self.description_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self.description_label)

        # self.list = List()
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # self.list.add_item("item test")
        # card_layout.addWidget(self.list)

        # Add the card container to the main layout
        main_layout.addWidget(card_container)
        

        # # Styling the card container
        # card_container.setStyleSheet("""
        #     QWidget {
        #         background-color: #2F343E;                    
        #         border-radius: 5px;
        #         padding: 10px;
        #     }
        # """)

        # Styling the card container
        card_container.setStyleSheet("""
            QWidget {
                background-color: #2F343E;                    
                border-radius: 5px;
            }
        """)

        # Set fixed size for the card (optional)
        # self.setFixedSize(150, 150)
