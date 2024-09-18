from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import Qt

from components import CustomLabel

class CardWidget(QWidget):
    def __init__(self, title, description, parent=None):
        super().__init__(parent)

        # Create the main layout
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(main_layout)

        # Create a container widget for card content
        card_container = QWidget()
        card_layout = QVBoxLayout()
        card_layout.setContentsMargins(10, 10, 10, 10)  # Margins inside the card container
        card_container.setLayout(card_layout)
        
        # Title
        self.title_label = CustomLabel(title)
        self.title_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self.title_label)

        # Description
        self.description_label = CustomLabel(description)
        self.description_label.setWordWrap(True)
        self.description_label.setAlignment(Qt.AlignCenter)
        card_layout.addWidget(self.description_label)

        # Add the card container to the main layout
        main_layout.addWidget(card_container)

        # Styling the card container
        card_container.setStyleSheet("""
            QWidget {
                background-color: #2F343E;                    
                border-radius: 5px;
                padding: 10px;
            }
        """)

        # Set fixed size for the card (optional)
        self.setFixedSize(150, 150)
