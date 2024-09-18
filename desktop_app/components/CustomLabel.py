from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt

class CustomLabel(QLabel):
    def __init__(self, text="", parent=None):
        super().__init__(text, parent)

        # Set default properties
        self.setAlignment(Qt.AlignCenter)  # Center the text
        self.setStyleSheet("color: white;")  # Default text color
        self.setFont(QFont("Arial", 14))  # Default font and size

    def set_custom_style(self, color="white", font_family="Arial", font_size=14):
        """Custom method to change the label's appearance."""
        self.setStyleSheet(f"color: {color};")
        self.setFont(QFont(font_family, font_size))

    def update_text(self, new_text):
        """Custom method to update the text."""
        self.setText(new_text)