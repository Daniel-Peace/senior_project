from PyQt5.QtWidgets    import QGraphicsView
from PyQt5.QtWidgets    import QGraphicsScene
from PyQt5.QtWidgets    import QVBoxLayout
from PyQt5.QtWidgets    import QWidget
from PyQt5.QtWidgets    import QGraphicsPixmapItem
from PyQt5.QtGui        import QPixmap, QImage
from PyQt5.QtCore       import Qt


class VideoDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        
        # Layout
        layout = QVBoxLayout(self)
        layout.addWidget(self.view)
        self.setLayout(layout)

        # Initialize placeholder items
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)
        self.scene.setSceneRect(0, 0, 700, 500)  # Default scene size

    def update_image(self, numpy_array):
        """Update the displayed image with a new NumPy array."""
        self.image_width = numpy_array.shape[1]
        self.image_height = numpy_array.shape[0]
        
        height, width, channels = numpy_array.shape
        bytes_per_line = channels * width

        format_map = {
            1: QImage.Format_Grayscale8,
            3: QImage.Format_RGB888,
            4: QImage.Format_RGBA8888
        }
        
        if channels not in format_map:
            raise ValueError(f"Unsupported number of channels: {channels}")

        qimage = QImage(numpy_array.data, width, height, bytes_per_line, format_map[channels])

        # Update QGraphicsView
        self.pixmap_item.setPixmap(QPixmap.fromImage(qimage))
        self.scene.setSceneRect(0, 0, width, height)
        self.view.fitInView(self.pixmap_item, Qt.IgnoreAspectRatio)