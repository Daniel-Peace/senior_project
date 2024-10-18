from PyQt5.QtWidgets    import QGraphicsView, QGraphicsScene, QVBoxLayout, QWidget, QGraphicsPixmapItem
from PyQt5.QtGui        import QPixmap, QImage
from PyQt5.QtCore       import Qt
from screeninfo         import get_monitors

class VideoView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)

        # Remove borders and scrollbars
        self.view.setFrameShape(QGraphicsView.NoFrame)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Initialize placeholder items
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)

    def update_image(self, numpy_array):        
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
        pixmap = QPixmap.fromImage(qimage)
        # pixmap = pixmap.scaled(monitors[0].width/2, monitors[0].width/2, Qt.KeepAspectRatio)
        self.pixmap_item.setPixmap(pixmap)
        
        width = pixmap.width()
        height = pixmap.height()

        # Set the scene rect to the size of the image
        self.scene.setSceneRect(0, 0, width, height)
        
        # Resize the QGraphicsView to fit the image size
        self.view.setFixedSize(width, height)
        
        # Resize the container widget to fit the new image size
        self.setFixedSize(self.view.size())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        
