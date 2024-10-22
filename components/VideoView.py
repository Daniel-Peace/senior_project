from PyQt5.QtWidgets    import QGraphicsView, QGraphicsScene, QVBoxLayout, QWidget, QGraphicsPixmapItem
from PyQt5.QtGui        import QPixmap, QImage
from PyQt5.QtCore       import Qt

class VideoView(QWidget):
    # constructor
    def __init__(self, parent=None):
        # instantiate parent object
        super().__init__(parent)

        # creating a graphics view and scene
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)

        # removing borders and scrollbars
        self.view.setFrameShape(QGraphicsView.NoFrame)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # initializing placeholder items
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)

    # this function updates the iamge with each new frame received from the ROS usb camera node
    def update_image(self, numpy_array): 
        # getting data about image from numpy array       
        height, width, channels = numpy_array.shape
        bytes_per_line = channels * width

        format_map = {
            1: QImage.Format_Grayscale8,
            3: QImage.Format_RGB888,
            4: QImage.Format_RGBA8888
        }
        
        # checking if formatting is supported
        if channels not in format_map:
            raise ValueError(f"Unsupported number of channels: {channels}")

        # creating a QImage from numpy array
        qimage = QImage(numpy_array.data, width, height, bytes_per_line, format_map[channels])

        # creating a QPixmap from the QImage and updating image
        pixmap = QPixmap.fromImage(qimage)
        self.pixmap_item.setPixmap(pixmap)
        width = pixmap.width()
        height = pixmap.height()

        # setting view size based on image size
        self.scene.setSceneRect(0, 0, width, height)
        self.view.setFixedSize(width, height)
        self.setFixedSize(self.view.size())
        
