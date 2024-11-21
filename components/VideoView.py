# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class creates a video feed for a usb camera. It also draws boxes around AprilTags
# that are currently being detected.
#
# -------------------------------------------------------------------------------------------

from PyQt5.QtCore       import Qt
from PyQt5.QtGui        import QImage, QPixmap, QPainter, QPen
from PyQt5.QtWidgets    import QGraphicsPixmapItem, QGraphicsScene, QGraphicsView, QWidget
from apriltag_ros.msg   import AprilTagDetectionArray
import numpy as np
import tf.transformations as tf_trans

FX = 1008.307177038531
FY = 1005.865107977325
CX = 643.2913125401469
CY = 401.0211161336514

detectionArray = AprilTagDetectionArray()
print(detectionArray)

# this class creates a video feed for a ros usb camera node
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
        self.pixmap = QPixmap.fromImage(qimage)

        if len(detectionArray.detections) != 0:
            current_distance = 999
            current_apriltag_index = -1
            for index, detection in enumerate(detectionArray.detections):
                if detection.pose.pose.pose.position.z < current_distance:
                    current_apriltag_index = index
                    current_distance = detection.pose.pose.pose.position.z

            for index, detection in enumerate(detectionArray.detections):
                top_left, bottom_right = self.get_bounding_box_2d(self.get_tag_corners_2d(detection.pose.pose.pose, 0.172))
                # drawing box around AprilTags
                self.painter = QPainter(self.pixmap)

                self.enRectangle = None
                if index == current_apriltag_index:
                    self.penRectangle = QPen(Qt.blue)
                else:
                    self.penRectangle = QPen(Qt.red)

                self.penRectangle.setWidth(3)
                self.painter.setPen(self.penRectangle)

                x = 0
                y = 0
                z = 0
                x = detection.pose.pose.pose.position.x
                y = detection.pose.pose.pose.position.y
                z = detection.pose.pose.pose.position.z
                
                # Calculate pixel coordinates
                u = (FX * x / z) + CX
                v = (FY * y / z) + CY
                
                u_int = int(round(u))
                v_int = int(round(v))
                
                boxWidth = bottom_right[0] - top_left[0]
                boxHeight = bottom_right[1] - top_left[1]

                self.painter.drawRect(u_int - boxWidth/2, v_int - boxHeight/2, boxWidth, boxHeight)
                self.painter.end()


# ---------------------------------------------- KEEP INCASE --------------------------------------------------------------
        # if len(detectionArray.detections) != 0:
        #     # looping over detections
        #     current_distance = 999
        #     current_apriltag_index = -1
        #     for index, detection in enumerate(detectionArray.detections):
        #         if detection.pose.pose.pose.position.z < current_distance:
        #             current_apriltag_index = index
        #             current_distance = detection.pose.pose.pose.position.z
        #     for index, detection in enumerate(detectionArray.detections):
        #         x = 0
        #         y = 0
        #         z = 0
        #         x = detection.pose.pose.pose.position.x
        #         y = detection.pose.pose.pose.position.y
        #         z = detection.pose.pose.pose.position.z

        #         # Check for division by zero
        #         if z == 0:
        #             print("z can't be zero")
                
        #         else:
        #             # drawing box around AprilTags
        #             self.painter = QPainter(self.pixmap)
        #             self.enRectangle = None
        #             if index == current_apriltag_index:
        #                 self.penRectangle = QPen(Qt.blue)
        #             else:
        #                 self.penRectangle = QPen(Qt.red)
        #             self.penRectangle.setWidth(3)
        #             self.painter.setPen(self.penRectangle)
                    
        #             # Calculate pixel coordinates
        #             u = (FX * x / z) + CX
        #             v = (FY * y / z) + CY
                    
        #             u_int = int(round(u))
        #             v_int = int(round(v))

        #             boxWidth = int(round((FX * 0.216) / z))
        #             boxHeight = int(round((FY * 0.279) / z))

        #             self.painter.drawRect(u_int - boxWidth/2,v_int - boxHeight/2,boxWidth,boxHeight)
        #             self.painter.end()
# -------------------------------------------------------------------------------------------------------------------------------

        self.pixmap_item.setPixmap(self.pixmap)
        width = self.pixmap.width()
        height = self.pixmap.height()

        # setting view size based on image size
        self.scene.setSceneRect(0, 0, width, height)
        self.view.setFixedSize(width, height)
        self.setFixedSize(self.view.size())

    # this functions receives an array of tag detections and updates the list of tags in this program
    def updateTagDetections(self, msg:AprilTagDetectionArray):
        global detectionArray 
        detectionArray = msg
    
    def get_tag_corners_2d(self, pose, size):
        # Extract position and quaternion from pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # Calculate 3D corner points relative to tag's center
        half_size = size / 2
        tag_corners_3d = np.array([
            [-half_size, -half_size, 0],
            [ half_size, -half_size, 0],
            [ half_size,  half_size, 0],
            [-half_size,  half_size, 0]
        ])
        
        # Apply rotation and translation based on pose
        rot_matrix = tf_trans.quaternion_matrix(quat)[:3, :3]
        tag_corners_3d_rotated = (rot_matrix @ tag_corners_3d.T).T + np.array([x, y, z])
        
        # Project 3D points to 2D using the camera intrinsics
        tag_corners_2d = []
        for corner in tag_corners_3d_rotated:
            # Perspective projection using FX, FY, CX, CY
            u = (FX * corner[0] / corner[2]) + CX
            v = (FY * corner[1] / corner[2]) + CY
            tag_corners_2d.append((u, v))
        
        return np.array(tag_corners_2d)
    
    def get_bounding_box_2d(self, tag_corners_2d):
        # Extract min and max x and y values from the four corner points
        x_coords = tag_corners_2d[:, 0]
        y_coords = tag_corners_2d[:, 1]
        
        top_left = (min(x_coords), min(y_coords))
        bottom_right = (max(x_coords), max(y_coords))
        
        return top_left, bottom_right
