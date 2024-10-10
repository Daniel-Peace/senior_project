import sys
import numpy as np
import cv2
import rospy
from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs.msg    import Image
from PyQt5.QtWidgets    import QApplication
from PyQt5.QtCore       import QTimer, pyqtSignal, QObject

from messages.msg import Assigned_apriltag
from apriltag_ros.msg   import AprilTagDetection, AprilTagDetectionArray

from components         import MainWindow
# ------------------------------------------------------------------------------------------------------

class Communicator(QObject):
    update_image_signal = pyqtSignal(np.ndarray)
    update_boxes_signal = pyqtSignal(list)
    update_id_signal    = pyqtSignal(str)

def handle_incoming_images(image):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert to RGB
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge Error: {e}")
        return

    np_image = np.array(cv_image_rgb)
    communicator.update_image_signal.emit(np_image)


def handle_assigned_apriltag(apriltag):
    id = apriltag.apriltag
    communicator.update_id_signal.emit(str(id))

def handle_detected_apriltags(msg):
    pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    rospy.init_node('gui_main', anonymous=True)

    communicator = Communicator()
    communicator.update_image_signal.connect(window.graphics_display.update_image)
    # communicator.update_id_signal.connect(window.card1.description_label.update_text)

    rospy.Subscriber('usb_cam/image_raw', Image, handle_incoming_images)
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, handle_detected_apriltags)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, handle_assigned_apriltag)

    timer = QTimer()
    timer.timeout.connect(lambda: rospy.rostime.wallsleep(0.01))
    timer.start(10)

    sys.exit(app.exec_())
