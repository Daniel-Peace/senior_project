import sys
import rospy
import cv2
import numpy as np
from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs.msg    import Image
from PyQt5.QtWidgets    import QApplication
from PyQt5.QtCore       import QObject, pyqtSignal, QTimer
from components         import MainWindow2

# creates signals for updating UI
class Communicator(QObject):
    # -------------------------------------------------------
    # ADD NEW SIGNALS HERE
    # -------------------------------------------------------
    updateImageSignal = pyqtSignal(np.ndarray)


# -------------------------------------------------------
# ADD CALLBACK FUNCTIONS HERE
# -------------------------------------------------------

# handles incoming images from the usb_cam/image_raw ROS topic
def handle_incoming_images(msg):
    # creating CV bridge to convert ROS images to CV images
    bridge = CvBridge()

    try:
        # attempting to convert ROS Image to CV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # converting the color scheme from BGR to RGB
        correctedCvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    except CvBridgeError as e:
        # logging error if conversion above failed
        rospy.logerr(f"CV Bridge Error: {e}")
        return
    
    # sending image to videoView in window
    communicator.updateImageSignal.emit(correctedCvImage)
    
    

# "main function"
if __name__ == "__main__":
    # creating main app
    app = QApplication(sys.argv)

    # creating main window
    window = MainWindow2()

    # creating communicator object facilitates the use of signals
    communicator = Communicator()

    # -------------------------------------------------------
    # CONNECT SIGNALS HERE
    # -------------------------------------------------------
    communicator.updateImageSignal.connect(window.videoView.update_image)

    # showing main window
    window.show()

    # initializing gui as ROS node
    rospy.init_node('gui_main', anonymous=True)

    # creating communicator object facilitates the use of signals
    communicator = Communicator()
    communicator.updateImageSignal.connect(window.videoView.update_image)

    # -------------------------------------------------------
    # ADD ROS TOPICS HERE
    # -------------------------------------------------------
    rospy.Subscriber('usb_cam/image_raw', Image, handle_incoming_images)

    # pausing to allow ROS callback function and the UI to sync
    timer = QTimer()
    timer.timeout.connect(lambda: rospy.rostime.wallsleep(0.01))
    timer.start(10)

    sys.exit(app.exec_())