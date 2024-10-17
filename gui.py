import sys
import rospy
import cv2
import numpy as np
from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs.msg    import Image
from messages.msg       import Timer_status, Current_timer, Casualty_prediction, ModelPredictionStatuses
from apriltag_ros.msg   import AprilTagDetectionArray
from PyQt5.QtWidgets    import QApplication
from PyQt5.QtCore       import QObject, pyqtSignal, QTimer
from components         import MainWindow
from casualty           import Casualty

# global variables
reportNumber = 0

# creates signals for updating UI
class Communicator(QObject):
    # -------------------------------------------------------
    # ADD NEW SIGNALS HERE
    # -------------------------------------------------------
    updateImageSignal                   = pyqtSignal(np.ndarray)
    updateAprilTagCountdown             = pyqtSignal(str)
    updateAprilTagTimer                 = pyqtSignal(str)
    updatePredictionCountdown           = pyqtSignal(str)
    updatePredictionTimer               = pyqtSignal(str)
    updateBackgroundAprilTagCountdown   = pyqtSignal(str)
    updateTextColorAprilTagCountdown    = pyqtSignal(str)
    updateBackgroundAprilTagTimer       = pyqtSignal(str)
    updateTextColorAprilTagTimer        = pyqtSignal(str)
    updateBackgroundPredictionCountdown = pyqtSignal(str)
    updateTextColorPredictionCountdown  = pyqtSignal(str)
    updateBackgroundPredictionTimer     = pyqtSignal(str)
    updateTextColorPredictionTimer      = pyqtSignal(str)
    updateCurrentTagDetections          = pyqtSignal(str)
    updateCurrentPredictions            = pyqtSignal(Casualty)
    updateReportList                    = pyqtSignal(str, Casualty)
    updateModelReportStatuses           = pyqtSignal(str)


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

def handle_apriltag_countdown(msg):
    timer_status    = msg.timer_status
    time_left       = msg.time_left
    if timer_status == 1:
        communicator.updateAprilTagCountdown.emit(str(time_left))
    else:
        communicator.updateAprilTagCountdown.emit('--')

def handle_apriltag_timer(msg):
    timer_status    = msg.timer_status
    time_left       = msg.time_left
    if timer_status == 1:
        communicator.updateAprilTagTimer.emit(str(time_left))
    else:
        communicator.updateAprilTagTimer.emit('--')

def handle_prediction_countdown(msg):
    timer_status    = msg.timer_status
    time_left       = msg.time_left
    if timer_status == 1:
        communicator.updatePredictionCountdown.emit(str(time_left))
    else:
        communicator.updatePredictionCountdown.emit('--')

def handle_prediction_timer(msg):
    timer_status    = msg.timer_status
    time_left       = msg.time_left
    if timer_status == 1:
        communicator.updatePredictionTimer.emit(str(time_left))
    else:
        communicator.updatePredictionTimer.emit('--')

def handle_current_timer(msg):
    current_timer = msg.current_timer

    if current_timer == 0:
        # changing to active colors
        communicator.updateBackgroundAprilTagCountdown.emit("#A7C087")
        communicator.updateTextColorAprilTagCountdown.emit("#282C32")

        # changing back to origial colors
        communicator.updateBackgroundPredictionTimer.emit("#2F343E")
        communicator.updateTextColorPredictionTimer.emit("#ADB2BD")
        communicator.updateBackgroundAprilTagTimer.emit("#2F343E")
        communicator.updateTextColorAprilTagTimer.emit("#ADB2BD")

    elif current_timer == 1:
        # changing to active colors
        communicator.updateBackgroundAprilTagTimer.emit("#A7C087")
        communicator.updateTextColorAprilTagTimer.emit("#282C32")

        # changing back to origial colors
        communicator.updateBackgroundAprilTagCountdown.emit("#2F343E")
        communicator.updateTextColorAprilTagCountdown.emit("#ADB2BD")
        communicator.updateBackgroundPredictionCountdown.emit("#2F343E")
        communicator.updateTextColorPredictionCountdown.emit("#ADB2BD")

    elif current_timer == 2:
        # changing to active colors
        communicator.updateBackgroundPredictionCountdown.emit("#A7C087")
        communicator.updateTextColorPredictionCountdown.emit("#282C32")

        # changing back to origial colors
        communicator.updateBackgroundAprilTagTimer.emit("#2F343E")
        communicator.updateTextColorAprilTagTimer.emit("#ADB2BD")
        communicator.updateBackgroundPredictionTimer.emit("#2F343E")
        communicator.updateTextColorPredictionTimer.emit("#ADB2BD")

    else:
        # changing to active colors
        communicator.updateBackgroundPredictionTimer.emit("#A7C087")
        communicator.updateTextColorPredictionTimer.emit("#282C32")

        # changing back to origial colors
        communicator.updateBackgroundPredictionCountdown.emit("#2F343E")
        communicator.updateTextColorPredictionCountdown.emit("#ADB2BD")
        communicator.updateBackgroundAprilTagCountdown.emit("#2F343E")
        communicator.updateTextColorAprilTagCountdown.emit("#ADB2BD")

def handle_current_tag_detections(msg):
    detectionList = ""

    # checking if a detection was made
    if len(msg.detections)  != 0:
        first_iteration = True

        # looping over detections
        for detections in msg.detections:
            # checking if this is the first iteration
            if first_iteration:
                first_iteration = False
                detectionList += ("id: " + str(detections.id[0]))
            else:
                detectionList += ("\nid: " + str(detections.id[0]))
    else:
        detectionList += "id: --"

    communicator.updateCurrentTagDetections.emit(detectionList)

def handle_finalized_reports(msg:Casualty_prediction):
    global reportNumber
    casualty = Casualty()
    casualty.apriltag               = msg.apriltag
    casualty.is_coherent            = msg.is_coherent
    casualty.time_ago               = msg.time_ago
    casualty.severe_hemorrhage      = msg.severe_hemorrhage
    casualty.respiratory_distress   = msg.respiratory_distress
    casualty.heart_rate             = msg.heart_rate
    casualty.respiratory_rate       = msg.respiratory_rate
    casualty.trauma_head            = msg.trauma_head
    casualty.trauma_torso           = msg.trauma_torso
    casualty.trauma_lower_ext       = msg.trauma_lower_ext
    casualty.trauma_upper_ext       = msg.trauma_upper_ext
    casualty.alertness_ocular       = msg.alertness_ocular
    casualty.alertness_verbal       = msg.alertness_verbal
    casualty.alertness_motor        = msg.alertness_motor
    communicator.updateCurrentPredictions.emit(casualty)
    reportTitle = "Report " + str(reportNumber)
    reportNumber += 1
    communicator.updateReportList.emit(reportTitle, casualty)

def handle_model_prediction_statuses(msg:ModelPredictionStatuses):
    status_list = ""
    for model_prediction_status in msg.modelPredictionStatuses:
        status_list += "Model " + str(model_prediction_status.model_number) + ": " + str(model_prediction_status.made_prediction) + "\n"

    communicator.updateModelReportStatuses.emit(status_list)
    
# "main function"
if __name__ == "__main__":
    # initializing gui as ROS node
    rospy.init_node('gui_main', anonymous=True)

    # creating main app
    app = QApplication(sys.argv)

    # creating main window
    window = MainWindow()  

    # creating communicator object facilitates the use of signals
    communicator = Communicator()

    # -------------------------------------------------------
    # CONNECT SIGNALS HERE
    # -------------------------------------------------------
    communicator.updateImageSignal.connect(window.videoView.update_image)
    communicator.updateImageSignal.connect(window.videoView.update_image)
    communicator.updateAprilTagCountdown.connect(window.aprilTagCountdownCard.updateBody)
    communicator.updateBackgroundAprilTagCountdown.connect(window.aprilTagCountdownCard.updateBackgroundColor)
    communicator.updateTextColorAprilTagCountdown.connect(window.aprilTagCountdownCard.updateTextColor)
    communicator.updateAprilTagTimer.connect(window.aprilTagTimerCard.updateBody)
    communicator.updateBackgroundAprilTagTimer.connect(window.aprilTagTimerCard.updateBackgroundColor)
    communicator.updateTextColorAprilTagTimer.connect(window.aprilTagTimerCard.updateTextColor)
    communicator.updatePredictionCountdown.connect(window.predictionCountdownCard.updateBody)
    communicator.updateBackgroundPredictionCountdown.connect(window.predictionCountdownCard.updateBackgroundColor)
    communicator.updateTextColorPredictionCountdown.connect(window.predictionCountdownCard.updateTextColor)
    communicator.updatePredictionTimer.connect(window.predictionTimerCard.updateBody)
    communicator.updateBackgroundPredictionTimer.connect(window.predictionTimerCard.updateBackgroundColor)
    communicator.updateTextColorPredictionTimer.connect(window.predictionTimerCard.updateTextColor)
    communicator.updateCurrentTagDetections.connect(window.currentDetections.updateBody)
    communicator.updateCurrentPredictions.connect(window.predictions.updateReportValues)
    communicator.updateReportList.connect(window.reportList.list.addItemToList)
    window.reportList.list.itemClicked.connect(window.predictions.updateOnClick)
    communicator.updateModelReportStatuses.connect(window.modelPredictionStatuses.updateBody)

    # -------------------------------------------------------
    # ADD ROS TOPICS HERE
    # -------------------------------------------------------
    rospy.Subscriber('usb_cam/image_raw', Image, handle_incoming_images)
    rospy.Subscriber('apriltag_countdown_status', Timer_status, handle_apriltag_countdown)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handle_apriltag_timer)
    rospy.Subscriber('prediction_countdown_status', Timer_status, handle_prediction_countdown)
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_prediction_timer)
    rospy.Subscriber('current_timer', Current_timer, handle_current_timer)
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, handle_current_tag_detections)
    rospy.Subscriber('final_report', Casualty_prediction, handle_finalized_reports)
    rospy.Subscriber('model_prediction_statuses', ModelPredictionStatuses, handle_model_prediction_statuses)

    # showing main window
    window.show()

    # pausing to allow ROS callback function and the UI to sync
    timer = QTimer()
    timer.timeout.connect(lambda: rospy.rostime.wallsleep(0.01))
    timer.start(10)

    sys.exit(app.exec_())