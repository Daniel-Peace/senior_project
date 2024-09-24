# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program keeps track of the closest AprilTag to the camera when a scan is initiated.
# It receives AprilTag data from the "tag_detections" topic and begins updating the current
# AprilTag when it receives a timer_status message of true from the "apriltag_timer_status"
# topic. Once it receives a timer_status mesage of false from the "apriltag_timer_status"
# topic, it stops updating the current apriltag even if this program receives further
# AprilTag detections. Upon receiving a timer_status message of True form the
# "prediction_timer_status", it publishes the current AprilTag to the "assigned_apriltag"
# topic. When the prediction timer ends, all tracking values are reset to be ready for the
# next scan.
# -------------------------------------------------------------------------------------------

# imports
import time
import rospy
from sys                import getallocatedblocks

# ROS messages
from messages.msg       import Assigned_apriltag
from messages.msg       import Timer_status
from apriltag_ros.msg   import AprilTagDetection, AprilTagDetectionArray

# timer states
TIMER_ENDED  = 0
TIMER_STARTED    = 1
TIMER_CANCELLED  = 2

# global variables
current_apriltag        = -1
current_distance        = 999
apriltag_timer_started  = False

# initializing node
rospy.init_node('assign_apriltag', anonymous=True)

# creating publisher
publisher = rospy.Publisher('assigned_apriltag', Assigned_apriltag, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# resets program for next scan
def reset():
    system_print("Resetting global variables for next scan")
    global current_apriltag
    global current_distance
    global apriltag_timer_started
    current_apriltag        = -1
    current_distance        = 999
    apriltag_timer_started  = False

# updates the current AprilTag to the closest AprilTag in view of the camera
def update_current_apriltag(april_tag_detections):
    # checking if an AprilTag has been detected
    if len(april_tag_detections.detections)  != 0:
        system_print("Detected AprilTag")

        # checking if AprilTag timer has started
        if apriltag_timer_started:
            global current_apriltag
            global current_distance

            # looping over detections
            for index, detection in enumerate(april_tag_detections.detections):
                if detection.pose.pose.pose.position.z < current_distance:
                    current_apriltag = detection.id[0]
                    system_print("Current AprilTag: " + str(current_apriltag))
                    current_distance = detection.pose.pose.pose.position.z

# handles actions that take place on timer start and stop
def handle_apriltag_timer_status(timer):
    global apriltag_timer_started
    if timer.timer_status == TIMER_STARTED:
        system_print("Timer started")
        reset()
        apriltag_timer_started = True
        system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))
    elif timer.timer_status == TIMER_ENDED:
        apriltag_timer_started = False
        system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))
        system_print("Timer ended")
        system_print("Final AprilTag: " + str(current_apriltag))
    else:
        apriltag_timer_started = False
        system_print("Timer cancelled")
        reset()

# publishes current AprilTag whent the prediction timer ends
def handle_prediction_timer_status(timer):
    # checking if the prediction timer started
    if timer.timer_status == TIMER_STARTED:
        system_print("Prediction timer started")

        # creating ROS message
        assigned_apriltag = Assigned_apriltag()

        # initializing the message
        assigned_apriltag.apriltag = current_apriltag

        # publishing the current AprilTag
        system_print("Publishing current AprilTag")
        publisher.publish(assigned_apriltag)
    elif timer.timer_status == TIMER_ENDED:
        system_print("Prediction timer ended")
    else:
        system_print("Timer cancelled")

if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, update_current_apriltag)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handle_apriltag_timer_status)
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_prediction_timer_status)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
