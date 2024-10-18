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

# ROS messages
from messages.msg       import Assigned_apriltag
from messages.msg       import Timer_status
from apriltag_ros.msg   import AprilTagDetectionArray

# timer states
TIMER_ENDED  = 0
TIMER_STARTED    = 1
TIMER_CANCELLED  = 2

# global variables
current_apriltag        = -1
current_distance        = 999
apriltag_timer_started  = False
previous_timer_status = -1

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

def publishApriltag():
    system_print("Publishing Apriltag")
    assignedApriltag = Assigned_apriltag()
    assignedApriltag.apriltag = current_apriltag
    publisher.publish(assignedApriltag)

# updates the current AprilTag to the closest AprilTag in view of the camera
def updateCurrentApriltag(msg:AprilTagDetectionArray):
    # checking if an AprilTag has been detected
    if len(msg.detections) != 0:
        system_print("Detected AprilTag")

        # checking if AprilTag timer has started
        if apriltag_timer_started:
            global current_apriltag
            global current_distance

            # looping over detections
            for index, detection in enumerate(msg.detections):
                if detection.pose.pose.pose.position.z < current_distance:
                    current_apriltag = detection.id[0]
                    system_print("Current AprilTag: " + str(current_apriltag))
                    current_distance = detection.pose.pose.pose.position.z

            publishApriltag()            

# handles actions that take place when the apriltag timer starts
def onApriltagTimerStart():
    global apriltag_timer_started
    system_print("AprilTag timer has started")
    reset()
    apriltag_timer_started = True
    system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))

# handles actions that take place when the apriltag timer ends
def onApriltagTimerEnd():
    global apriltag_timer_started
    system_print("AprilTag timer has ended")
    apriltag_timer_started = False
    system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))
    system_print("Final AprilTag: " + str(current_apriltag))

# handles actions that take place when the apriltag timer is canceled
def onApriltagTimerCancel():
    global apriltag_timer_started
    system_print("AprilTag timer has been cancelled")
    apriltag_timer_started = False
    system_print("Timer cancelled")
    reset()

# handles actions that take place on timer start and stop
def handleApriltagTimerStatus(timer):
    global previous_timer_status

    # checking if the timer status has changed
    if previous_timer_status != timer.timer_status:
        # updating AprilTag timer state
        previous_timer_status = timer.timer_status

        # checking timer state
        if timer.timer_status == TIMER_STARTED:
            onApriltagTimerStart()
        elif timer.timer_status == TIMER_ENDED:
            onApriltagTimerEnd()
        else:
            onApriltagTimerCancel()

if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, updateCurrentApriltag)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handleApriltagTimerStatus)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
