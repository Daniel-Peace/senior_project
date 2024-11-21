#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program tracks which AprilTag is clostest to camera and publishes the tag once
# the apriltag timer has started. Once the timer has ended it stops publishing the Apriltag
# and resets the program for the next apriltag scan. If the timer is canceled, it will also
# reset the program. 
# -------------------------------------------------------------------------------------------

import time
import rospy
from messages.msg       import Assigned_apriltag
from messages.msg       import Timer_status
from apriltag_ros.msg   import AprilTagDetectionArray

# timer states (enum)
TIMER_ENDED         = 0
TIMER_STARTED       = 1
TIMER_CANCELLED     = 2

# global variables
current_apriltag        = -1
current_distance        = 999
apriltag_timer_started  = False
previous_timer_status   = -1

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

# this function publishes the currently picked apriltag
def publish_apriltag():
    system_print("Publishing Apriltag")
    assignedApriltag = Assigned_apriltag()
    assignedApriltag.apriltag = current_apriltag
    publisher.publish(assignedApriltag)

# updates the current AprilTag to the closest AprilTag in view of the camera
def update_current_apriltag(msg:AprilTagDetectionArray):
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

            publish_apriltag()            

# handles actions that take place when the apriltag timer starts
def on_apriltag_timer_start():
    global apriltag_timer_started

    system_print("AprilTag timer has started")
    reset()
    apriltag_timer_started = True
    system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))

# handles actions that take place when the apriltag timer ends
def on_apriltag_timer_end():
    global apriltag_timer_started

    system_print("AprilTag timer has ended")
    apriltag_timer_started = False
    system_print("\"timer_started\" updated to: " + str(apriltag_timer_started))
    system_print("Final AprilTag: " + str(current_apriltag))

# handles actions that take place when the apriltag timer is canceled
def on_apriltag_timer_cancel():
    global apriltag_timer_started

    system_print("AprilTag timer has been cancelled")
    apriltag_timer_started = False
    system_print("Timer cancelled")
    reset()

# handles actions that take place on timer start and stop
def handle_apriltag_timer_status(timer):
    global previous_timer_status

    # checking if the timer status has changed
    if previous_timer_status != timer.timer_status:
        # updating AprilTag timer state
        previous_timer_status = timer.timer_status

        # checking timer state
        if timer.timer_status == TIMER_STARTED:
            on_apriltag_timer_start()
        elif timer.timer_status == TIMER_ENDED:
            on_apriltag_timer_end()
        else:
            on_apriltag_timer_cancel()

# "main function" of the program
if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, update_current_apriltag)
    rospy.Subscriber('apriltag_scanning_timer_state', Timer_status, handle_apriltag_timer_status)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
