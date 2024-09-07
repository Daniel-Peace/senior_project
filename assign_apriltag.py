# imports
from sys import getallocatedblocks
import rospy
import time

# ROS messages
from messages.msg       import Assigned_apriltag
from messages.msg       import Timer_status
from apriltag_ros.msg   import AprilTagDetection, AprilTagDetectionArray

# global variables
current_apriltag    = 999
current_distance    = 999
timer_started       = False

# initializing node
rospy.init_node('assign_apriltag', anonymous=True)

# creating ROS topic and publisher
publisher = rospy.Publisher('assigned_apriltag', Assigned_apriltag, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

def reset():
    system_print("Resetting global variables")
    global current_apriltag
    global current_distance
    global timer_started
    current_apriltag    = 999
    current_distance    = 999
    timer_started       = False

# updates "current_apriltag" to the latest apriltag scanned
def update_current_apriltag(april_tag_detections):
    if len(april_tag_detections.detections)  != 0:
        system_print("Detected AprilTag")
    if timer_started:
        global current_apriltag
        global current_distance

        # looping over detections
        for index, detection in enumerate(april_tag_detections.detections):
            if detection.pose.pose.pose.position.z < current_distance:
                current_apriltag = detection.id[0]
                system_print("Current AprilTag: " + str(current_apriltag))
                current_distance = detection.pose.pose.pose.position.z

# handles actions that take place on timer start and stop
def handle_timer_status(timer):
    global timer_started
    if timer.timer_status:
        system_print("Timer started")
        reset()
        timer_started = True
        system_print("\"timer_started\" updated to: " + str(timer_started))
    else:
        timer_started = False
        system_print("\"timer_started\" updated to: " + str(timer_started))
        system_print("Timer ended")
        system_print("Final AprilTag: " + str(current_apriltag))

def publish_apriltag(timer):
    if timer.timer_status:
        assigned_apriltag = Assigned_apriltag()
        assigned_apriltag.apriltag = current_apriltag
        publisher.publish(assigned_apriltag)



if __name__ == "__main__":
    # registering callback functions
    print("---------------------------------------------------------------------")
    system_print("Registering callback functions")
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, update_current_apriltag)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handle_timer_status)
    rospy.Subscriber('prediction_timer_status', Timer_status, publish_apriltag)
    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
