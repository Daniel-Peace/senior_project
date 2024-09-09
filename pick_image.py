# -------------------------------------------------------------------------------------------
# Daniel Peace
# Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program keeps track of the current image published to the "usb_cam/image_raw" topic.
# When a timer_status message of true is received from the "prediction_timer_status" topic,
# it publishes the current image to the "picked_image" topic.
# -------------------------------------------------------------------------------------------

# imports
import rospy
import time

# ROS messages
from sensor_msgs.msg    import Image
from messages.msg       import Timer_status

# initializing node
rospy.init_node('pick_image', anonymous=True)

# creating publisher
publisher = rospy.Publisher('picked_image', Image, queue_size=10)

# creating global variable to hold current image
current_image = Image()

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# updates the current image with the latest image from 'usb_cam/image_raw'
def update_current_image(image):
    global current_image
    current_image = image

# publishes current image to "picked_image" topic
def publish_image():
    system_print("Publishing current image")
    publisher.publish(current_image)

# handles actions corresponding to the timer starting and stopping
def handle_timer_status(msg):
    if msg.timer_status == True:
        system_print("Prediction timer started")
        publish_image()
    else:
        system_print("Prediction timer ended")

if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('usb_cam/image_raw', Image, update_current_image)
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_timer_status)

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)

    print("\n---------------------------------------------------------------------")
