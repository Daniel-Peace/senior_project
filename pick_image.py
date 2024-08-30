# imports
import rospy
from sensor_msgs.msg    import Image

# creating publisher
publisher = rospy.Publisher('picked_image', Image, queue_size=10)

# creating gloabl variable to hold current image
current_image = Image()

# gets the current image from 'usb_cam/image_raw'
def get_current_image(image):
    global current_image
    current_image = image
# publishes current image to be used by yoloV8.py
def publish_image(timer):
    if timer.timer_status == True:
        publisher.publish(current_image)


# registering callback functions
rospy.Subscriber('usb_cam/image_raw', Image, get_current_image)