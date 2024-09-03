# imports
import rospy
from sensor_msgs.msg    import Imu
from sensor_msgs.msg    import NavSatFix
from messages.msg       import Pinned_apriltag

# stores the robots location
class Robot_location:
    def __init__(self, latitude = 0, longitude = 0, altitude = 0, heading = 0) -> None:
        self.latitude   = latitude
        self.longitude  = longitude
        self.altitude   = altitude
        self.heading    = heading

robot_location = Robot_location()

# initializing node
rospy.init_node('pin_apriltag', anonymous=True)

# initializing publisher
publisher   = rospy.Publisher('pinned_apriltag', Pinned_apriltag, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# this functions updates the object storing the robot's current location
def update_gps(msg):
    robot_location.latitude     = msg.latitude
    robot_location.longitude    = msg.longitude
    robot_location.altitude     = msg.altidtude

# updates the robots heading based on the Imu
def update_imu(msg):
    robot_location.heading = msg.heading


if __name__ == "__main__":
    # registering callback functions
    print("---------------------------------------------------------------------")
    system_print("Registering callback functions")
    # rospy.Subscriber('robot_location', NavSatFix, update_gps)
    # rospy.Subscriber('robot_direction', Imu, )
    # rospy.Subscriber('apriltag', , receive_model_predictions)