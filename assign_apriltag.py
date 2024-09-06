# imports
import rospy

# ROS messages
from messages.msg       import Assigned_apriltag
from apriltag_ros.msg   import AprilTagDetection

# global variables
current_april_tag = -1

# initializing node
rospy.init_node('assign_apriltag', anonymous=True)

# creating ROS topic and publisher
publisher = rospy.Publisher('assigned_apriltag', Assigned_apriltag, queue_size=10)



def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

def update_current_apriltag(april_tag_detections):
    for index, detection in enumerate(april_tag_detections.detections):
        if index == 0:
            current_april_tag = detection[0]
        else:
            pass

if __name__ == "__main__":

    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('tag_detections', AprilTagDetection, update_current_apriltag)
    

    while True:
        # prompting user
        print("---------------------------------------------------------------------")
        system_print("Enter the integer you would like to assign to the AprilTag")
        print("    or type q to exit:")

        while True:
            # getting user choice
            print("---------------------------------------------------------------------")
            choice = input("\u001b[34m -> \u001b[0m")


            # validating user input
            if choice.isdigit():
                assigned_apriltag = Assigned_apriltag()
                assigned_apriltag.apriltag = int(choice)
                publisher.publish(assigned_apriltag)
                break
            elif choice.upper() == 'Q':
                print("---------------------------------------------------------------------")
                system_print("Exiting...")
                exit(0)
            else:
                print("---------------------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")
