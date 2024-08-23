# imports
import rospy
from messages.msg import Assigned_apriltag

# creating ROS topic and publisher
publisher = rospy.Publisher('assigned_apriltag', Assigned_apriltag, queue_size=10)

if __name__ == "__main__":
    # initializing node
    rospy.init_node('locate_correct_apriltag', anonymous=True)

    # prompting user
    print("system: Type \"s\" to start timer or q to quit:")

    while True:
        # getting user choice
        choice = input("-> ")

        # validating user input
        if choice.isdigit():
            assigned_apriltag = Assigned_apriltag()
            assigned_apriltag.apriltag = int(choice)
            publisher.publish(assigned_apriltag)
        elif choice.upper() == 'Q':
            print("system: Exiting...")
            exit(0)
        else:
            print("system: invalid choice")
