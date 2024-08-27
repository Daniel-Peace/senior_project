# imports
import rospy
from messages.msg import Assigned_apriltag

# creating ROS topic and publisher
publisher = rospy.Publisher('assigned_apriltag', Assigned_apriltag, queue_size=10)

def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

if __name__ == "__main__":
    # initializing node
    rospy.init_node('assign_apriltag', anonymous=True)

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
                system_print("Invalid choice")