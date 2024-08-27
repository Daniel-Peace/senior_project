# imports
import time
import rospy
from messages.msg import Timer_status

# constants
TIMER_LENGTH = 10

# creating ROS topic and publisher
publisher = rospy.Publisher('prediction_timer_status', Timer_status, queue_size=10)

def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

def timer():
    # creating Timer_status message to indicate timer has started
    timer_status = Timer_status()
    timer_status.timer_status = True

    # publishing message
    print("---------------------------------------------------------------------")
    system_print("Publishing timer status message")
    publisher.publish(timer_status)

    # starting timer
    system_print("Timer started for " + str(TIMER_LENGTH) + " seconds")
    time.sleep(TIMER_LENGTH)
    system_print("Timer ended")

    # updating timer message to indicate timer has ended
    timer_status.timer_status = False

    # publishing message
    system_print("Publishing timer status message")
    publisher.publish(timer_status)

if __name__ == "__main__":
    # initializing node
    rospy.init_node('prediction_timer_node', anonymous=True)

    while True:
        # prompting user
        print("---------------------------------------------------------------------")
        system_print(" Type \"s\" to start timer or q to quit:")

        while True:
            # getting user choice
            print("---------------------------------------------------------------------")
            choice = input("\u001b[34m -> \u001b[0m")

            # validating user input
            if choice.upper() == 'S':
                timer()
                break
            elif choice.upper() == 'Q':
                print("---------------------------------------------------------------------")
                system_print("Exiting...")
                print("---------------------------------------------------------------------")
                exit(0)
            else:
                print("---------------------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")
