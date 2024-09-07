# imports
from pickle import PROTO
import time
import rospy
from messages.msg import Timer_status

# constants
APRILTAG_TIMER_LENGTH   = 10
PREDICTION_TIMER_LENGTH = 20
APRILTAG_TIMER      = 0
PREDICTION_TIMER    = 1

# creating ROS topic and publisher
apriltag_publisher      = rospy.Publisher('apriltag_timer_status', Timer_status, queue_size=10)
prediction_publisher    = rospy.Publisher('prediction_timer_status', Timer_status, queue_size=10)

def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

def timer(timer_type):
    # creating Timer_status message to indicate timer has started
    timer_status = Timer_status()
    timer_status.timer_status = True

    timer_length = None

    # publishing message
    print("---------------------------------------------------------------------")
    system_print("Publishing timer status message")
    if timer_type == APRILTAG_TIMER:
        apriltag_publisher.publish(timer_status)
        timer_length = APRILTAG_TIMER_LENGTH
    else:
        prediction_publisher.publish(timer_status)
        timer_length = PREDICTION_TIMER_LENGTH

    # starting timer
    system_print("Timer started for " + str(timer_length) + " seconds")
    time.sleep(timer_length)
    system_print("Timer ended")

    # updating timer message to indicate timer has ended
    timer_status.timer_status = False

    # publishing message
    system_print("Publishing timer status message")
    if timer_type == APRILTAG_TIMER:
        apriltag_publisher.publish(timer_status)
        timer_length = APRILTAG_TIMER_LENGTH
    else:
        prediction_publisher.publish(timer_status)
        timer_length = PREDICTION_TIMER_LENGTH


if __name__ == "__main__":
    # initializing node
    rospy.init_node('prediction_timer_node', anonymous=True)

    while True:
        # prompting user
        print("---------------------------------------------------------------------")
        system_print(" Type \"s\" to start timer for apriltag or q to quit:")

        while True:
            # getting user choice
            print("---------------------------------------------------------------------")
            choice = input("\u001b[34m -> \u001b[0m")

            # validating user input
            if choice.upper() == 'S':
                timer(APRILTAG_TIMER)
                break
            elif choice.upper() == 'Q':
                print("---------------------------------------------------------------------")
                system_print("Exiting...")
                print("---------------------------------------------------------------------")
                exit(0)
            else:
                print("---------------------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")

        # prompting user
        print("---------------------------------------------------------------------")
        system_print(" Type \"s\" to start timer for predictions or q to quit:")

        while True:
            # getting user choice
            print("---------------------------------------------------------------------")
            choice = input("\u001b[34m -> \u001b[0m")

            # validating user input
            if choice.upper() == 'S':
                timer(PREDICTION_TIMER)
                break
            elif choice.upper() == 'Q':
                print("---------------------------------------------------------------------")
                system_print("Exiting...")
                print("---------------------------------------------------------------------")
                exit(0)
            else:
                print("---------------------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")
