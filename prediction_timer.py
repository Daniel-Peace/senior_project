# imports
import time
import rospy
from messages.msg import Timer_status

# constants
TIMER_LENGTH = 10

# creating ROS topic and publisher
publisher = rospy.Publisher('prediction_timer_status', Timer_status, queue_size=10)

def handle_timer():
    # creating Timer_status message to indicate timer has started
    timer_status = Timer_status()
    timer_status.timer_status = True

    # publishing message
    print("system: Publishing timer status message")
    publisher.publish(timer_status)

    # starting timer
    print("system: Timer started for " + str(TIMER_LENGTH) + " seconds")
    time.sleep(TIMER_LENGTH)
    print("Timer ended")

    # updating timer message to indicate timer has ended
    timer_status.timer_status = False

    # publishing message
    print("system: Publishing timer status message")
    publisher.publish(timer_status)

if __name__ == "__main__":
    # initializing node
    rospy.init_node('prediction_timer_node', anonymous=True)

    # prompting user
    print("system: Type \"s\" to start timer or q to quit:")

    while True:
        # getting user choice
        choice = input("-> ")

        # validating user input
        if choice.upper() == 'S':
            handle_timer()
        elif choice.upper() == 'Q':
            print("system: Exiting...")
            exit(0)
        else:
            print("system: invalid choice")