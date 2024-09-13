# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program handles starting and stopping timers for detecting AprilTags along with
# Starting all of the Predictions. It publishing Timer_status messages to
# "apriltag_timer_status" and "prediction_timer_status". It triggers these timers by
# detecting if a button on a controller has not been pressed for more than 5 seconds.
#
# The length of the timers can be changed using the following constants:
#   - BUTTON_TIMER_LENGTH
#   - APRILTAG_TIMER_LENGTH
#   - PREDICTION_TIMER_LENGTH
#
# A debug flag has been added to the top of the program. If this flag is set, you will
# be promped to start and stop the timers manually.
# -------------------------------------------------------------------------------------------

# imports
import time
import rospy

# ROS messages
from messages.msg import Command
from messages.msg import Timer_status

# general constants
RUN_DEBUG               = False
BUTTON_TIMER_LENGTH     = 5
APRILTAG_TIMER_LENGTH   = 10
PREDICTION_TIMER_LENGTH = 20
APRILTAG_TIMER          = 0
PREDICTION_TIMER        = 1

# global variables
button_pressed          = True
timer_ready             = False
not_pressed_counter     = 0
current_timer           = 0

# initializing node
rospy.init_node('prediction_timer_node', anonymous=True)

# creating ROS topic and publisher
apriltag_publisher      = rospy.Publisher('apriltag_timer_status', Timer_status, queue_size=10)
prediction_publisher    = rospy.Publisher('prediction_timer_status', Timer_status, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# flips the current_timer value back and forth between APRILTAG_TIMER and PREDICTION_TIMER
def flip_current_timer():
    global current_timer
    if current_timer == APRILTAG_TIMER:
        current_timer = PREDICTION_TIMER
    elif current_timer == PREDICTION_TIMER:
        current_timer = APRILTAG_TIMER

# starts either a AprilTag timer or a prediction timer depending on timer_type
def timer():
    global current_timer

    # creating Timer_status message to indicate timer has started
    timer_status = Timer_status()
    timer_status.timer_status = True
    timer_length = None

    # publishing message
    print("---------------------------------------------------------------------")
    system_print("Publishing timer status message")
    if current_timer == APRILTAG_TIMER:
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
    if current_timer == APRILTAG_TIMER:
        apriltag_publisher.publish(timer_status)
        timer_length = APRILTAG_TIMER_LENGTH
    else:
        prediction_publisher.publish(timer_status)
        timer_length = PREDICTION_TIMER_LENGTH

# updates button_pressed based on the command received
def handle_command(command):
    system_print("Received command")
    global button_pressed
    global timer_ready
    if command.chan5 > 0:
        button_pressed  = True
        timer_ready     = True
    else:
        button_pressed = False

# handles starting the timers when the button has not been pressed for BUTTON_TIMER_LENGTH seconds
def manage_timers():
    global button_pressed
    global not_pressed_counter
    global timer_ready

    system_print("System ready")

    while True:
        # checking if ctrl-c was entered
        if rospy.is_shutdown():
            break

        if button_pressed:
            # reseting no_pressed_counter
            not_pressed_counter = 0
        elif (not button_pressed) and timer_ready:
            # printing countdown banner
            if not_pressed_counter == 0:
                system_print("starting timer in:")

            # printing countdown
            if (((not_pressed_counter * 0.2) % 1) == 0):
                print("\t" + str(int((BUTTON_TIMER_LENGTH - (not_pressed_counter * 0.2)) + 0.5)) + " seconds")

            # incrementing not_pressed_counter
            not_pressed_counter += 1


            # checking if button timeout has occured
            if (not_pressed_counter * 0.2) == BUTTON_TIMER_LENGTH:
                # starting timer
                timer()

                # flipping timer
                flip_current_timer()

                # updating timer status to ensure button is pressed agin before starting a timer
                timer_ready = False

        time.sleep(0.2)

if __name__ == "__main__":
    if RUN_DEBUG:
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
                    current_timer = APRILTAG_TIMER
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

            # prompting user
            print("---------------------------------------------------------------------")
            system_print(" Type \"s\" to start timer for predictions or q to quit:")

            while True:
                # getting user choice
                print("---------------------------------------------------------------------")
                choice = input("\u001b[34m -> \u001b[0m")

                # validating user input
                if choice.upper() == 'S':
                    current_timer = PREDICTION_TIMER
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
    else:
        # registering callback function
        rospy.Subscriber('/button_status', Command, handle_command)

        # starting timer sequence
        manage_timers()
