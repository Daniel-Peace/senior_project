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
from messages.msg import Timer_status
from messages.msg import Command

# general constants
RUN_DEBUG   = False
TIMER_TICK  = 0.2

# timer lengths
BUTTON_TIMER_LENGTH     = 5
APRILTAG_TIMER_LENGTH   = 10
PREDICTION_TIMER_LENGTH = 20

# timer types
APRILTAG_TIMER      = 0
PREDICTION_TIMER    = 1

# timer states
TIMER_ENDED      = 0
TIMER_STARTED    = 1
TIMER_CANCELLED  = 2

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

# publishes the passed in "timer_status" to the current timer's ROS topic
def publish_timer_status(timer_status):
    global current_timer
    # publishing message
    system_print("Publishing timer status message")
    if current_timer == APRILTAG_TIMER:
        apriltag_publisher.publish(timer_status)
    else:
        prediction_publisher.publish(timer_status)

# starts either a AprilTag timer or a prediction timer depending on timer_type
def timer(timer_length)-> int:
    current_timer_tick = 0
    while True:
        # printing time left in
        if (((current_timer_tick * 0.2) % 1) == 0):
            system_print("Time left: " + str(int(timer_length - (current_timer_tick * 0.2))))

        # checking if timer is cancelled
        if button_pressed:
            system_print("Timer cancelled")
            return TIMER_CANCELLED
        
        # timer is finished
        if current_timer_tick == int(timer_length/TIMER_TICK):
            system_print("Timer ended")
            return TIMER_ENDED

        # incrementing timer tick
        current_timer_tick += 1
        time.sleep(0.2)

# updates button_pressed based on the command received
def check_button(command):
    system_print("Received command")
    global button_pressed
    global timer_ready

    # checking if the trigger is pressed
    if command.chan8 > 1600:
        system_print("Trigger is pressed")
        button_pressed  = True
        timer_ready     = True
    else:
        system_print("Trigger is not pressed")
        button_pressed = False




# handles starting the timers when the button has not been pressed for BUTTON_TIMER_LENGTH seconds
def manage_timers():
    system_print("System ready")

    # bringing global variables into this scope
    global button_pressed
    global not_pressed_counter
    global timer_ready

    while True:
        # checking if ctrl-c was entered
        if rospy.is_shutdown():
            break
        
        # checking if button is pressed
        if button_pressed:
            # reseting not_pressed_counter
            not_pressed_counter = 0
        elif (not button_pressed) and timer_ready:
            # printing countdown banner
            if not_pressed_counter == 0:
                system_print("starting timer in:")

            # printing countdown
            if (((not_pressed_counter * 0.2) % 1) == 0):
                print("\t" + str(int((BUTTON_TIMER_LENGTH - (not_pressed_counter * 0.2)) + 0.5)) + " seconds")

            # checking if button timeout has occured
            if (not_pressed_counter * 0.2) == BUTTON_TIMER_LENGTH:
                # creating Timer_status object
                timer_status = Timer_status()

                # publishing timer started status
                timer_status.timer_status = TIMER_STARTED
                publish_timer_status(timer_status)

                # starting timer
                timer_state = None
                if current_timer == PREDICTION_TIMER:
                    timer_state = timer(PREDICTION_TIMER_LENGTH)
                else:
                    timer_state = timer(APRILTAG_TIMER_LENGTH)

                # checking if the timer ended or was cancelled
                if timer_state == TIMER_ENDED:
                    timer_status.timer_status = TIMER_ENDED
                    publish_timer_status(timer_status)
                    timer_ready = False
                    flip_current_timer()
                else:
                    timer_status.timer_status = TIMER_CANCELLED
                    publish_timer_status(timer_status)

            # incrementing not_pressed_counter
            not_pressed_counter += 1

        # slowing loop down
        time.sleep(0.2)

if __name__ == "__main__":
    if RUN_DEBUG:
        # registering callback function
        rospy.Subscriber('/button_status', Command, check_button)

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
        #rospy.Subscriber('/drone_commands', Command, check_button)
        rospy.Subscriber('/button_status', Command, check_button)

        # starting timer sequence
        manage_timers()
