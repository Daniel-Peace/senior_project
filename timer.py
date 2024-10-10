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
from messages.msg import Current_timer

# general constants
RUN_DEBUG   = False
TIMER_TICK  = 0.2

# timer lengths
BUTTON_TIMER_LENGTH     = 5
APRILTAG_TIMER_LENGTH   = 10
PREDICTION_TIMER_LENGTH = 20

# timer types
APRILTAG_COUNTDOWN      = 0
APRILTAG_TIMER          = 1
PREDICTION_COUNTDOWN    = 2
PREDICTION_TIMER        = 3

# indexing directions
INDEX_FORWARD   = 1
INDEX_BACKWARD  = 0

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
apriltag_countdown_publisher    = rospy.Publisher('apriltag_countdown_status', Timer_status, queue_size=10)
apriltag_timer_publisher        = rospy.Publisher('apriltag_timer_status', Timer_status, queue_size=10)
prediction_countdown_publisher  = rospy.Publisher('prediction_countdown_status', Timer_status, queue_size=10)
prediction_timer_publisher      = rospy.Publisher('prediction_timer_status', Timer_status, queue_size=10)
current_timer_publisher         = rospy.Publisher('current_timer', Current_timer, queue_size=10)

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

def index_timer(direction):
    global current_timer
    if direction == 0:
        current_timer = (current_timer + 3) % 4
    else:
        current_timer = (current_timer + 1) % 4

# publishes the passed in "timer_status" to the current timer's ROS topic
def publish_timer_status(timer_status):
    global current_timer
    # publishing message
    system_print("Publishing timer status message")
    if current_timer == APRILTAG_COUNTDOWN:
        apriltag_countdown_publisher.publish(timer_status)
    elif current_timer == APRILTAG_TIMER:
        apriltag_timer_publisher.publish(timer_status)
    elif current_timer == PREDICTION_COUNTDOWN:
        prediction_countdown_publisher.publish(timer_status)
    else:
        prediction_timer_publisher.publish(timer_status)

# starts either a AprilTag timer or a prediction timer depending on timer_type
def timer(timer_length)-> int:
    current_timer_tick = 0
    timer_status = Timer_status()
    timer_status.timer_status = TIMER_STARTED
    while True:
        # printing time left in
        if (((current_timer_tick * 0.2) % 1) == 0):
            system_print("Time left: " + str(int(timer_length - (current_timer_tick * 0.2))))
            timer_status.time_left = int(timer_length - (current_timer_tick * 0.2))
            publish_timer_status(timer_status)

        # checking if timer is cancelled
        if button_pressed:
            system_print("Timer cancelled")
            timer_status.timer_status = TIMER_CANCELLED
            publish_timer_status(timer_status)
            return TIMER_CANCELLED
        
        # timer is finished
        if current_timer_tick == int(timer_length/TIMER_TICK):
            system_print("Timer ended")
            timer_status.timer_status = TIMER_ENDED
            publish_timer_status(timer_status)
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

    current_timer_msg = Current_timer()
    current_timer_msg.current_timer = current_timer
    current_timer_publisher.publish(current_timer_msg)
    
    while True:
        # checking if ctrl-c was entered
        if rospy.is_shutdown():
            break
        
        # checking if the button is not being pressed
        if (not button_pressed) and timer_ready:
            # starting countdown timer for 
            timer_state = timer(BUTTON_TIMER_LENGTH)

            # checking if the timer ended
            if timer_state == TIMER_ENDED:
                # indexing to the next timer
                index_timer(INDEX_FORWARD)
                current_timer_msg.current_timer = current_timer
                current_timer_publisher.publish(current_timer_msg)

                # checking what the current timer is
                if current_timer == APRILTAG_TIMER:
                    # starting AprilTag timer
                    timer_state = timer(APRILTAG_TIMER_LENGTH)
                else:
                    # starting prediction timer
                    timer_state = timer(PREDICTION_TIMER_LENGTH)

                # checking if timer ended
                if timer_state == TIMER_ENDED:
                    timer_ready = False
                    index_timer(INDEX_FORWARD)
                    current_timer_msg.current_timer = current_timer
                    current_timer_publisher.publish(current_timer_msg)
                else:
                    index_timer(INDEX_BACKWARD)
                    current_timer_msg.current_timer = current_timer
                    current_timer_publisher.publish(current_timer_msg)

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

        time.sleep(0.5)
        # starting timer sequence
        manage_timers()
