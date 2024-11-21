#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program handles timers surrounding AprilTag detections and affliction predictions.
# It cycles through the following timers:
# - A countdown timer for when you are about to scan for an AprilTag 
# - A timer for scanning the AprilTag 
# - A countdown timer for when you are about to scan for afflictions
# - A timer for scanning for afflictions

# It publishing "Timer_state" ROS messages, which contain a timer's current status along with 
# the amount of time left, to:
# - "apriltag_countdown_timer_state"
# - "apriltag_scanning_timer_state" 
# - "prediction_countdown_timer_state"
# - "prediction_scanning_timer_state"
# 
# It triggers these timers by detecting if a trigger on a controller is not being pressed.
# It receives info about the triggers state from the "/button_status" topic.
#
# The length of the timers can be changed using the following constants:
#   - BUTTON_TIMER_LENGTH
#   - APRILTAG_TIMER_LENGTH
#   - PREDICTION_TIMER_LENGTH
#
# You will notice a few additional topics that this program publishes to. These are for use
# by the GUI to allow some of its elements to update. These topics include:
# - /current_timer # used to indicate which timer is active
# - /loop_state # used to determine which part of the pipeline loop we are in
#
# -------------------------------------------------------------------------------------------

# imports
import time
import rospy

# ROS messages
from messages.msg import Timer_state
from messages.msg import Command
from messages.msg import Current_timer
from messages.msg import LoopState


# ================================== CHANGE THESE IF NEEDED ===================================

BUTTON_TIMER_LENGTH     = 5
APRILTAG_TIMER_LENGTH   = 10
PREDICTION_TIMER_LENGTH = 20

# =============================================================================================

# general constants
TIMER_TICK  = 0.2

# timer types (enum)
APRILTAG_COUNTDOWN      = 0
APRILTAG_TIMER          = 1
PREDICTION_COUNTDOWN    = 2
PREDICTION_TIMER        = 3

# indexing directions (enum)
INDEX_BACKWARD  = 0
INDEX_FORWARD   = 1

# timer states (enum)
TIMER_ENDED      = 0
TIMER_STARTED    = 1
TIMER_CANCELLED  = 2

# global variables
button_pressed          = True
timer_ready             = False # used to check if the trigger has been pressed again before starting the next timer
not_pressed_counter     = 0
current_timer           = 0
previous_button_state   = True

# initializing node
rospy.init_node('prediction_timer_node', anonymous=True)

# creating ROS topic and publisher
apriltag_countdown_publisher    = rospy.Publisher('apriltag_countdown_timer_state', Timer_state, queue_size=10)
apriltag_timer_publisher        = rospy.Publisher('apriltag_scanning_timer_state', Timer_state, queue_size=10)
prediction_countdown_publisher  = rospy.Publisher('prediction_countdown_timer_state', Timer_state, queue_size=10)
prediction_timer_publisher      = rospy.Publisher('prediction_scanning_timer_state', Timer_state, queue_size=10)
current_timer_publisher         = rospy.Publisher('current_timer', Current_timer, queue_size=10)
loop_status_publisher           = rospy.Publisher('loop_state', LoopState, queue_size=10)

# used for printing formatted system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# moves to the next timer or the previous one depending on the user pressing the trigger on the controller
def index_timer(indexing_direction):
    global current_timer

    # indexing timer type
    if indexing_direction == 0:
        current_timer = (current_timer + 3) % 4
    else:
        current_timer = (current_timer + 1) % 4

# publishes the passed in "timer_status" to the current timer's ROS topic
def publish_timer_status(timer_status):
    global current_timer

    # publishing timer status message
    system_print("Publishing timer status message")
    if current_timer == APRILTAG_COUNTDOWN:
        apriltag_countdown_publisher.publish(timer_status)
    elif current_timer == APRILTAG_TIMER:
        apriltag_timer_publisher.publish(timer_status)
    elif current_timer == PREDICTION_COUNTDOWN:
        prediction_countdown_publisher.publish(timer_status)
    else:
        prediction_timer_publisher.publish(timer_status)

# starts a timer for the length of time passed in. Also publishes info about the program loop state for gui.py
def timer(timer_length)-> int:
    current_timer_tick = 0
    timer_status = Timer_state()
    timer_status.timer_status = TIMER_STARTED

    # This is used by the GUI
    if current_timer == APRILTAG_COUNTDOWN:
        loopState = LoopState()
        loopState.state = "Waiting to assign AprilTag"
        loop_status_publisher.publish(loopState)
    elif current_timer == APRILTAG_TIMER:
        loopState = LoopState()
        loopState.state = "Assigning AprilTag"
        loop_status_publisher.publish(loopState)
    elif current_timer == PREDICTION_COUNTDOWN:
        loopState = LoopState()
        loopState.state = "Waiting to scan for afflications"
        loop_status_publisher.publish(loopState)
    else:
        loopState = LoopState()
        loopState.state = "Scanning for afflications"
        loop_status_publisher.publish(loopState)

    # loops until timer is finished or canceled
    while True:
        # printing time left
        if (((current_timer_tick * TIMER_TICK) % 1) == 0):
            system_print("Time left: " + str(int(timer_length - (current_timer_tick * TIMER_TICK))))
            timer_status.time_left = int(timer_length - (current_timer_tick * TIMER_TICK))
            publish_timer_status(timer_status)

        # checking if timer is cancelled
        if button_pressed:
            system_print("Timer cancelled")
            timer_status.timer_status = TIMER_CANCELLED
            publish_timer_status(timer_status)
            return TIMER_CANCELLED
        
        # checking if the timer is finished
        if current_timer_tick == int(timer_length/TIMER_TICK):
            system_print("Timer ended")
            timer_status.timer_status = TIMER_ENDED
            publish_timer_status(timer_status)
            return TIMER_ENDED

        # incrementing timer tick
        current_timer_tick += 1
        time.sleep(TIMER_TICK)

# updates button_pressed based on the command received
def check_button(command):
    global button_pressed
    global button_pressed
    global timer_ready
    global previous_button_state

    button_state_changed = False

    # checking if the trigger is pressed
    if command.chan8 > 1600:
        button_pressed  = True
        timer_ready     = True
    else:
        button_pressed = False

    # checking if the state of the button changed and updating the previous state if it did
    if previous_button_state != button_pressed:
            previous_button_state = button_pressed
            button_state_changed = True

    # checking if the button state changed and informing user if it did
    if button_state_changed:
        if button_pressed:
            system_print("Trigger pressed")
        else:
            system_print("Trigger released")

# handles starting the timers when the button has not been pressed for BUTTON_TIMER_LENGTH seconds
def manage_timers():
    system_print("System ready")
    global button_pressed
    global not_pressed_counter
    global timer_ready

    # used by GUI
    current_timer_msg = Current_timer()
    current_timer_msg.current_timer = current_timer
    current_timer_publisher.publish(current_timer_msg)

    # used by GUI
    loopState = LoopState()
    loopState.state = "Waiting to assign AprilTag"
    loop_status_publisher.publish(loopState)
    
    while True:
        # checking if ctrl-c was entered
        if rospy.is_shutdown():
            exit(0)

        # used by GUI
        if current_timer == APRILTAG_COUNTDOWN:
            pass
        elif current_timer == APRILTAG_TIMER:
            loopState = LoopState()
            loopState.state = "Assigning AprilTag"
            loop_status_publisher.publish(loopState)
        elif current_timer == PREDICTION_COUNTDOWN:
            loopState = LoopState()
            loopState.state = "Waiting to scan for afflications"
            loop_status_publisher.publish(loopState)
        else:
            loopState = LoopState()
            loopState.state = "Scanning for afflications"
            loop_status_publisher.publish(loopState)
        
        # checking if the button is not being pressed
        if (not button_pressed) and timer_ready:
            # starting countdown timer 
            timer_state = timer(BUTTON_TIMER_LENGTH)

            # checking if the timer ended
            if timer_state == TIMER_ENDED:
                # indexing to the next timer
                index_timer(INDEX_FORWARD)

                # used by GUI
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
                    # indexing to the next timer
                    index_timer(INDEX_FORWARD)

                    # used by GUI
                    current_timer_msg.current_timer = current_timer
                    current_timer_publisher.publish(current_timer_msg)
                else:
                    # indexing to the previous timer
                    index_timer(INDEX_BACKWARD)

                    # used by GUI
                    current_timer_msg.current_timer = current_timer
                    current_timer_publisher.publish(current_timer_msg)

        # slowing loop down
        time.sleep(0.2)       

# "main function" of the program
if __name__ == "__main__":
    # creating ros subscriber to recieve trigger inputs
    rospy.Subscriber('/button_status', Command, check_button)

    # sleeping to allow gui.py to sync up
    time.sleep(0.5)

    # starting timer sequence
    manage_timers()
