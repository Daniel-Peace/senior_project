#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program handles the trigger on a controller being pressed or not. It can do this
# a physical controller or trhough the user entering a character. The mode it runs in
# is changed with the constant USE_CONTROLLER. As is implied with the name, if this
# constant is set to True, the program will look for a controller input from the
# "/joy" topic. If set to false, the use can enter the character 'T' to toggle the trigger
# back and forth from pressed to released. The state of the trigger is published to
# "/button_status" and is used by "timer.py"
# -------------------------------------------------------------------------------------------

import rospy
import time
from sensor_msgs.msg    import Joy
from messages.msg       import Command

# ================================== CHANGE THESE IF NEEDED ===================================

USE_CONTROLLER = True

# =============================================================================================

# global variables
locked          = False
has_released    = True

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# just helps reduce number of long print statements in code
def print_horizontal_line():
    print("---------------------------------------------------------------------")

# initializing node
rospy.init_node('button_press', anonymous=True)

# creating publisher
publisher = rospy.Publisher('button_status', Command, queue_size=10)

# creating command message
command = Command()
command.chan8 = 2000

# handles controller messages sent by the "joy" node
def handle_controller_inputs(msg:Joy):
    global locked
    global has_released

    # checking button state
    if msg.buttons[3] == 1 and has_released:
        if locked:
            locked = False
        else:
            locked = True

        has_released = False
        command.chan8 = 2000
    elif msg.buttons[3] == 0:
        has_released = True

    # checking if the button is not locked
    if not locked:    
        if msg.axes[5] == -1:
            command.chan8 = 2000
        elif msg.axes[5] == 1:
            command.chan8 = 0

    publisher.publish(command)

# checking if user wants to use the controller
if USE_CONTROLLER:
    # registering callback function
    rospy.Subscriber('joy', Joy, handle_controller_inputs)

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
else:
    while True:
        # prompting user
        print_horizontal_line()
        system_print(" Type \"t\" to toggle the button on or type q to quit:")
        while True:
            # getting user choice
            print_horizontal_line()
            choice = input("\u001b[34m -> \u001b[0m")

            # validating user input
            if choice.upper() == 'T':
                command.chan8 = 2000
                publisher.publish(command)
                break
            elif choice.upper() == 'Q':
                print_horizontal_line()
                system_print("Exiting...")
                print_horizontal_line()
                exit(0)
            else:
                print_horizontal_line()
                system_print("\u001b[31mInvalid choice...\u001b[0m")

        # prompting user
        print_horizontal_line
        system_print(" Type \"t\" to toggle the button off or type q to quit:")
        while True:
            # getting user choice
            print_horizontal_line
            choice = input("\u001b[34m -> \u001b[0m")

            # validating user input
            if choice.upper() == 'T':
                command.chan8 = 0
                publisher.publish(command)
                break
            elif choice.upper() == 'Q':
                print_horizontal_line()
                system_print("Exiting...")
                print_horizontal_line()
                exit(0)
            else:
                print_horizontal_line()
                system_print("\u001b[31mInvalid choice...\u001b[0m")
