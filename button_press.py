import rospy
import time

from messages.msg import Command

def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# initializing node
rospy.init_node('button_press', anonymous=True)

# creating publisher
publisher = rospy.Publisher('button_status', Command, queue_size=10)

# creating command message
command = Command()

while True:
    # prompting user
    print("---------------------------------------------------------------------")
    system_print(" Type \"t\" to toggle the button on or type q to quit:")
    while True:
        # getting user choice
        print("---------------------------------------------------------------------")
        choice = input("\u001b[34m -> \u001b[0m")

        # validating user input
        if choice.upper() == 'T':
            command.chan8 = 2000
            publisher.publish(command)
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
    system_print(" Type \"t\" to toggle the button off or type q to quit:")
    while True:
        # getting user choice
        print("---------------------------------------------------------------------")
        choice = input("\u001b[34m -> \u001b[0m")

        # validating user input
        if choice.upper() == 'T':
            command.chan8 = 0
            publisher.publish(command)
            break
        elif choice.upper() == 'Q':
            print("---------------------------------------------------------------------")
            system_print("Exiting...")
            print("---------------------------------------------------------------------")
            exit(0)
        else:
            print("---------------------------------------------------------------------")
            system_print("\u001b[31mInvalid choice...\u001b[0m")
