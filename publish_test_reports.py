#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program simply publishes test reports should you want to test "send_reports.py"
# It allows the user to pick one of three reports to publish. The report is published
# to it's corresponding topic. This would be one of the following:
# - /critical_report
# - /vitals_report
# - /injury_report
# -------------------------------------------------------------------------------------------

# imports
import rospy
from messages.msg import Critical_report
from messages.msg import Injury_report
from messages.msg import Vitals_report

# general constants
TEAM_NAME = "coordinated robotics"

# initializing ros node
rospy.init_node('test_reports', anonymous=True)

# initializing publishers
critical_report_pub   = rospy.Publisher('critical_report', Critical_report, queue_size=10)
vitals_report_pub     = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
injury_report_pub     = rospy.Publisher('injury_report', Injury_report, queue_size=10)

# used for printing formatted system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# just helps reduce number of long print statements in code
def print_horizontal_line():
    print("---------------------------------------------------------------------")

# send a test critical report
def send_critical_report():
    print_horizontal_line()

    # creating test report
    critical_report = Critical_report()

    # initializing report
    critical_report.casualty_id = 5
    critical_report.team        = TEAM_NAME
    critical_report.system      = "Joe"
    critical_report.type        = "severe_hemorrhage"
    critical_report.value       = 1

    # publishing report
    critical_report_pub.publish(critical_report)

    # printing the report that was publihsed for reference
    system_print("Published the following report:")
    print_horizontal_line()
    print(critical_report)
    print_horizontal_line()

# sends a test injury report
def send_injury_report():
    print_horizontal_line()

    # creating test report
    injury_report = Injury_report()

    # initializing report
    injury_report.casualty_id = 3
    injury_report.team        = TEAM_NAME
    injury_report.system      = "Pam"
    injury_report.type        = "trauma_head"
    injury_report.value       = 0

    # publishing report
    injury_report_pub.publish(injury_report)

    # printing the report that was publihsed for reference
    system_print("Published the following report:")
    print_horizontal_line()
    print(injury_report)
    print_horizontal_line()

# sends a test vitals report
def send_vitals_report():
    print_horizontal_line()

    # creating test report
    vitals_report = Vitals_report()

    # initializing report
    vitals_report.casualty_id = 3
    vitals_report.team        = TEAM_NAME
    vitals_report.system      = "Bullwinkle"
    vitals_report.type        = "hr"
    vitals_report.value       = 60
    vitals_report.time_ago    = 14

    # publishing report
    vitals_report_pub.publish(vitals_report)

    # printing the report that was publihsed for reference
    system_print("Published the following report:")
    print_horizontal_line
    print(vitals_report)
    print_horizontal_line()

# function to call other functions based on user_input
def manage_publishers():
    # looping until user quits
    while not rospy.is_shutdown():
        # prompting user
        print_horizontal_line()
        system_print("Choose which report you would like to send:")
        print("\ta. Critical Report")
        print("\tb. Inury Report")
        print("\tc. Vitals Report")
        print_horizontal_line()
        while True:
            # prompting user
            choice = input("\u001b[34m-> \u001b[0m")

            # checking user choice
            if choice == 'a' or choice == 'A':
                send_critical_report()
            elif choice == 'b' or choice == 'B':
                send_injury_report()
            elif choice == 'c' or choice == 'C':
                send_vitals_report()
            elif choice == 'q' or choice == 'Q':
                print_horizontal_line()
                system_print("Exiting...")
                print_horizontal_line()
                exit(0)
            else:
                print_horizontal_line()
                system_print("Invalid choice...")
                print_horizontal_line()

# "main function" of the program
if __name__ == '__main__':
    manage_publishers()