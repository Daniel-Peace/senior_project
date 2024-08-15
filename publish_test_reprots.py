#!/usr/bin/env python3

# imports
import rospy
from messages.msg import Critical_report
from messages.msg import Injury_report
from messages.msg import Vitals_report

# constants
TEAM_NAME = "coordinated robotics"

# initializing publishers
critical_report_pub   = rospy.Publisher('critical_report', Critical_report, queue_size=10)
vitals_report_pub     = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
injury_report_pub     = rospy.Publisher('injury_report', Injury_report, queue_size=10)

def print_bar():
    print("------------------------------------------------------------------------------")

# send a test critical report
def send_critical_report():
    print_bar()

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

    print("Published the following report:\n")
    print(critical_report)

    print_bar()

def send_injury_report():
    print_bar()

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

    print("Published the following report:\n")
    print(injury_report)

    print_bar()

def send_vitals_report():
    print_bar()

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

    print("Published the following report:\n")
    print(vitals_report)

    print_bar()

def test():
    rospy.init_node('test_node', anonymous=True)
    while not rospy.is_shutdown():
        # prompting user
        print("Choose which report you would like to send:")
        print("\ta. Critical Report")
        print("\tb. Inury Report")
        print("\tc. Vitals Report")
        while True:
            # prompting user
            choice = input("\u001b[34m-> \u001b[0m")

            if choice == 'a' or choice == 'A':
                send_critical_report()
            elif choice == 'b' or choice == 'B':
                send_injury_report()
            elif choice == 'c' or choice == 'C':
                send_vitals_report()
            elif choice == 'q' or choice == 'Q':
                print("Exiting...")
                break
            else:
                print("\u001b[34m-> \u001b[0m \u001b[31mInvalid choice...\u001b[0m")

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass