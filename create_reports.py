#!/usr/bin/env python3

#   +------------------------------+
#   | reference for yolo classes   |
#   +----------------------+-------+
#   | LOWER_EXT_WOUND      |   0   |
#   +----------------------+-------+
#   | LOWER_EXT_AMPUTATION |   1   |
#   +----------------------+-------+
#   | SEVERE_HEMORRHAGE    |   2   |
#   +----------------------+-------+
#   | TORSO_WOUND          |   3   |
#   +----------------------+-------+
#   | HEAD_WOUND           |   4   |
#   +----------------------+-------+
#   | UPPER_EXT_WOUND      |   5   |
#   +----------------------+-------+

# imports
import rospy
import time
from enum import Enum
from messages.msg import Critical_report
from messages.msg import Injury_report
from messages.msg import Prediction
from messages.msg import Vitals_report

# constants
TEAM_NAME = "coordinated robotics"

# creating ROS topics
critical_report_pub   = rospy.Publisher('critical_report', Critical_report, queue_size=10)
vitals_report_pub     = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
injury_report_pub     = rospy.Publisher('injury_report', Injury_report, queue_size=10)

# this function handles the yolo predictions
def handle_yolo_prediction(message):
    for prediction in message.prediction_elements:
        print(prediction.injury_class)
        print(prediction.confidence)

        print("---------------------------------------------------------------")
        if prediction.injury_class == 2:
            critical_report = Critical_report()
            critical_report.casualty_id = 0
            critical_report.team        = TEAM_NAME
            critical_report.system      = "test_system"
            critical_report.type        = "severe_hemorrhage"
            critical_report.value       = 1

            # publishing report
            critical_report_pub.publish(critical_report)
            print(critical_report)

        else:
            injury_report   = Injury_report()
            if prediction.injury_class == 0:
                injury_report.casualty_id = 0
                injury_report.team        = TEAM_NAME
                injury_report.system      = "test_system"
                injury_report.type        = "trauma_lower_ext"
                injury_report.value       = 1
  
            elif prediction.injury_class == 1:
                injury_report.casualty_id = 0
                injury_report.team        = TEAM_NAME
                injury_report.system      = "test_system"
                injury_report.type        = "trauma_lower_ext"
                injury_report.value       = 2

            elif prediction.injury_class == 3:
                injury_report.casualty_id = 0
                injury_report.team        = TEAM_NAME
                injury_report.system      = "test_system"
                injury_report.type        = "trauma_torso"
                injury_report.value       = 1

            elif prediction.injury_class == 5:
                injury_report.casualty_id = 0
                injury_report.team        = TEAM_NAME
                injury_report.system      = "test_system"
                injury_report.type        = "trauma_upper_ext"
                injury_report.value       = 1

            # publishing report
            injury_report_pub.publish(injury_report)
            print(injury_report)

# core loop that creates reports based on predictions from AI/ML models
def create_reports():
    # creating ROS node
    rospy.init_node('yoloV8_node', anonymous=True)

    # registering callback functions
    rospy.Subscriber('yoloV8_prediction', Prediction, handle_yolo_prediction)

    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)


if __name__ == "__main__":
    create_reports()