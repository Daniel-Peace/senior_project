#   +------------------------------+    +------------------------------+
#   | Models and their numbers     |    | Affliction types and values  |
#   +----------------------+-------+    +----------------------+-------+
#   | YoloV8               |   0   |    | SEVERE_HEMORRHAGE    |   0   |
#   +----------------------+-------+    +----------------------+-------+
#   | Auditory             |   1   |    | RESPIRATORY_DISTRESS |   1   |
#   +----------------------+-------+    +----------------------+-------+
#   | Radar                |   2   |    | HEART_RATE           |   2   |
#   +----------------------+-------+    +----------------------+-------+
#   |                      |   3   |    | RESPIRATORY_RATE     |   3   |
#   +----------------------+-------+    +----------------------+-------+
#   |                      |   4   |    | TRAUMA_HEAD          |   4   |
#   +----------------------+-------+    +----------------------+-------+
#   |                      |   5   |    | TRAUMA_TORSO         |   5   |
#   +----------------------+-------+    +----------------------+-------+
#   |                      |   6   |    | TRAUMA_LOWER_EXT     |   6   |
#   +----------------------+-------+    +----------------------+-------+
#                                       | TRAUMA_UPPER_EXT     |   7   |
#                                       +----------------------+-------+
#                                       | ALERTNESS_OCULAR     |   8   |
#                                       +----------------------+-------+
#                                       | ALERTNESS_VERBAL     |   9   |
#                                       +----------------------+-------+
#                                       | ALERTNESS_MOTOR      |   10  |
#                                       +----------------------+-------+

# imports
import rospy
import time
import json

# from casualty import Afflication_type as AT
from casualty import Casualty

from messages.msg import Assigned_apriltag, Critical_report
from messages.msg import Injury_report
from messages.msg import Prediction
from messages.msg import Vitals_report
from messages.msg import Timer_status

# constants
TEAM_NAME   = "coordinated robotics"
SYSTEM      = "JOE"

# stores the april tag for the current casualty
apriltag = -1

# trackers
received_april = False
model_0_has_not_predicted = True
model_1_has_not_predicted = True
model_2_has_not_predicted = True
model_3_has_not_predicted = True
model_4_has_not_predicted = True
model_5_has_not_predicted = True

# creating ROS topics
critical_report_pub   = rospy.Publisher('critical_report', Critical_report, queue_size=10)
vitals_report_pub     = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
injury_report_pub     = rospy.Publisher('injury_report', Injury_report, queue_size=10)

# initializing node
rospy.init_node('vote_and_create', anonymous=True)

# creating casualty objects for each model to hold predictions
model_0_casualty    = Casualty()   # yoloV8
model_1_casualty    = Casualty()   # auditory
model_2_casualty    = Casualty()   # radar
model_3_casualty    = Casualty()   # NONE
model_4_casualty    = Casualty()   # NONE
model_5_casualty    = Casualty()   # NONE
finalized_casualty  = Casualty()

# # publishes injury reports
# def publish_injury_reports():
#     injury_report = Injury_report()
#     injury_report.casualty_id   = finalized_casualty.apriltag
#     injury_report.team          = TEAM_NAME
#     injury_report.system        = SYSTEM
#     injury_report.type          = AT.TRAUMA_HEAD
#     injury_report.value         = finalized_casualty.trauma_head
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.TRAUMA_TORSO
#     injury_report.value         = finalized_casualty.trauma_torso
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.TRAUMA_LOWER_EXT
#     injury_report.value         = finalized_casualty.trauma_lower_ext
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.TRAUMA_UPPER_EXT
#     injury_report.value         = finalized_casualty.trauma_upper_ext
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.ALERTNESS_OCULAR
#     injury_report.value         = finalized_casualty.alertness_ocular
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.ALERTNESS_VERBAL
#     injury_report.value         = finalized_casualty.alertness_verbal
#     injury_report_pub.publish(injury_report)
#     injury_report.type          = AT.ALERTNESS_MOTOR
#     injury_report.value         = finalized_casualty.alertness_motor
#     injury_report_pub.publish(injury_report)

# # publishes vitals reports
# def publish_vitals_reports():
#     vitals_report = Vitals_report()
#     vitals_report.casualty_id   = finalized_casualty.apriltag
#     vitals_report.team          = TEAM_NAME
#     vitals_report.system        = SYSTEM
#     vitals_report.type          = AT.HEART_RATE
#     vitals_report.value         = finalized_casualty.heart_rate
#     vitals_report_pub.publish(vitals_report)
#     vitals_report.type          = AT.RESPIRATORY_RATE
#     vitals_report.value         = finalized_casualty.respiratory_rate
#     vitals_report_pub.publish(vitals_report)

# # publishes critical reports
# def publish_critical_reports():
#     critical_report = Critical_report()
#     critical_report.casualty_id = TEAM_NAME
#     critical_report.system      = SYSTEM
#     critical_report.type        = AT.SEVERE_HEMORRHAGE
#     critical_report.value       = finalized_casualty.severe_hemorrhage
#     critical_report_pub.publish(critical_report)
#     critical_report.type        = AT.RESPIRATORY_DISTRESS
#     critical_report.value       = finalized_casualty.respiratory_distress

# loops until all needed predictions have been received
def wait_for_predictions():
    while model_0_has_not_predicted:
        continue

# publishes reports to respective topics
def publish_reports():
    finalized_casualty.publish_reports()
    # publish_critical_reports()
    # publish_vitals_reports()
    # publish_injury_reports()

# handles combining and finalizing reports
def finalize_afflication_values():
    wait_for_predictions()

def wait_for_apriltag():
    while not received_april:
        continue

# resets tracking variables
def reset_trackers():
    received_april              = False
    model_0_has_not_predicted   = True
    model_1_has_not_predicted   = True
    model_2_has_not_predicted   = True
    model_3_has_not_predicted   = True
    model_4_has_not_predicted   = True
    model_5_has_not_predicted   = True

# resets apriltag value
def reset_apriltag():
    apriltag = -1

# this functions resets all of the afflication values to -1
def reset_casualty_objects():
    model_0_casualty.reset()
    model_1_casualty.reset()
    model_2_casualty.reset()
    model_3_casualty.reset()
    model_4_casualty.reset()
    model_5_casualty.reset()
    finalized_casualty.reset()

# sets the AprilTag of the current casualty
def set_apriltag():
    wait_for_apriltag()
    finalized_casualty.apriltag = apriltag

# calls all reset functions
def reset():
    reset_casualty_objects()
    reset_apriltag()
    reset_trackers()

# handles actions that need to take place when a timer finishes
def on_timer_finish():
    finalize_afflication_values()
    publish_reports()

# handles actions that need to take place when a timer starts
def on_timer_start():
    reset()
    set_apriltag()

# handles actions corresponding to the timer starting and stopping
def handle_timer_status(msg):
    if msg.timer_status == True:
        on_timer_start()
    else:
        on_timer_finish()

# gets the assigned AprilTag and sets it
def assign_apriltag(msg):
    apriltag = msg.apriltag

def receive_model_0_predictions(predictions):
    for prediction in predictions:
        if prediction.tyoe == Casualty.SEVERE_HEMORRHAGE:
            model_0_casualty.severe_hemorrhage = prediction.value
        elif prediction.type == Casualty.TRAUMA_HEAD:
            model_0_casualty.trauma_head = prediction.value
        elif prediction.type == Casualty.TRAUMA_TORSO:
            model_0_casualty.trauma_torso = prediction.value
        elif prediction.type == Casualty.TRAUMA_LOWER_EXT:
            model_0_casualty.trauma_lower_ext = prediction.value
        elif prediction.type == Casualty.TRAUMA_UPPER_EXT:
            model_0_casualty.trauma_upper_ext = prediction.value

if __name__ == "__main__":
    # registering callback functions
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_timer_status)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, assign_apriltag)
    rospy.Subscriber('model_0_predictions', Prediction, receive_model_0_predictions)

# loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
