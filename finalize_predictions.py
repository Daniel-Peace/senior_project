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

# loops until all needed predictions have been received
def wait_for_predictions():
    print("system: waiting for predictions...")
    while model_0_has_not_predicted:
        time.sleep(0.5)
        continue
    print("system: received model 0 prediction")

# publishes reports to respective topics
def publish_reports():
    finalized_casualty.publish_reports()

# handles combining and finalizing reports
def finalize_afflication_values():
    set_apriltag()
    wait_for_predictions()
    finalized_casualty.severe_hemorrhage = model_0_casualty.severe_hemorrhage
    finalized_casualty.trauma_head = model_0_casualty.trauma_head
    finalized_casualty.trauma_torso = model_0_casualty.trauma_torso
    finalized_casualty.trauma_lower_ext = model_0_casualty.trauma_lower_ext
    finalized_casualty.trauma_upper_ext = model_0_casualty.trauma_upper_ext

# waits for specified models to finish predictions
def wait_for_apriltag():
    print("system: waiting for AprilTag...")
    while not received_april:
        time.sleep(0.5)
        continue

# resets tracking variables
def reset_trackers():
    print("system: reseting trackers")
    global received_april
    global model_0_has_not_predicted
    global model_1_has_not_predicted
    global model_2_has_not_predicted
    global model_3_has_not_predicted
    global model_4_has_not_predicted
    global model_5_has_not_predicted

    received_april              = False
    model_0_has_not_predicted   = True
    model_1_has_not_predicted   = True
    model_2_has_not_predicted   = True
    model_3_has_not_predicted   = True
    model_4_has_not_predicted   = True
    model_5_has_not_predicted   = True

# resets apriltag value
def reset_apriltag():
    print("system: reseting AprilTag")
    apriltag = -1

# this functions resets all of the afflication values to -1
def reset_casualty_objects():
    print("system: reseting casuatly objects")
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
    print("system: setting AprilTag")
    finalized_casualty.apriltag = apriltag

# calls all reset functions
def reset():
    reset_casualty_objects()
    reset_apriltag()
    reset_trackers()

# handles actions that need to take place when a timer finishes
def on_timer_finish():
    print("system: timer has ended...")
    finalize_afflication_values()
    publish_reports()

# handles actions that need to take place when a timer starts
def on_timer_start():
    print("system: timer has started...")
    reset()

# handles actions corresponding to the timer starting and stopping
def handle_timer_status(msg):
    if msg.timer_status == True:
        on_timer_start()
    else:
        on_timer_finish()

# gets the assigned AprilTag and sets it
def assign_apriltag(msg):
    print("system: received AprilTag")
    global apriltag
    apriltag = msg.apriltag
    global received_april
    received_april = True
    print("system: set AprilTag")

# callback function for when predictions are received from model 0
def receive_model_0_predictions(predictions):
    for prediction in predictions.prediction_elements:
        if prediction.type == Casualty.SEVERE_HEMORRHAGE:
            model_0_casualty.severe_hemorrhage = prediction.value
        elif prediction.type == Casualty.TRAUMA_HEAD:
            model_0_casualty.trauma_head = prediction.value
        elif prediction.type == Casualty.TRAUMA_TORSO:
            model_0_casualty.trauma_torso = prediction.value
        elif prediction.type == Casualty.TRAUMA_LOWER_EXT:
            model_0_casualty.trauma_lower_ext = prediction.value
        elif prediction.type == Casualty.TRAUMA_UPPER_EXT:
            model_0_casualty.trauma_upper_ext = prediction.value
    model_0_casualty.print_self()
    global model_0_has_not_predicted
    model_0_has_not_predicted = False

if __name__ == "__main__":
    # registering callback functions
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_timer_status)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, assign_apriltag)
    rospy.Subscriber('model_0_predictions', Prediction, receive_model_0_predictions)

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
