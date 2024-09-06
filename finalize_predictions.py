#   +------------------------------+    +------------------------------+
#   | Models and their numbers     |    | Affliction types and values  |
#   +----------------------+-------+    +----------------------+-------+
#   | YoloV8               |   0   |    | SEVERE_HEMORRHAGE    |   0   |
#   +----------------------+-------+    +----------------------+-------+
#   | Auditory             |   1   |    | RESPIRATORY_DISTRESS |   1   |
#   +----------------------+-------+    +----------------------+-------+
#   | Radar                |   2   |    | HEART_RATE           |   2   |
#   +----------------------+-------+    +----------------------+-------+
#   | Face Mesh/Joints     |   3   |    | RESPIRATORY_RATE     |   3   |
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
from casualty import Casualty
from messages.msg import Assigned_apriltag, Critical_report
from messages.msg import Injury_report
from messages.msg import Vitals_report
from messages.msg import Timer_status
from messages.msg import Casualty_prediction

# general constants
TEAM_NAME       = "coordinated robotics"
SYSTEM          = "JOE"
NUM_OF_MODELS   = 6
DEFAULT_HR      = 100
DEFAULT_RR      = 25
TIMEOUT         = 2

# model weights
MODEL_0_WEIGHT = 1
MODEL_1_WEIGHT = 2
MODEL_2_WEIGHT = 1
MODEL_3_WEIGHT = 1
MODEL_4_WEIGHT = 1
MODEL_5_WEIGHT = 1

# model indexes
MODEL_0_INDEX = 0
MODEL_1_INDEX = 1
MODEL_2_INDEX = 2
MODEL_3_INDEX = 3
MODEL_4_INDEX = 4
MODEL_5_INDEX = 5

# stores the april tag for the current casualty
apriltag = -1

# tracks if an april tag has been assigned to the current casualty
received_april = False

# holds tracking values for if a model has published a submission yet
model_trackers = []
for i in range(NUM_OF_MODELS):
    model_trackers.append(False)

# creating casualty objects for each model to hold predictions
model_predictions = []
for i in range(NUM_OF_MODELS):
        model_predictions.append(Casualty())
finalized_casualty  = Casualty()

# holds weights for voting
model_weights = []
weight = MODEL_0_WEIGHT
model_weights.append(weight)
weight = MODEL_1_WEIGHT
model_weights.append(weight)
weight = MODEL_2_WEIGHT
model_weights.append(weight)
weight = MODEL_3_WEIGHT
model_weights.append(weight)
weight = MODEL_4_WEIGHT
model_weights.append(weight)
weight = MODEL_5_WEIGHT
model_weights.append(weight)

# creating ROS topics
critical_report_pub   = rospy.Publisher('critical_report', Critical_report, queue_size=10)
vitals_report_pub     = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
injury_report_pub     = rospy.Publisher('injury_report', Injury_report, queue_size=10)

# initializing node
rospy.init_node('vote_and_create', anonymous=True)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# resets the weight array to the default weights
def reset_weight_array():
    model_weights[MODEL_0_INDEX] = MODEL_0_WEIGHT
    model_weights[MODEL_1_INDEX] = MODEL_1_WEIGHT
    model_weights[MODEL_2_INDEX] = MODEL_2_WEIGHT
    model_weights[MODEL_3_INDEX] = MODEL_3_WEIGHT
    model_weights[MODEL_4_INDEX] = MODEL_4_WEIGHT
    model_weights[MODEL_5_INDEX] = MODEL_5_WEIGHT


# loops until all needed predictions have been received
def wait_for_predictions():
    received_all_predictions = False
    i = 0
    while not received_all_predictions:
        if i == 0:
            system_print("Waiting for predictions...")
        system_print("Models' prediction status:")
        print("------------------------------------------------------")
        received_all_predictions = True
        for index, tracker in enumerate(model_trackers, 0):
            if not tracker:
                received_all_predictions = False

            print("model " + str(index) + " has predicted:\t" + str(tracker))
        print("------------------------------------------------------")
        if i == int(TIMEOUT/0.2):
            break
        time.sleep(0.2)
        i += 1

    if not received_all_predictions:
        system_print("some models timed out")
    else:
        system_print("Received all needed predictions")


# publishes reports to respective topics
def publish_reports():
    system_print("Publishing reports")
    print("------------------------------------------------------")
    finalized_casualty.publish_reports()

# handles combining and finalizing reports
def finalize_afflication_values():
    system_print("Finalizing predictions")
    if model_predictions[MODEL_2_INDEX].time_ago >= 0:
        finalized_casualty.time_ago = model_predictions[MODEL_2_INDEX].time_ago
    else:
        finalized_casualty.time_ago = 0

    # checking if current casualty is coherent
    system_print("Checking if casualty is coherent...")
    if not model_predictions[MODEL_1_INDEX].is_coherent:
        system_print("Casualty is incoherent")
        model_weights[MODEL_1_INDEX] = 0
    else:
        system_print("Casualty is coherent")

    # adding up votes for severe_hemorrhage
    system_print("Finalizing severe hemorrhage prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.severe_hemorrhage == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.severe_hemorrhage == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.severe_hemorrhage = 1
    else:
        finalized_casualty.severe_hemorrhage = 0

    system_print("Finalizing respiratory distress prediction")
    # adding up votes for respiratory_distress
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.respiratory_distress == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.respiratory_distress == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.respiratory_distress = 1
    else:
        finalized_casualty.respiratory_distress = 0

    # averaging heart rate values
    system_print("Finalizing heart rate prediction")
    average_hr          = 0
    num_of_predictions  = 0
    has_changed         = False
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.heart_rate > 0:
            average_hr += model_prediction.heart_rate
            num_of_predictions += 1
            has_changed = True
    if has_changed:
        finalized_casualty.heart_rate = int(average_hr / num_of_predictions - 0.5 + 1)
    else:
        finalized_casualty.heart_rate = DEFAULT_HR

    # averaging respiratory rate values
    system_print("Finalizing respiratory rate prediction")
    average_rr          = 0
    num_of_predictions  = 0
    has_changed         = False
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.respiratory_rate > 0:
            average_rr += model_prediction.respiratory_rate
            num_of_predictions += 1
            has_changed = True
    if has_changed:
        finalized_casualty.respiratory_rate = int(average_rr / num_of_predictions - 0.5 + 1)
    else:
        finalized_casualty.respiratory_rate = DEFAULT_RR

    # adding up votes for trauma_head
    system_print("Finalizing trauma head prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.trauma_head == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.trauma_head == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.trauma_head = 1
    elif affliction_vote < 0:
        finalized_casualty.trauma_head = 0
    else:
        # TODO determine what guess should be
        pass

    # adding up votes for trauma_torso
    system_print("Finalizing trauma torso prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.trauma_torso == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.trauma_torso == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.trauma_torso = 1
    else:
        finalized_casualty.trauma_torso = 0

    # adding up votes for trauma_lower_ext
    system_print("Finalizing trauma lower ext. prediction")
    lower_injury_vote       = 0
    lower_amputation_vote   = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.trauma_lower_ext == 0:
            lower_injury_vote -= model_weights[index]
            lower_amputation_vote -= model_weights[index]
        elif model_prediction.trauma_lower_ext == 2:
            lower_amputation_vote += model_weights[index]
            lower_injury_vote += float(model_weights[index]/2)
        elif model_prediction.trauma_lower_ext == 1:
            lower_injury_vote += model_weights[index]

    if (lower_amputation_vote > 0) and (lower_amputation_vote > lower_injury_vote):
        finalized_casualty.trauma_lower_ext = 2
    elif (lower_injury_vote > 0) and (lower_amputation_vote <= lower_injury_vote):
        finalized_casualty.trauma_lower_ext = 1
    else:
        finalized_casualty.trauma_lower_ext = 0

    # adding up votes for trauma_upper_ext
    system_print("Finalizing trauma upper ext. prediction")
    upper_injury_vote       = 0
    upper_amputation_vote   = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.trauma_upper_ext == 0:
            upper_injury_vote -= model_weights[index]
            upper_amputation_vote -= model_weights[index]
        elif model_prediction.trauma_upper_ext == 2:
            upper_injury_vote += float(model_weights[index]/2)
            upper_amputation_vote += model_weights[index]
        elif model_prediction.trauma_upper_ext == 1:
            upper_injury_vote += model_weights[index]

    if (upper_amputation_vote > 0) and (upper_amputation_vote > upper_injury_vote):
        finalized_casualty.trauma_upper_ext = 2
    elif (upper_injury_vote > 0) and (upper_amputation_vote <= upper_injury_vote):
        finalized_casualty.trauma_upper_ext = 1
    else:
        finalized_casualty.trauma_upper_ext = 0


    # adding up votes for trauma_head
    system_print("Finalizing alertness ocular prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.alertness_ocular == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.alertness_ocular == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.alertness_ocular = 1
    else:
        finalized_casualty.alertness_ocular = 0

    # adding up votes for trauma_head
    system_print("Finalizing alertness verbal prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.alertness_verbal == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.alertness_verbal == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.alertness_verbal = 1
    else:
        finalized_casualty.alertness_verbal = 0

    # adding up votes for trauma_head
    system_print("Finalizing alertness motor prediction")
    affliction_vote = 0
    for index, model_prediction in enumerate(model_predictions, 0):
        if model_prediction.alertness_motor == 0:
            affliction_vote -= model_weights[index]
        elif model_prediction.alertness_motor == 1:
            affliction_vote += model_weights[index]

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.alertness_motor = 1
    else:
        finalized_casualty.alertness_motor = 0

# waits for specified models to finish predictions
def wait_for_apriltag():
    i = 0
    while not received_april:
        if i == 0:
            system_print("Waiting for AprilTag...")
        time.sleep(0.5)
        i += 1
        continue

# resets tracking variables
def reset_trackers():
    system_print("Reseting trackers")
    global received_april
    received_april              = False

    for i in range(6):
        model_trackers[i] = False

# resets apriltag value
def reset_apriltag():
    system_print("Reseting AprilTag")
    global apriltag
    apriltag = -1

# this functions resets all of the afflication values to -1
def reset_casualty_objects():
    system_print("Reseting casuatly objects")
    for model_prediction in model_predictions:
        model_prediction.reset()
    finalized_casualty.reset()

# sets the AprilTag of the current casualty
def set_apriltag():
    system_print("Setting AprilTag")
    finalized_casualty.apriltag = apriltag

# calls all reset functions
def reset():
    reset_casualty_objects()
    reset_apriltag()
    reset_trackers()

# handles actions that need to take place when a timer finishes
def on_timer_finish():
    system_print("Timer has ended")
    wait_for_predictions()
    wait_for_apriltag()
    set_apriltag()
    finalize_afflication_values()
    publish_reports()

# handles actions that need to take place when a timer starts
def on_timer_start():
    system_print("Timer has started")
    reset()

# handles actions corresponding to the timer starting and stopping
def handle_timer_status(msg):
    if msg.timer_status == True:
        on_timer_start()
    else:
        on_timer_finish()

# gets the assigned AprilTag and sets it
def assign_apriltag(msg):
    system_print("Received AprilTag")
    global apriltag
    apriltag = msg.apriltag
    global received_april
    received_april = True
    system_print("Set AprilTag")

# callback function for when predictions are received from model 0
def receive_model_predictions(casualty_ros):
    model_predictions[casualty_ros.model].severe_hemorrhage      = casualty_ros.severe_hemorrhage
    model_predictions[casualty_ros.model].respiratory_distress   = casualty_ros.respiratory_distress
    model_predictions[casualty_ros.model].heart_rate             = casualty_ros.heart_rate
    model_predictions[casualty_ros.model].respiratory_rate       = casualty_ros.respiratory_rate
    model_predictions[casualty_ros.model].trauma_head            = casualty_ros.trauma_head
    model_predictions[casualty_ros.model].trauma_torso           = casualty_ros.trauma_torso
    model_predictions[casualty_ros.model].trauma_lower_ext       = casualty_ros.trauma_lower_ext
    model_predictions[casualty_ros.model].trauma_upper_ext       = casualty_ros.trauma_upper_ext
    model_predictions[casualty_ros.model].alertness_ocular       = casualty_ros.alertness_ocular
    model_predictions[casualty_ros.model].alertness_verbal       = casualty_ros.alertness_verbal
    model_predictions[casualty_ros.model].alertness_motor        = casualty_ros.alertness_motor
    model_trackers[casualty_ros.model] = True

    system_print("Received prediction from model " + str(casualty_ros.model))
    print("------------------------------------------------------")
    print(casualty_ros)
    print("------------------------------------------------------")
    system_print("Updated casualty object for model " + str(casualty_ros.model))
    print("------------------------------------------------------")
    model_predictions[casualty_ros.model].print_self()
    print("------------------------------------------------------")


if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_timer_status)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, assign_apriltag)
    rospy.Subscriber('model_predictions', Casualty_prediction, receive_model_predictions)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
