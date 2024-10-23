# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program is responsible for receiving all predictions from all models and creating
# a finalized report about a casualty. It also receives the assigned AprilTag and
# adds this to the final report. To accomplish this the program waits for a prediction
# timer to start. Once a timer has started the program resets all variables and objects
# in preparation for receiving predictions and the assigned AprilTag. Anytime during the
# timer, the program can receive predictions and update each models prediction object.
# Once the timer has ended. The model weights 2 seconds both for final predictions to come
# in, and for an apriltag to be assigned. After this, it and moves on to
# finalizing all of the predictions. Upon completing this process, it calls the
# publish_reports method on finalized_casualty which creates reports for each afflication
# type and publishes them to their respective topics.
#
# During the finalzation process this program assumes that if a member of a casualty
# object is -1, then the model represented by that object did not predict for that
# affliction type.
#
# For all alertness affliction types there is currently only one model for each
# type. This program therefore assigns those models predictions directly to the final
# report rather than using a voting system of anykind.
#
# For affliction categories using a voting system, the weights assigned to each model
# can be adjusted using the weight constants towards the top of the program
#
# If any of the afflication categories fails to find a model that made a prediction for
# said category, the model assigns whichever value would be considered "normal"
# by default. For both heart rate and respiratory rate, these values can be modified using
# the DEFAULT_HR and DEFAULT_RR constants.
#
# The timout timers length can be adjusted using the PREDICTION_TIMEOUT and APRILTAG_TIMEOUT
# constants.
#
# In the event that an AprilTag is not received, it will assign -1 to the report and leave
# it to "send_report.py" to handle.
#
# Some functions and parts of the initialization process require knowledge of the number
# of models being linked. The NUM_OF_MODELS should be set accordingly. You should also
# add more weight constants, index constants, and modify the initialization of the weights
# array in the case of adding more models. (This may be improved in the future)
# -------------------------------------------------------------------------------------------

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
#   | Depth Camera         |   4   |    | TRAUMA_HEAD          |   4   |
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
import time
import rospy
from casualty import Casualty

# ROS messages
from messages.msg import Assigned_apriltag
from messages.msg import Casualty_prediction
from messages.msg import Critical_report
from messages.msg import Injury_report
from messages.msg import Timer_status
from messages.msg import Vitals_report
from messages.msg import ModelPredictionStatus
from messages.msg import ModelPredictionStatuses

# timer states
TIMER_ENDED     = 0
TIMER_STARTED   = 1
TIMER_CANCELLED  = 2

# general constants
NUM_OF_MODELS       = 6
DEFAULT_HR          = 100
DEFAULT_RR          = 25
PREDICTION_TIMEOUT  = 2
APRILTAG_TIMEOUT    = 2

# model weights
MODEL_0_WEIGHT = 1
MODEL_1_WEIGHT = 3
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

# tracks the previous state of the timer
previousPredictionTimerState    = -1
previousApriltagTimerState      = -1

# holds tracking values for if a model has published a submission yet
prediction_received = []
for index in range(NUM_OF_MODELS):
    prediction_received.append(False)

# creating casualty objects for each model to hold predictions
model_predictions = []
for index in range(NUM_OF_MODELS):
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

# initializing node
rospy.init_node('vote_and_create', anonymous=True)

final_report_publisher = rospy.Publisher('final_report', Casualty_prediction, queue_size=10)
model_prediction_statuses_publisher = rospy.Publisher('model_prediction_statuses', ModelPredictionStatuses, queue_size=10)

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
    timeout_tracker = 0

    # waiting until all predictions have been received or timeout occurs
    while not received_all_predictions:
        model_prediction_statuses = ModelPredictionStatuses()
        if timeout_tracker == 0:
            system_print("Waiting for predictions...")

        # checking which models have published predictions
        received_all_predictions = True
        for index, has_predicted in enumerate(prediction_received):
            model_prediction_status = ModelPredictionStatus()
            model_prediction_status.model_number = index
            model_prediction_status.made_prediction = has_predicted
            model_prediction_statuses.modelPredictionStatuses.append(model_prediction_status)
            if not has_predicted:
                received_all_predictions = False

        model_prediction_statuses_publisher.publish(model_prediction_statuses)

        # checking for timeout
        if timeout_tracker == int(PREDICTION_TIMEOUT/0.2):
            break
        time.sleep(0.2)
        timeout_tracker += 1

    # printing status of which models predicted
    if not received_all_predictions:
        system_print("some models timed out")
    else:
        system_print("Received all needed predictions")
    system_print("Models' prediction status:")
    print("------------------------------------------------------")
    for index, has_predicted in enumerate(prediction_received, 0):
        print("model " + str(index) + " has predicted:\t" + str(has_predicted))
    print("------------------------------------------------------")


# publishes reports to respective topics
def publish_reports():
    system_print("Publishing reports")
    print("------------------------------------------------------")
    finalized_casualty.publish_reports()
    final_report = Casualty_prediction()
    final_report.apriltag               = apriltag
    final_report.is_coherent            = finalized_casualty.is_coherent
    final_report.time_ago               = finalized_casualty.time_ago
    final_report.severe_hemorrhage      = finalized_casualty.severe_hemorrhage
    final_report.respiratory_distress   = finalized_casualty.respiratory_distress
    final_report.heart_rate             = finalized_casualty.heart_rate
    final_report.respiratory_rate       = finalized_casualty.respiratory_rate
    final_report.trauma_head            = finalized_casualty.trauma_head
    final_report.trauma_torso           = finalized_casualty.trauma_torso
    final_report.trauma_lower_ext       = finalized_casualty.trauma_lower_ext
    final_report.trauma_upper_ext       = finalized_casualty.trauma_upper_ext
    final_report.alertness_ocular       = finalized_casualty.alertness_ocular
    final_report.alertness_verbal       = finalized_casualty.alertness_verbal
    final_report.alertness_motor        = finalized_casualty.alertness_motor
    final_report_publisher.publish(final_report)

# handles combining and finalizing reports
def finalize_afflication_values():
    system_print("Finalizing predictions")

    # getting time_ago value
    if model_predictions[MODEL_2_INDEX].time_ago >= 0:
        finalized_casualty.time_ago = model_predictions[MODEL_2_INDEX].time_ago
    elif model_predictions[MODEL_4_INDEX].time_ago >= 0:
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

    # adding up votes for respiratory_distress
    system_print("Finalizing respiratory distress prediction")
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
        if model_prediction.heart_rate >= 0:
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
        if model_prediction.respiratory_rate >= 0:
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
    else:
        finalized_casualty.trauma_head = 0

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
            lower_injury_vote += float(model_weights[index]/2)
            lower_amputation_vote += model_weights[index]
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

    # finalizing alertness ocular
    system_print("Finalizing alertness ocular prediction")
    if model_predictions[MODEL_3_INDEX].alertness_ocular >= 0:
        finalized_casualty.alertness_ocular = model_predictions[MODEL_3_INDEX].alertness_ocular
    else:
        finalized_casualty.alertness_ocular = 0

    # finalizing alertness verbal
    system_print("Finalizing alertness verbal prediction")
    if model_predictions[MODEL_1_INDEX].alertness_verbal >= 0:
        finalized_casualty.alertness_verbal = model_predictions[MODEL_1_INDEX].alertness_verbal
    else:
        finalized_casualty.alertness_verbal = 0

    # finalizing alertness motor
    system_print("Finalizing alertness motor prediction")
    if model_predictions[MODEL_3_INDEX].alertness_motor >= 0:
        finalized_casualty.alertness_motor = model_predictions[MODEL_3_INDEX].alertness_motor
    else:
        finalized_casualty.alertness_motor = 0

# waits for specified models to finish predictions
def wait_for_apriltag():
    timeout_tracker = 0
    while not received_april:
        if timeout_tracker == 0:
            system_print("Waiting for AprilTag...")

        # checking for timeout
        if timeout_tracker == int(APRILTAG_TIMEOUT/0.2):
            break
        time.sleep(0.2)
        timeout_tracker += 1
        continue

    if not received_april:
        system_print("Failed to receive AprilTag")

# resets tracking variables
def reset_trackers():
    system_print("Reseting trackers")
    global received_april
    received_april              = False

    for i in range(NUM_OF_MODELS):
        prediction_received[i] = False

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
def setApriltag():
    system_print("Setting AprilTag")
    finalized_casualty.apriltag = apriltag

# handles actions that need to take place when a timer starts
def onPredictionTimerStart():
    system_print("Timer has started")
    reset_casualty_objects()
    reset_trackers()
    reset_weight_array()

# handles actions that need to take place when a timer finishes
def onPredictionTimerEnd():
    system_print("Timer has ended")
    wait_for_predictions()
    finalize_afflication_values()
    publish_reports()
    reset_casualty_objects()
    reset_trackers()
    reset_weight_array()

# handles actions that need to take place when a timer is cancelled
def onPredictionTimerCancel():
    system_print("Timer has been cancelled")
    reset_casualty_objects()
    reset_trackers()
    reset_weight_array()

# handles actions corresponding to the timer starting and stopping
def handlePredictionTimerStatus(msg:Timer_status):
    global previousPredictionTimerState

    # checking if the timer status has changed
    if previousPredictionTimerState != msg.timer_status:
        # updating timer
        previousPredictionTimerState = msg.timer_status

        # checking timer state
        if msg.timer_status == TIMER_STARTED:
            onPredictionTimerStart()
        elif msg.timer_status == TIMER_ENDED:
            onPredictionTimerEnd()
        else:
            onPredictionTimerCancel()

# handles actions that take place when the apriltag timer starts
def onApriltagTimerStart():
    system_print("AprilTag timer has started")
    reset_apriltag()

# handles actions that take place when the apriltag timer ends
def onApriltagTimerEnd():
    system_print("AprilTag timer has ended")
    wait_for_apriltag()
    setApriltag()
    reset_apriltag

# handles actions that take place when the apriltag timer is canceled
def onApriltagTimerCancel():
    system_print("AprilTag timer has been cancelled")
    reset_apriltag()

# handles actions based on the state of the AprilTag timer
def handle_apriltag_timer_status(msg:Timer_status):
    global previousApriltagTimerState

    # checking if the timer status has changed
    if previousApriltagTimerState != msg.timer_status:
        # updating timer
        previousApriltagTimerState = msg.timer_status

        # checking timer state
        if msg.timer_status == TIMER_STARTED:
            onApriltagTimerStart()
        elif msg.timer_status == TIMER_ENDED:
            onApriltagTimerEnd()
        else:
            onApriltagTimerCancel()

# gets the assigned AprilTag and sets it
def assign_apriltag(assigned_apriltag):
    system_print("Received AprilTag")
    global apriltag
    apriltag = assigned_apriltag.apriltag
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
    model_predictions[casualty_ros.model].is_coherent            = casualty_ros.is_coherent
    prediction_received[casualty_ros.model] = True

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
    rospy.Subscriber('prediction_timer_status', Timer_status, handlePredictionTimerStatus)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handle_apriltag_timer_status)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, assign_apriltag)
    rospy.Subscriber('model_predictions', Casualty_prediction, receive_model_predictions)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
