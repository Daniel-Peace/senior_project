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

#   +------------------------------+
#   | Affliction types and values  |
#   +----------------------+-------+
#   | SEVERE_HEMORRHAGE    |   0   |
#   +----------------------+-------+
#   | RESPIRATORY_DISTRESS |   1   |
#   +----------------------+-------+
#   | HEART_RATE           |   2   |
#   +----------------------+-------+
#   | RESPIRATORY_RATE     |   3   |
#   +----------------------+-------+
#   | TRAUMA_HEAD          |   4   |
#   +----------------------+-------+
#   | TRAUMA_TORSO         |   5   |
#   +----------------------+-------+
#   | TRAUMA_LOWER_EXT     |   6   |
#   +----------------------+-------+
#   | TRAUMA_UPPER_EXT     |   7   |
#   +----------------------+-------+
#   | ALERTNESS_OCULAR     |   8   |
#   +----------------------+-------+
#   | ALERTNESS_VERBAL     |   9   |
#   +----------------------+-------+
#   | ALERTNESS_MOTOR      |   10  |
#   +----------------------+-------+

# imports
import time
import rospy
import json
from casualty       import Casualty
from messages.msg   import Assigned_apriltag
from messages.msg   import Casualty_prediction
from messages.msg   import Critical_report
from messages.msg   import Injury_report
from messages.msg   import Timer_status
from messages.msg   import Vitals_report
from messages.msg   import ModelPredictionStatus
from messages.msg   import ModelPredictionStatuses
from messages.msg   import LoopState

# timer states
TIMER_ENDED         = 0
TIMER_STARTED       = 1
TIMER_CANCELLED     = 2

# general constants
DEFAULT_HR          = 100
DEFAULT_RR          = 25
PREDICTION_TIMEOUT  = 2
APRILTAG_TIMEOUT    = 2

# this class holds relavent information about an AI or ML model
class Model:
    # constructor
    def __init__(self, name="model", weight=0, casualty=Casualty(), has_predicted=False, coherent_dependent=False, determines_coherent=False):
        self.name                   = name
        self.weight                 = weight
        self.casualty               = casualty
        self.has_predicted          = has_predicted
        self.coherent_dependent     = coherent_dependent
        self.determines_coherent    = determines_coherent

    # resets model
    def reset(self):
        self.has_predicted = False
        self.casualty.reset()

    # prints model
    def print_self(self):
        print("+---------------------+--------------------+")
        print("| %-20s|%-20s|" % ("name", self.name))
        print("+---------------------+--------------------+")
        print("| %-20s|%-20s|" % ("weight", self.weight))
        print("+---------------------+--------------------+")
        print("| %-20s|%-20s|" % ("has predicted", self.has_predicted))
        print("+---------------------+--------------------+")
        print("| %-20s|%-20s|" % ("coherent dependent", self.coherent_dependent))
        print("+---------------------+--------------------+")
        print(f"|{'casualty':^42}|")
        print("+------------------------------------------+")
        self.casualty.print_self()
        print("+------------------------------------------+\n")

# holds configuration data from model_configs.json
model_configs = None

# stores the april tag for the current casualty
apriltag = -1

# tracks if an april tag has been assigned to the current casualty
received_april = False

# tracks the previous state of the timers
previousPredictionTimerState    = -1
previousApriltagTimerState      = -1

# list of models
model_array: Model = []

# opening json config file
with open('model_configs.json', 'r') as file:
    model_configs = json.load(file)

# loading model info into array
for model in model_configs['models']:
    model = Model(name=model['name'], weight=model['weight'], casualty=Casualty(),determines_coherent=model['determines_coherent'])
    model_array.append(model)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# printing current models' info
print("------------------------------------------------------")
system_print("Loaded the following models:")
print("------------------------------------------------------\n")
for model in model_array:
    model.print_self()

# creating casualty object for final report
finalized_casualty  = Casualty()

# initializing node
rospy.init_node('vote_and_create', anonymous=True)

final_report_publisher              = rospy.Publisher('final_report', Casualty_prediction, queue_size=10)
model_prediction_statuses_publisher = rospy.Publisher('model_prediction_statuses', ModelPredictionStatuses, queue_size=10)
loop_status_publisher               = rospy.Publisher('loop_state', LoopState, queue_size=10)

# resets the weight array to the default weights
def reset_weights():
    global model_configs
    for index, model in enumerate(model_array):
        model.weight = model_configs['models'][index]['weight']

# loops until all needed predictions have been received
def wait_for_predictions():
    received_all_predictions = False
    timeout_tracker = 0

    loopState = LoopState()
    loopState.state = "Waiting for predictions"
    loop_status_publisher.publish(loopState)

    # waiting until all predictions have been received or timeout occurs
    while not received_all_predictions:
        model_prediction_statuses = ModelPredictionStatuses()
        if timeout_tracker == 0:
            system_print("Waiting for predictions...")

        # checking which models have published predictions
        received_all_predictions = True
        for model in model_array:
            model_prediction_status = ModelPredictionStatus()
            model_prediction_status.model_name = model.name
            model_prediction_status.made_prediction = model.has_predicted
            model_prediction_statuses.modelPredictionStatuses.append(model_prediction_status)
            if not model.has_predicted:
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
    print("+---------------------+---------------------+")
    print("| %-20s| %-20s|" % ("name", 'status'))
    print("+---------------------+---------------------+")
    for model in model_array:
        print("| %-20s| %-20s|" % (model.name, str(model.has_predicted)))
        print("+---------------------+---------------------+")
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

    loopState = LoopState()
    loopState.state = "Publishing Reports"
    loop_status_publisher.publish(loopState)
    time.sleep(2)
    
    loopState = LoopState()
    loopState.state = "Waiting to assign AprilTag"
    loop_status_publisher.publish(loopState)

# handles combining and finalizing reports
def finalize_afflication_values():
    system_print("Finalizing predictions")

    # setting time_ago value
    finalized_casualty.time_ago = 20

    # checking if current casualty is coherent
    is_coherent_vote = 0
    is_coherent = True
    for model in model_array:
        if model.determines_coherent:
            if model.casualty.is_coherent:
                is_coherent_vote += model.weight
            else:
                is_coherent_vote -= model.weight
    if is_coherent_vote < 0:
        is_coherent == False
        system_print("Casualty is incoherent")
    else:
        system_print("Casualty is coherent")

    # adding up votes for severe_hemorrhage
    system_print("Finalizing severe hemorrhage prediction")
    affliction_vote = 0

    # looping over models
    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        if model.casualty.severe_hemorrhage == 0:
            affliction_vote -= model.weight
        elif model.casualty.severe_hemorrhage == 1:
            affliction_vote += model.weight

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.severe_hemorrhage = 1
    else:
        finalized_casualty.severe_hemorrhage = 0

    # adding up votes for respiratory_distress
    system_print("Finalizing respiratory distress prediction")
    affliction_vote = 0

    # looping over models
    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        if model.casualty.respiratory_distress == 0:
            affliction_vote -= model.weight
        elif model.casualty.respiratory_distress == 1:
            affliction_vote += model.weight

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.respiratory_distress = 1
    else:
        finalized_casualty.respiratory_distress = 0

    # averaging heart rate values
    system_print("Finalizing heart rate prediction")
    hr_weighted_sum     = 0
    weight_sum          = 0
    has_changed         = False
    for model in model_array:
        if model.casualty.heart_rate >= 0:
            hr_weighted_sum += model.casualty.heart_rate * model.weight
            weight_sum      += model.weight
            has_changed     = True
    if has_changed:
        finalized_casualty.heart_rate = int(hr_weighted_sum / weight_sum + 0.5)
    else:
        finalized_casualty.heart_rate = DEFAULT_HR

    # averaging respiratory rate values
    system_print("Finalizing respiratory rate prediction")
    rr_weighted_sum     = 0
    weight_sum          = 0
    has_changed         = False
    for model in model_array:
        if model.casualty.respiratory_rate >= 0:
            rr_weighted_sum += model.casualty.respiratory_rate
            weight_sum      += model.weight
            has_changed     = True
    if has_changed:
        finalized_casualty.respiratory_rate = int(rr_weighted_sum / weight_sum + 0.5)
    else:
        finalized_casualty.respiratory_rate = DEFAULT_RR

    # adding up votes for trauma_head
    system_print("Finalizing trauma head prediction")
    affliction_vote = 0

    # looping over models
    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        if model.casualty.trauma_head == 0:
            affliction_vote -= model.weight
        elif model.casualty.trauma_head == 1:
            affliction_vote += model.weight

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.trauma_head = 1
    else:
        finalized_casualty.trauma_head = 0

    # adding up votes for trauma_torso
    system_print("Finalizing trauma torso prediction")
    affliction_vote = 0

    # looping over models
    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        if model.casualty.trauma_torso == 0:
            affliction_vote -= model.weight
        elif model.casualty.trauma_torso == 1:
            affliction_vote += model.weight

    # checking final vote count
    if affliction_vote > 0:
        finalized_casualty.trauma_torso = 1
    else:
        finalized_casualty.trauma_torso = 0

    # adding up votes for trauma_lower_ext
    system_print("Finalizing trauma lower ext. prediction")
    lower_injury_vote       = 0
    lower_amputation_vote   = 0

    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        if model.casualty.trauma_lower_ext == 0:
            lower_injury_vote       -= model.weight
            lower_amputation_vote   -= model.weight
        elif model.casualty.trauma_lower_ext == 1:
            lower_injury_vote       += model.weight
        elif model.casualty.trauma_lower_ext == 2:
            lower_injury_vote       += float(model.weight/2)
            lower_amputation_vote   += model.weight

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

    for model in model_array:
        # checking if current casualty is incoherent and 
        if not is_coherent:
            # skiping all coherent dependent models
            if model.coherent_dependent:
                continue

        # checking model prediction
        print("prediction" + str(model.casualty.trauma_upper_ext))
        if model.casualty.trauma_upper_ext == 0:
            upper_injury_vote       -= model.weight
            upper_amputation_vote   -= model.weight
        elif model.casualty.trauma_upper_ext == 1:
            upper_injury_vote       += model.weight
        elif model.casualty.trauma_upper_ext == 2:
            upper_injury_vote       += float(float(model.weight)/2)
            upper_amputation_vote   += model.weight

    print("injury count" + str(upper_injury_vote))
    print("amputation" + str(upper_amputation_vote))
    
    if (upper_amputation_vote > 0) and (upper_amputation_vote > upper_injury_vote):
        finalized_casualty.trauma_upper_ext = 2
    elif (upper_injury_vote > 0) and (upper_amputation_vote <= upper_injury_vote):
        finalized_casualty.trauma_upper_ext = 1
    else:
        finalized_casualty.trauma_upper_ext = 0

    # # finalizing alertness ocular
    # system_print("Finalizing alertness ocular prediction")
    # if model_predictions[MODEL_3_INDEX].alertness_ocular >= 0:
    #     finalized_casualty.alertness_ocular = model_predictions[MODEL_3_INDEX].alertness_ocular
    # else:
    #     finalized_casualty.alertness_ocular = 0

    # # finalizing alertness verbal
    # system_print("Finalizing alertness verbal prediction")
    # if model_predictions[MODEL_1_INDEX].alertness_verbal >= 0:
    #     finalized_casualty.alertness_verbal = model_predictions[MODEL_1_INDEX].alertness_verbal
    # else:
    #     finalized_casualty.alertness_verbal = 0

    # # finalizing alertness motor
    # system_print("Finalizing alertness motor prediction")
    # if model_predictions[MODEL_3_INDEX].alertness_motor >= 0:
    #     finalized_casualty.alertness_motor = model_predictions[MODEL_3_INDEX].alertness_motor
    # else:
    #     finalized_casualty.alertness_motor = 0

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
def reset_apriltag_tracker():
    system_print("Reseting AprilTag tracker")
    global received_april
    received_april = False

# resets apriltag value
def reset_apriltag():
    system_print("Reseting AprilTag")
    global apriltag
    apriltag = -1

# resets models in model_array
def reset_models():
    for model in model_array:
        model.reset()
    
# sets the AprilTag of the current casualty
def set_apriltag():
    system_print("Setting AprilTag")
    finalized_casualty.apriltag = apriltag

# handles actions that need to take place when a timer starts
def onPredictionTimerStart():
    system_print("Timer has started")    

# handles actions that need to take place when a timer finishes
def onPredictionTimerEnd():
    # finalizing and publishing
    system_print("Timer has ended")
    wait_for_predictions()
    finalize_afflication_values()
    publish_reports()

    # resetting
    reset_models()
    reset_apriltag()
    reset_apriltag_tracker
    finalized_casualty.reset()    

# handles actions that need to take place when a timer is cancelled
def onPredictionTimerCancel():
    system_print("Timer has been cancelled")
    reset_models()
    finalized_casualty.reset()
    set_apriltag()

# handles actions corresponding to the timer starting and stopping
def handlePredictionTimerStatus(msg:Timer_status):
    # bringing global variable into local scope
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

# handles actions that take place when the apriltag timer ends
def onApriltagTimerEnd():
    system_print("AprilTag timer has ended")
    set_apriltag()

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
def receive_apriltag(assigned_apriltag):
    system_print("Received AprilTag")
    global apriltag
    apriltag = assigned_apriltag.apriltag
    global received_april
    received_april = True
    system_print("Set AprilTag")

# callback function for when predictions are received from model 0
def receive_model_predictions(casualty_ros):
    model_array[casualty_ros.model].casualty.severe_hemorrhage      = casualty_ros.severe_hemorrhage
    model_array[casualty_ros.model].casualty.respiratory_distress   = casualty_ros.respiratory_distress
    model_array[casualty_ros.model].casualty.heart_rate             = casualty_ros.heart_rate
    model_array[casualty_ros.model].casualty.respiratory_rate       = casualty_ros.respiratory_rate
    model_array[casualty_ros.model].casualty.trauma_head            = casualty_ros.trauma_head
    model_array[casualty_ros.model].casualty.trauma_torso           = casualty_ros.trauma_torso
    model_array[casualty_ros.model].casualty.trauma_lower_ext       = casualty_ros.trauma_lower_ext
    model_array[casualty_ros.model].casualty.trauma_upper_ext       = casualty_ros.trauma_upper_ext
    model_array[casualty_ros.model].casualty.alertness_ocular       = casualty_ros.alertness_ocular
    model_array[casualty_ros.model].casualty.alertness_verbal       = casualty_ros.alertness_verbal
    model_array[casualty_ros.model].casualty.alertness_motor        = casualty_ros.alertness_motor
    model_array[casualty_ros.model].casualty.is_coherent            = casualty_ros.is_coherent
    model_array[casualty_ros.model].has_predicted                   = True

    system_print("Received prediction from model " + str(casualty_ros.model))
    print("------------------------------------------------------")
    print(casualty_ros)
    print("------------------------------------------------------")
    system_print("Updated casualty object for model " + str(casualty_ros.model))
    print("------------------------------------------------------")
    model_array[casualty_ros.model].casualty.print_self()
    print("------------------------------------------------------")


if __name__ == "__main__":
    print("---------------------------------------------------------------------")
    # registering callback functions
    system_print("Registering callback functions")
    rospy.Subscriber('prediction_timer_status', Timer_status, handlePredictionTimerStatus)
    rospy.Subscriber('apriltag_timer_status', Timer_status, handle_apriltag_timer_status)
    rospy.Subscriber('assigned_apriltag', Assigned_apriltag, receive_apriltag)
    rospy.Subscriber('model_predictions', Casualty_prediction, receive_model_predictions)

    system_print("Waiting for timer to start...")

    # loops until user kills the program
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)
