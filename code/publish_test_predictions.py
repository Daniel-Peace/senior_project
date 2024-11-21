#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program simulates multiple models making predictions about a casualty. You can
# adjust the TIMER_BASED constant in order to choose to run fake predictions manually or
# have them run when the prediction timer starts.
#
# You can add more simulated predictions if you wish, you will just need to also add them to 
# either the model_predictions list or the handlePredictionTimerStatus if you wish for
# them to be used. You should also add an elif to the user prompt section to include any 
# new choices
# -------------------------------------------------------------------------------------------

import rospy
import time
from messages.msg       import Casualty_prediction
from messages.msg       import Timer_status

# timer states (enum)
TIMER_ENDED         = 0
TIMER_STARTED       = 1
TIMER_CANCELLED     = 2

# global variables
previous_timer_status = -1
model_predictions = []

# ================================== CHANGE THESE IF NEEDED ===================================

# Set to True if you want to have this node run based on the prediction timer
TIMER_BASED = True

# computer vision model
model_0_prediction = Casualty_prediction()
model_0_prediction.severe_hemorrhage      = 1
model_0_prediction.respiratory_distress   = -1
model_0_prediction.heart_rate             = -1
model_0_prediction.respiratory_rate       = -1
model_0_prediction.trauma_head            = 0
model_0_prediction.trauma_torso           = 1
model_0_prediction.trauma_lower_ext       = 1
model_0_prediction.trauma_upper_ext       = 1
model_0_prediction.alertness_ocular       = -1
model_0_prediction.alertness_verbal       = -1
model_0_prediction.alertness_motor        = -1
model_0_prediction.is_coherent            = True
model_0_prediction.model                  = 0

# auditory model
model_1_prediction = Casualty_prediction()
model_1_prediction.severe_hemorrhage      = 1
model_1_prediction.respiratory_distress   = 0
model_1_prediction.heart_rate             = -1
model_1_prediction.respiratory_rate       = -1
model_1_prediction.trauma_head            = 0
model_1_prediction.trauma_torso           = 0
model_1_prediction.trauma_lower_ext       = 2
model_1_prediction.trauma_upper_ext       = 1
model_1_prediction.alertness_ocular       = -1
model_1_prediction.alertness_verbal       = 0
model_1_prediction.alertness_motor        = -1
model_1_prediction.is_coherent            = True
model_1_prediction.model                  = 1

# face mesh/joints
model_2_prediction = Casualty_prediction()
model_2_prediction.severe_hemorrhage      = -1
model_2_prediction.respiratory_distress   = -1
model_2_prediction.heart_rate             = 63
model_2_prediction.respiratory_rate       = -1
model_2_prediction.trauma_head            = -1
model_2_prediction.trauma_torso           = -1
model_2_prediction.trauma_lower_ext       = -1
model_2_prediction.trauma_upper_ext       = -1
model_2_prediction.alertness_ocular       = 1
model_2_prediction.alertness_verbal       = -1
model_2_prediction.alertness_motor        = 1
model_2_prediction.model                  = 2

# depth camera
model_3_prediction = Casualty_prediction()
model_3_prediction.severe_hemorrhage      = -1
model_3_prediction.respiratory_distress   = 1
model_3_prediction.heart_rate             = -1
model_3_prediction.respiratory_rate       = 144
model_3_prediction.trauma_head            = -1
model_3_prediction.trauma_torso           = -1
model_3_prediction.trauma_lower_ext       = -1
model_3_prediction.trauma_upper_ext       = -1
model_3_prediction.alertness_ocular       = -1
model_3_prediction.alertness_verbal       = -1
model_3_prediction.alertness_motor        = -1
model_3_prediction.is_coherent            = True
model_3_prediction.model                  = 3

# declaring and initializing model 4 prediction
model_4_prediction = Casualty_prediction()
model_4_prediction.severe_hemorrhage      = -1
model_4_prediction.respiratory_distress   = 1
model_4_prediction.heart_rate             = 144
model_4_prediction.respiratory_rate       = 50
model_4_prediction.trauma_head            = -1
model_4_prediction.trauma_torso           = -1
model_4_prediction.trauma_lower_ext       = -1
model_4_prediction.trauma_upper_ext       = -1
model_4_prediction.alertness_ocular       = -1
model_4_prediction.alertness_verbal       = -1
model_4_prediction.alertness_motor        = -1
model_4_prediction.is_coherent            = True
model_4_prediction.model                  = 4

# adding models to "model_predictions"
model_predictions.append(model_0_prediction)
model_predictions.append(model_1_prediction)
model_predictions.append(model_2_prediction)
model_predictions.append(model_3_prediction)
model_predictions.append(model_4_prediction)

# =============================================================================================

# initializing ROS node
rospy.init_node('publish_test_predictions', anonymous=True)

# initializing publisher
publisher = rospy.Publisher('model_predictions', Casualty_prediction, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# just helps reduce number of long print statements in code
def print_horizontal_line():
    print("---------------------------------------------------------------------")    

# this funcion publishes a model prediction based on the passed in index
def publish_prediction(model_number):
    print_horizontal_line()
    system_print("Publishing model " + str(model_number) + " prediction...")
    publisher.publish(model_predictions[model_number])

# This function publishes all reports added to this function when the prediction timer starts
def handlePredictionTimerStatus(msg:Timer_status):
    global previous_timer_status

    # checking if the timer status has changed
    if previous_timer_status != msg.timer_status:
        # updating AprilTag timer state
        previous_timer_status = msg.timer_status

        # checking if prediction timer has started
        if msg.timer_status == TIMER_STARTED:

            # ================================== CHANGE THESE IF NEEDED ===================================

            # auditory
            print_horizontal_line()
            system_print("Publishing model " + str(model_1_prediction.model) + " prediction...")
            publisher.publish(model_1_prediction)

            # face mesh/joints
            print_horizontal_line()
            system_print("Publishing model " + str(model_2_prediction.model) + " prediction...")
            publisher.publish(model_2_prediction)

            # depth camera
            print_horizontal_line()
            system_print("Publishing model " + str(model_3_prediction.model) + " prediction...")
            publisher.publish(model_3_prediction)

            # =============================================================================================

# "main function" of the program
if __name__ == '__main__':
    # checking if in manual mode or timer mode
    if TIMER_BASED:
        # creating ros subscriber for prediction timer
        rospy.Subscriber('prediction_scanning_timer_state', Timer_status, handlePredictionTimerStatus)

        # loops until user kills the program
        while not rospy.is_shutdown():
            # sleeping to slow the loop down
            time.sleep(0.2)
        print_horizontal_line()
        system_print("Exiting...")
        print_horizontal_line()
        exit(0)
    else:
        # looping until program is terminated
        while True:
            # prompting user to select action
            print_horizontal_line()
            system_print("Please enter the number of the model you would like to simulate predicting\n     or type q to quit:")

            # looping unitl a valid choice is entered or the program is quit
            while True:
                # prompting user
                print_horizontal_line()
                choice = input("\u001b[34m-> \u001b[0m")

                # checking user's choice
                if choice == '0':
                    publish_prediction(0)
                    break
                elif choice == '1':
                    publish_prediction(1)
                    break
                elif choice == '2':
                    publish_prediction(2)
                    break
                elif choice == '3':
                    publish_prediction(3)
                    break
                elif choice == '4':
                    publish_prediction(4)
                    break
                elif choice == 'q' or choice == 'Q':
                    print_horizontal_line()
                    system_print("Exiting...")
                    print_horizontal_line()
                    exit(0)
                else:
                    print_horizontal_line()
                    system_print("\u001b[31mInvalid choice...\u001b[0m")
