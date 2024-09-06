# imports
import rospy
import time
from messages.msg       import Casualty_prediction

# constants
NUMBER_OF_MODELS = 6

# creating ROS node
rospy.init_node('publish_test_predictions', anonymous=True)

# initializing publisher
publisher = rospy.Publisher('model_predictions', Casualty_prediction, queue_size=10)

# this list holds test predictions
model_predictions = []

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# this function initializes the "model_predictions" list with test predictions
def initialize_predictions():
    # declaring and initializing model 1 prediction
    prediction = Casualty_prediction()
    prediction.severe_hemorrhage      = 1
    prediction.respiratory_distress   = -1
    prediction.heart_rate             = -1
    prediction.respiratory_rate       = -1
    prediction.trauma_head            = 0
    prediction.trauma_torso           = 1
    prediction.trauma_lower_ext       = 1
    prediction.trauma_upper_ext       = 1
    prediction.alertness_ocular       = -1
    prediction.alertness_verbal       = -1
    prediction.alertness_motor        = -1
    prediction.is_coherent            = True
    prediction.model                  = 0

    # adding prediction to list
    model_predictions.append(prediction)

    # declaring and initializing model 1 prediction
    prediction = Casualty_prediction()
    prediction.severe_hemorrhage      = 1
    prediction.respiratory_distress   = 0
    prediction.heart_rate             = -1
    prediction.respiratory_rate       = -1
    prediction.trauma_head            = 0
    prediction.trauma_torso           = 1
    prediction.trauma_lower_ext       = 1
    prediction.trauma_upper_ext       = 2
    prediction.alertness_ocular       = 0
    prediction.alertness_verbal       = 0
    prediction.alertness_motor        = 1
    prediction.is_coherent            = True
    prediction.model                  = 1

    # adding prediction to list
    model_predictions.append(prediction)

    # declaring and initializing model 2 prediction
    prediction = Casualty_prediction()
    prediction.severe_hemorrhage      = -1
    prediction.respiratory_distress   = 1
    prediction.heart_rate             = 123
    prediction.respiratory_rate       = 40
    prediction.trauma_head            = -1
    prediction.trauma_torso           = -1
    prediction.trauma_lower_ext       = -1
    prediction.trauma_upper_ext       = -1
    prediction.alertness_ocular       = -1
    prediction.alertness_verbal       = -1
    prediction.alertness_motor        = -1
    prediction.is_coherent            = True
    prediction.model                  = 2

    # adding prediction to list
    model_predictions.append(prediction)

    # declaring and initializing model 3 prediction
    prediction = Casualty_prediction()
    prediction.severe_hemorrhage      = -1
    prediction.respiratory_distress   = -1
    prediction.heart_rate             = -1
    prediction.respiratory_rate       = -1
    prediction.trauma_head            = -1
    prediction.trauma_torso           = -1
    prediction.trauma_lower_ext       = -1
    prediction.trauma_upper_ext       = -1
    prediction.alertness_ocular       = 1
    prediction.alertness_verbal       = 0
    prediction.alertness_motor        = 1
    prediction.is_coherent            = True
    prediction.model                  = 3

    # adding prediction to list
    model_predictions.append(prediction)

# this function publishes a specific prediction from "model_predictions" 
# based on what index is passed in
def publish_prediction(model_number):
    print("------------------------------------------------------")
    system_print("Publishing model " + str(model_number) + " prediction...")
    publisher.publish(model_predictions[model_number])



if __name__ == '__main__':
    # initializing prediction list
    initialize_predictions()

    # looping until program is terminated
    while True:
        # prompting user to select action
        print("------------------------------------------------------")
        system_print("Please select an option below\n     or type q to quit:")
        print("\ta. publish model 0 prediction")
        print("\tb. publish model 1 prediction")
        print("\tc. publish model 2 prediction")
        print("\td. publish model 3 prediction")
        print("\te. publish model 4 prediction")
        print("\tf. publish model 5 prediction")

        # looping unitl a valid choice is entered or the program is quit
        while True:
            # prompting user
            print("------------------------------------------------------")
            choice = input("\u001b[34m-> \u001b[0m")

            # checking user's choice
            if choice == 'a':
                publish_prediction(0)
                break
            elif choice == 'b':
                publish_prediction(1)
                break
            elif choice == 'c':
                publish_prediction(2)
                break
            elif choice == 'd':
                publish_prediction(3)
                break
            elif choice == 'e':
                break
            elif choice == 'f':
                break
            elif choice == 'q' or choice == 'Q':
                print("------------------------------------------------------")
                system_print("Exiting...")
                print("------------------------------------------------------")
                exit(0)
            else:
                print("------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")
