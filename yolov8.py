#!/usr/bin/env python3
# ---------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# ---------------------------------------------------------------------------------------------
# This program implements Ultralytics' YOLOv8 to make predictions about possible affliction
# someone may have. It provides the options of running the model using a path provided by the
# user or by using images published to the "picked_image" topic. The results of the model
# prediction are stored in a ROS message of type "Casualty_prediction" and published to the
# "model_predictions" topic.
#
# The ROS message "Casualty_prediction" initializes to all 0s for all affliction values
# With that, when the model can make predictions about an affliction but 
# does not, it should be assumed that the casualty does not have that
# specific affliction and the field in the ROS message should be left as zero.
#
# The model pulls the weights from a folder in the src directory named "weights".
# You can change the weights being used by changing the WEIGHTS constant to a path of
# your choosing.
#
# The confidence value threshold for what predictions are published can be set with the
# constant CONFIDENCE_THRESHOLD. Any predictions that have a confidence value below this
# threshold will be ignored.
#
# You may choose if you would like to pass in a path to an image or have the program pull
# images from the "picked_image" topic by using the DEBUG flag. If this is set to true, you
# will have the choice to run the program with either a path to an image or using images
# published to the "picked_image" topic. If DEBUG is set to false the program will run with
# the camera by default.
#
# Affliction types that can be predicted:
# - trauma_head
# - trauma_torso
# - trauma_lower_ext
# - amputation_lower_ext
# - trauma_upper_ext
# - amputation_upper_ext
# - severe_hemorrhage
#
# ROS topic subscriptions:
# - /picked_image
#
# ROS topics for publishing
# - /model_predictions
#
# ---------------------------------------------------------------------------------------------

import time
import rospy
import numpy as np
from PIL                import Image as PImage
from sensor_msgs.msg    import Image
from ultralytics        import YOLO
from cv_bridge          import CvBridge
from messages.msg       import Casualty_prediction

# ================================== CHANGE THESE IF NEEDED ===================================

DEBUG                   = True
WEIGHTS                 = './weights/iteration_4.pt'
CONFIDENCE_THRESHOLD    = 0

# =============================================================================================

# general constants
MODEL_NUMBER            = 0

# operation mode (enum)
RUN_WITH_CAMERA         = 0
RUN_WITH_PATH           = 1

# affliction types that YOLOv8 will predict (enum)
TRAUMA_HEAD             = 0
TRAUMA_TORSO            = 1
TRAUMA_LOWER_EXT        = 2
AMPUTATION_LOWER_EXT    = 3
TRAUMA_UPPER_EXT        = 4
AMPUTATION_UPPER_EXT    = 5
SEVERE_HEMORRHAGE       = 6

# creating ROS node
rospy.init_node('model_0', anonymous=True)

# creating publisher
publisher = rospy.Publisher('model_predictions', Casualty_prediction, queue_size=10)

# initializing YOLOv8 model
model = YOLO(WEIGHTS)

# bridge for if a camera is being used
bridge = None

# used for printing formatted system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# just helps reduce number of long print statements in code
def print_horizontal_line():
    print("---------------------------------------------------------------------")

# publishes the results of a prediction to "model_predictions"
def publish_results(predictions):
    # declaring and initializing ROS message
    casualty_prediction = Casualty_prediction()
    print("not initialized")
    print(casualty_prediction)
    casualty_prediction.is_coherent            = True
    casualty_prediction.model                  = MODEL_NUMBER

    # looping over results from prediction and updating ROS message
    for prediction in predictions:
        for cls, conf in zip(prediction.boxes.cls.cpu().numpy(), prediction.boxes.conf.cpu().numpy()):
            # checking if the prediction has a confidence value above the set threshold
            if conf < CONFIDENCE_THRESHOLD:
                continue
            else:
                # checking affliction type and updating ROS message
                if cls == TRAUMA_HEAD:
                    casualty_prediction.trauma_head = 1
                elif cls == TRAUMA_TORSO:
                    casualty_prediction.trauma_torso = 1
                elif cls == TRAUMA_LOWER_EXT:
                    if casualty_prediction.trauma_lower_ext != 2:
                        casualty_prediction.trauma_lower_ext = 1
                elif cls == AMPUTATION_LOWER_EXT:
                    casualty_prediction.trauma_lower_ext = 2
                elif cls == TRAUMA_UPPER_EXT:
                    if casualty_prediction.trauma_upper_ext != 2:
                        casualty_prediction.trauma_upper_ext = 1
                elif cls == AMPUTATION_UPPER_EXT:
                    casualty_prediction.trauma_upper_ext = 2
                elif cls == SEVERE_HEMORRHAGE:
                    casualty_prediction.severe_hemorrhage = 1
                else:
                    print_horizontal_line()
                    system_print("\u001b[31mERROR - invalid affliction class\u001b[0m")

    # printing ROS message for reference
    print_horizontal_line()
    system_print("ROS message")
    print_horizontal_line()
    print(casualty_prediction)

    # publishing ROS message
    publisher.publish(casualty_prediction)

# sets up the model to run with a camera
def setup_for_camera_use():
    global bridge

    print_horizontal_line()
    system_print("Setting up model to run with camera...")

    # setting up bridge for converting ROS images to cv images
    bridge = CvBridge()

    # setting up callback function to receive images and make predections on them
    rospy.Subscriber('picked_image', Image, run_model_on_image_from_camera)

    system_print("Use ctrl + c if you wish to quit")
    system_print("Waiting for image...")

    # running node until user terminates the process
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)

# runs YOLOv8 on an image published to the "/picked_image" topic
def run_model_on_image_from_camera(raw_image):
    system_print("Received image")

    # converting image to a cv image
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

    # converting cv image to numpy array
    np_image = np.array(cv_image)

    print_horizontal_line()
    system_print("Prediction results:")
    print_horizontal_line()

    # running model on the numpy array
    results = model.predict(np_image)

    # printing results of prediction as JSON for reference
    for result in results:
        print("\n+-------+---------------+")
        print("| class\t| Confidence\t|")
        print("+-------+---------------+")
        for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
            print("| " + str(int(cls)) + "\t| " + str(conf) + "\t|")
            print("+-------+---------------+")

    # publishing results to ROS topic
    publish_results(results)

# runs YOLOv8 on an image specified by a path
def run_model_with_path():
    while True:
        # getting image path from user
        print_horizontal_line()
        system_print("Enter a path to an image or type q to quit:")
        print_horizontal_line()
        image_path = input("\u001b[34m-> \u001b[0m")

        # checking if the user chose to quit
        if image_path == 'q' or image_path == 'Q':
            print_horizontal_line()
            system_print("Exiting...")
            print_horizontal_line()
            exit(0)

        # opening image
        image = PImage.open(image_path)

        print_horizontal_line()
        system_print("Prediction results:")
        print_horizontal_line()

        # running model on image
        results = model.predict(source=image)

        # printing results of prediction for reference
        for result in results:
            print("\n+-------+---------------+")
            print("| class\t| Confidence\t|")
            print("+-------+---------------+")
            for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                print("| " + str(int(cls)) + "\t| " + str(conf) + "\t|")
                print("+-------+---------------+")

        # publishing results to ROS topic
        publish_results(results)

# "main function" of the program
if __name__ == "__main__":
    # checking if program is in debug mode
    if DEBUG:
        # prompting user
        print_horizontal_line()
        system_print("Choose how you would like to run this program or type q to quit:")
        print("\ta. Run using usb_camera node")
        print("\tb. Run using a path to an image")
        
        # looping unitl a valid choice is entered or the program is quit
        while True:
            # prompting user
            print_horizontal_line()
            choice = input("\u001b[34m-> \u001b[0m")

            # checking user's choice
            if choice == 'a':
                # setting up for use with camera
                setup_for_camera_use()
                break

            elif choice == 'b':
                print_horizontal_line()
                system_print("Setting up model to run with path...")
                run_model_with_path()
                break

            elif choice == 'q' or choice == 'Q':
                print_horizontal_line()
                system_print("Exiting...")
                print_horizontal_line()
                exit(0)

            else:
                print_horizontal_line()
                system_print("\u001b[31mInvalid choice...\u001b[0m")
    else:
        # setting up for use with camera
        setup_for_camera_use()
