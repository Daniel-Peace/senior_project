#!/usr/bin/env python3

# imports
import numpy as np
import time
import json
import rospy
from messages.msg       import Prediction
from messages.msg       import Prediction_element
from casualty           import Casualty
from PIL                import Image as PImage
from ultralytics        import YOLO
from sensor_msgs.msg    import Image
from cv_bridge          import CvBridge

# constants
RUN_WITH_CAMERA         = 0
RUN_WITH_PATH           = 1
CONFIDENCE_THRESHOLD    = 0

# prediction types
TRAUMA_HEAD             = 0
TRAUMA_TORSO            = 1
TRAUMA_LOWER_EXT        = 2
AMPUTATION_LOWER_EXT    = 3
TRAUMA_UPPER_EXT        = 4
AMPUTATION_UPPER_EXT    = 5
SEVERE_HEMORRHAGE       = 6

# creating publisher
publisher = rospy.Publisher('model_0_predictions', Prediction, queue_size=10)

# initializing yoloV8 model
model = YOLO("./yoloV8_weights/best.pt")

# creating bridge object
bridge = CvBridge()

def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# publishes the results of a prediction to "yoloV8_prediction"
def publish_results(results):
    # looping over results to add predictions to casualty object
    casualty = Casualty()
    for result in results:
        for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
            if conf < CONFIDENCE_THRESHOLD:
                continue
            else:
                if cls == TRAUMA_HEAD:
                    casualty.trauma_head = 1
                elif cls == TRAUMA_TORSO:
                    casualty.trauma_torso = 1
                elif cls == TRAUMA_LOWER_EXT:
                    if casualty.trauma_lower_ext != 2:
                        casualty.trauma_lower_ext = 1
                elif cls == AMPUTATION_LOWER_EXT:
                    casualty.trauma_lower_ext = 2
                elif cls == TRAUMA_UPPER_EXT:
                    if casualty.trauma_upper_ext != 2:
                        casualty.trauma_upper_ext = 1
                elif cls == AMPUTATION_UPPER_EXT:
                    casualty.trauma_upper_ext = 2
                elif cls == SEVERE_HEMORRHAGE:
                    casualty.severe_hemorrhage = 1
                else:
                    print("---------------------------------------------------------------------")
                    system_print("\u001b[31mERROR - invalid affliction class\u001b[0m")
    print("---------------------------------------------------------------------")
    system_print("Results as casualty object")
    print("---------------------------------------------------------------------")
    casualty.print_self()

    # creating ROS prediction message
    prediction = Prediction()

    # creating prediction element for trauma_head
    prediction_element = Prediction_element()
    prediction_element.type = Casualty.TRAUMA_HEAD
    if casualty.trauma_head == -1:
        prediction_element.value = 0
    else:
        prediction_element.value = casualty.trauma_head
    prediction.prediction_elements.append(prediction_element)

    # creating prediction element for trauma_torso
    prediction_element = Prediction_element()
    prediction_element.type = Casualty.TRAUMA_TORSO
    if casualty.trauma_torso == -1:
        prediction_element.value = 0
    else:
        prediction_element.value = casualty.trauma_torso
    prediction.prediction_elements.append(prediction_element)

    # creating prediction element for trauma_torso
    prediction_element = Prediction_element()
    prediction_element.type = Casualty.TRAUMA_LOWER_EXT
    if casualty.trauma_lower_ext == -1:
        prediction_element.value = 0
    else:
        prediction_element.value = casualty.trauma_lower_ext
    prediction.prediction_elements.append(prediction_element)

    # creating prediction element for trauma_torso
    prediction_element = Prediction_element()
    prediction_element.type = Casualty.TRAUMA_UPPER_EXT
    if casualty.trauma_upper_ext == -1:
        prediction_element.value = 0
    else:
        prediction_element.value = casualty.trauma_upper_ext
    prediction.prediction_elements.append(prediction_element)

    # creating prediction element for trauma_torso
    prediction_element = Prediction_element()
    prediction_element.type = Casualty.SEVERE_HEMORRHAGE
    if casualty.severe_hemorrhage == -1:
        prediction_element.value = 0
    else:
        prediction_element.value = casualty.severe_hemorrhage
    prediction.prediction_elements.append(prediction_element)

    # printing ROS message for reference
    print("---------------------------------------------------------------------")
    system_print("ROS message")
    print("---------------------------------------------------------------------")
    print(prediction)

    # Publishing ROS message
    publisher.publish(prediction)

# runs yoloV8 on an image published to the "usb_cam/image_raw"
def run_predictor_with_camera(raw_image):

    # converting image to a cv image
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

    # converting cv image to numpy array
    np_image = np.array(cv_image)

    print("---------------------------------------------------------------------")
    system_print("Prediction results:")
    print("---------------------------------------------------------------------")

    # running model on the numpy array
    results = model.predict(np_image)

    # publishing resutls to ros topic
    publish_results(results)



# runs yoloV8 on an image specified by a path to an image
def run_predictor_with_path():
    while True:
        # getting image path from user
        print("---------------------------------------------------------------------")
        system_print("Enter a path to an image:")
        print("---------------------------------------------------------------------")
        image_path = input("\u001b[34m-> \u001b[0m")

        # check if the user chose to quit
        if image_path == 'q' or image_path == 'Q':
            print("---------------------------------------------------------------------")
            system_print("Exiting...")
            print("---------------------------------------------------------------------")
            break

        # opening image
        image = PImage.open(image_path)

        print("---------------------------------------------------------------------")
        system_print("Prediction results:")
        print("---------------------------------------------------------------------")

        # running model on image
        results = model.predict(source=image)

        # publishing resutls to ros topic
        publish_results(results)



# sets up the predictor based on the users mode choice
def setup_predictor(choice):
    # creating ROS node
    rospy.init_node('model_1', anonymous=True)

    # checking which mode to run the predictor in
    if choice == RUN_WITH_CAMERA:
        print("---------------------------------------------------------------------")
        system_print("Setting up model to run with camera...")
        # registering callback functions
        rospy.Subscriber('usb_cam/image_raw', Image, run_predictor_with_camera)
    else:
        print("---------------------------------------------------------------------")
        system_print("Setting up model to run with path...")
        # running predictor on image paths
        run_predictor_with_path()
        return

    # running node until user terminates the process
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)


# main function if script is run independently
if __name__ == "__main__":
    # prompting user
    print("---------------------------------------------------------------------")
    system_print("Choose how you would like to run this program\n\ta. Run using usb_camera node\n\tb. Run using a path to an image")

    # looping unitl a valid choice is entered or the program is quit
    while True:
        # prompting user
        print("---------------------------------------------------------------------")
        choice = input("\u001b[34m-> \u001b[0m")

        # checking user's choice
        if choice == 'a':
            setup_predictor(RUN_WITH_CAMERA)
            break
        elif choice == 'b':
            setup_predictor(RUN_WITH_PATH)
            break
        elif choice == 'q' or choice == 'Q':
            print("---------------------------------------------------------------------")
            system_print("Exiting...")
            print("---------------------------------------------------------------------")
            exit(0)
            break
        else:
            print("---------------------------------------------------------------------")
            system_print("\u001b[31mInvalid choice...\u001b[0m")
