# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program implements Ultralytics' yoloV8. It provides the options of
# running the model using a path provided by the user or by using images published
# to the "picked_image" topic. The results of the model prediction are stored in a ROS
# message of type "Prediction.msg" and published to the "model_predictions" topic.
#
# This model only predicts certain types of afflictions. Fields in the ROS message that the
# model does not make predictions about are set to -1. When the model can make predictions
# about an afflication but does not, it is assumed that the casualty does not have that
# specific affliction and the field in the ROS message is set to zero to indicate this.
#
# The model pulls the weights from a folder in the src directory named yoloV8_weights.
# You can change the weights being used by changing WEIGHTS constant to a path of your chosing.
# The confidence value threshold for what predictions are published can be set with the
# constant CONFIDENCE_THRESHOLD.
#
# There is a DEBUG constant at the top of the program now. IF this is set to true, you
# will have the choice to run the program with either a path or with the camera. If
# it is set to false the program will run with the camera by default.
# -------------------------------------------------------------------------------------------

# imports
import json
import time
import rospy
import numpy as np
from PIL                import Image as PImage
from sensor_msgs.msg    import Image
from ultralytics        import YOLO
from cv_bridge          import CvBridge

# ROS messages
from messages.msg       import Casualty_prediction
from ultralytics.utils  import WEIGHTS_DIR

# general constants
DEBUG                   = False
RUN_WITH_CAMERA         = 0
RUN_WITH_PATH           = 1
WEIGHTS                 = './yoloV8_weights/iteration_4.pt'
CONFIDENCE_THRESHOLD    = 0
MODEL_NUMBER            = 0

# afflication types that yoloV8 will predict
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

# initializing yoloV8 model
model = YOLO(WEIGHTS)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# publishes the results of a prediction to "model_predictions"
def publish_results(results):
    # declaring and initializing ROS message
    casualty_prediction = Casualty_prediction()
    casualty_prediction.severe_hemorrhage      = 0
    casualty_prediction.respiratory_distress   = -1
    casualty_prediction.heart_rate             = -1
    casualty_prediction.respiratory_rate       = -1
    casualty_prediction.trauma_head            = 0
    casualty_prediction.trauma_torso           = 0
    casualty_prediction.trauma_lower_ext       = 0
    casualty_prediction.trauma_upper_ext       = 0
    casualty_prediction.alertness_ocular       = -1
    casualty_prediction.alertness_verbal       = -1
    casualty_prediction.alertness_motor        = -1
    casualty_prediction.is_coherent            = True
    casualty_prediction.model                  = MODEL_NUMBER

    # looping over results form prediction and updating ROS message
    for result in results:
        for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
            # checking if the prediction has a confidence value above the set threshold
            if conf < CONFIDENCE_THRESHOLD:
                continue
            else:
                # checking afflication type and updating ROS message
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
                    print("---------------------------------------------------------------------")
                    system_print("\u001b[31mERROR - invalid affliction class\u001b[0m")

    # printing ROS message for reference
    print("\n---------------------------------------------------------------------")
    system_print("ROS message")
    print("---------------------------------------------------------------------")
    print(casualty_prediction)

    # Publishing ROS message
    publisher.publish(casualty_prediction)

# runs yoloV8 on an image published to the "usb_cam/image_raw"
def run_model_with_camera(raw_image):
    system_print("Received image")

    # creating bridge object
    bridge = CvBridge()

    # converting image to a cv image
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

    # converting cv image to numpy array
    np_image = np.array(cv_image)

    print("---------------------------------------------------------------------")
    system_print("Prediction results:")
    print("---------------------------------------------------------------------")

    # running model on the numpy array
    results = model.predict(np_image)

    # printing results of prediction as json for reference
    for result in results:
        print("\n+-------+---------------+")
        print("| class\t| Confidence\t|")
        print("+-------+---------------+")
        for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
            print("| " + str(int(cls)) + "\t| " + str(conf) + "\t|")
            print("+-------+---------------+")

    # publishing results to ROS topic
    publish_results(results)

# runs yoloV8 on an image specified by a path
def run_model_with_path():
    while True:
        # getting image path from user
        print("---------------------------------------------------------------------")
        system_print("Enter a path to an image:")
        print("---------------------------------------------------------------------")
        image_path = input("\u001b[34m-> \u001b[0m")

        # checking if the user chose to quit
        if image_path == 'q' or image_path == 'Q':
            print("---------------------------------------------------------------------")
            system_print("Exiting...")
            print("---------------------------------------------------------------------")
            exit(0)

        # opening image
        image = PImage.open(image_path)

        print("---------------------------------------------------------------------")
        system_print("Prediction results:")
        print("---------------------------------------------------------------------")

        # running model on image
        results = model.predict(source=image)

        # printing results of prediction as json for reference
        for result in results:
            print("\n+-------+---------------+")
            print("| class\t| Confidence\t|")
            print("+-------+---------------+")
            for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                print("| " + str(int(cls)) + "\t| " + str(conf) + "\t|")
                print("+-------+---------------+")

        # publishing results to ros topic
        publish_results(results)

# sets up the model based on the users mode choice
def setup_model(choice):
    # checking which mode to run the model in
    if choice == RUN_WITH_CAMERA:
        print("---------------------------------------------------------------------")
        system_print("Setting up model to run with camera...")

        # registering callback function
        system_print("Waiting for image...")
        rospy.Subscriber('picked_image', Image, run_model_with_camera)
    else:
        print("---------------------------------------------------------------------")
        system_print("Setting up model to run with path...")
        # running predictor on image paths
        run_model_with_path()
        return

    # running node until user terminates the process
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)

if __name__ == "__main__":

    if DEBUG:
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
                setup_model(RUN_WITH_CAMERA)
                break
            elif choice == 'b':
                setup_model(RUN_WITH_PATH)
                break
            elif choice == 'q' or choice == 'Q':
                print("---------------------------------------------------------------------")
                system_print("Exiting...")
                print("---------------------------------------------------------------------")
                exit(0)
            else:
                print("---------------------------------------------------------------------")
                system_print("\u001b[31mInvalid choice...\u001b[0m")
    else:
        setup_model(RUN_WITH_CAMERA)
