# imports
import numpy as np
import time
import rospy
import json
from messages.msg       import Casualty_prediction
from PIL                import Image as PImage
from ultralytics        import YOLO
from sensor_msgs.msg    import Image
from cv_bridge          import CvBridge

# general constants
RUN_WITH_CAMERA         = 0
RUN_WITH_PATH           = 1
CONFIDENCE_THRESHOLD    = 0
MODEL_NUMBER            = 0

# afflication types
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
model = YOLO("./yoloV8_weights/iteration_2.pt")

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# publishes the results of a prediction to "model_0_predictions"
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
    print("---------------------------------------------------------------------")
    system_print("ROS message")
    print("---------------------------------------------------------------------")
    print(casualty_prediction)

    # Publishing ROS message
    publisher.publish(casualty_prediction)

# runs yoloV8 on an image published to the "usb_cam/image_raw"
def run_predictor_with_camera(raw_image):
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

    # publishing results to ROS topic
    publish_results(results)

# runs yoloV8 on an image specified by a path
def run_predictor_with_path():
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

        for result in results:
            combined_data = [{"affliction_class": int(cls), "confidence": float(conf)} for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy())]
            json.dumps(combined_data)
            print(combined_data)
            print("---------------------------------------------------------------------")

        # publishing results to ros topic
        publish_results(results)

# sets up the predictor based on the users mode choice
def setup_predictor(choice):
    # checking which mode to run the predictor in
    if choice == RUN_WITH_CAMERA:
        print("---------------------------------------------------------------------")
        system_print("Setting up model to run with camera...")

        # registering callback function
        system_print("Waiting for image...")
        rospy.Subscriber('picked_image', Image, run_predictor_with_camera)
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
        else:
            print("---------------------------------------------------------------------")
            system_print("\u001b[31mInvalid choice...\u001b[0m")
