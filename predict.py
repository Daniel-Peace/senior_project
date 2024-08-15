#!/usr/bin/env python3

# imports
from PIL import Image as PImage
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import json
import rospy
from messages.msg import Prediction
from messages.msg import Prediction_element

# constants
RUN_WITH_CAMERA = 0
RUN_WITH_PATH   = 1

# creating publisher
publisher = rospy.Publisher('yoloV8_prediction', Prediction, queue_size=10)

# initializing yoloV8 model
model = YOLO("./yoloV8_weights/best.pt")

# creating bridge object
bridge = CvBridge()

# publishes the results of a prediction to "yoloV8_prediction"
def publish_results(results):
    # formatting results into a json
    for result in results:
        combined_data = [{"class": int(cls), "confidence": float(conf)} for cls, conf in zip(result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy())]
        json.dumps(combined_data)
        print(combined_data)

    # iterating over predictions to create ROS message
    prediction = Prediction()
    for item in combined_data:
        prediction_element = Prediction_element()
        prediction_element.injury_class = item['class']
        prediction_element.confidence = item['confidence']
        prediction.prediction_elements.append(prediction_element)

    print(prediction)

    # Publishing ROS message
    publisher.publish(prediction)



# runs yoloV8 on an image published to the "usb_cam/image_raw"
def run_predictor_with_camera(raw_image):

    # converting image to a cv image
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

    # converting cv image to numpy array
    np_image = np.array(cv_image)

    # running model on the numpy array
    results = model.predict(np_image)

    # publishing resutls to ros topic
    publish_results(results)



# runs yoloV8 on an image specified by a path to an image
def run_predictor_with_path():
    while True:
        # getting image path from user
        image_path = input("Enter a path to an image:\n\u001b[34m-> \u001b[0m")

        # check if the user chose to quit
        if image_path == 'q' or image_path == 'Q':
            print("Exiting...")
            break

        # opening image
        image = PImage.open(image_path)

        # running model on image
        results = model.predict(source=image)

        # publishing resutls to ros topic
        publish_results(results)



# sets up the predictor based on the users mode choice
def setup_predictor(choice):

    # creating ROS node
    rospy.init_node('yoloV8_node', anonymous=True)

    print("--------------------------------------------------------------------------")

    # checking which mode to run the predictor in
    if choice == RUN_WITH_CAMERA:
        # registering callback functions
        rospy.Subscriber('usb_cam/image_raw', Image, run_predictor_with_camera)
    else:
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
    print("Choose how you would like to run this program\na. Run using usb_camera node\nb. Run using a path to an image")

    # looping unitl a valid choice is entered or the program is quit
    while True:
        # prompting user
        choice = input("\u001b[34m-> \u001b[0m")

        # checking user's choice
        if choice == 'a':
            setup_predictor(RUN_WITH_CAMERA)
            break
        elif choice == 'b':
            setup_predictor(RUN_WITH_PATH)
            break
        elif choice == 'q' or choice == 'Q':
            print("Exiting...")
            break
        else:
            print("\u001b[34m-> \u001b[0m \u001b[31mInvalid choice...\u001b[0m")