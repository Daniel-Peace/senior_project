from ultralytics import YOLO
# from PIL import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import json
import rospy
from messages.msg import Prediction
from messages.msg import Prediction_element

# creating publisher
publisher = rospy.Publisher('yoloV8_prediction', Prediction, queue_size=10)

# initializing yoloV8 model
model = YOLO("./yoloV8_weights/best.pt")

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
def run_predictor(raw_image):

    # converting image to a cv image
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

    # converting cv image to numpy array
    np_image = np.array(cv_image)

    # # getting image path from user
    # image_path = input("Enter a path to an image:\n")

    # # opening image
    # image = Image.open(image_path)

    # # running model on image
    # results = model.predict(source=image)

    # running model on the numpy array
    results = model.predict(np_image)

    publish_results(results)

def setup_predictor():
    # creating ROS node
    rospy.init_node('yoloV8_node', anonymous=True)

    # registering callback functions
    rospy.Subscriber('usb_cam/image_raw', Image, run_predictor)

    # running node until user terminates the process
    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)

if __name__ == "__main__":
    setup_predictor()