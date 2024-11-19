
figcaption {
  font-size: 0.9em;
  text-align: center;
  color: gray;
  margin-top: 5px;
}


## Overview
This project was worked on for use by Coordinated Robotics. Coordinated Robotics is a robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomous drones that can perform triage to assist first responders in mass casualty situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to approach it to save the most lives.

![rock-crawler-drone](./photos/bullwinkle.jpg)
<figcaption>
  Image from <a https://triagechallenge.darpa.mil="_blank">DARPA TRIAGE CHALLENGE</a>, used under fair use.
</figcaption>

With that in mind, this project focused on integrating AI and ML models and creating a pipeline for making predictions about a casualty, combining multiple predictions into one report, and submitting this report to a server to be reviewed. To implement this pipeline, the tasks mentioned above were broken up further into sub-tasks that could then be implemented using Python. To allow all of these separate programs to communicate, I used ROS's messaging system to perform IPC. The end result is a pipeline that allows other developers to easily integrate new ML and AI models into the pipeline, and have them automatically used in finalizing a report about a casualty with minimal configuration needed from the developer.

Alongside working on the pipeline I did also work on developing a computer vision model using YoloV8 by Ultralytics. To accomplish this, a custom dataset was created from data gathered at a test competition along with some data provided by DARPA. This dataset could then be used to train YoloV8 to make predictions on various injuries such as an injured arm or leg to name a couple.

![team-photo](./photos/team_photo.jpg)

Upon completing the pipeline and AI, I was able to test them at the first of three competitions for DTC last October. Though there is room for improvement, as there always is, these two technologies were rather successful and helped our team in placing second overall and first for teams not funded by DARPA.


## Prerequisites

### Software
Before getting started, you will want to make sure you ahve the following installed:
- Ubuntu 1.20.4
- ROS Noetic
  - May need to install [`apriltag_ros`](https://wiki.ros.org/apriltag_ros) package
  - May need to install [`usb_cam`](https://wiki.ros.org/usb_cam) package
  - May need to install [`joy`](https://wiki.ros.org/joy) package
- Python3
- YOLOv8 by Ultralytics
- CUDA for NVDIA graphicscards
- PyQt5 (if you wish to use the GUI)

### Hardware
Though they are not strictly required, the following hardware is helpful should you want to use certain aspects of this project:
- Webcam
- Game controller (Xbox 360 is known to be supported)

## Getting started:
Before we dive into starting up the pipeline and computer vision model
I want to give a breif description of each node and what it does:

### yolov8.py
This program implements [Ultralytics' YOLOv8](https://docs.ultralytics.com/models/yolov8/) to make predictions about possible affliction
someone may have. It provides the options of running the model using a path provided by the
user or by using images published to the `/picked_image` topic. The results of the model
prediction are stored in a ROS message of type `Casualty_prediction` and published to the
`model_predictions` topic.

The ROS message "Casualty_prediction" initializes to all 0s for all affliction values
With that, when the model can make predictions about an affliction but
does not, it should be assumed that the casualty does not have that
specific affliction and the field in the ROS message should be left as zero.

The model pulls the weights from a folder in the src directory named "weights".
You can change the weights being used by changing the WEIGHTS constant to a path of
your choosing.

The confidence value threshold for what predictions are published can be set with the
constant `CONFIDENCE_THRESHOLD`. Any predictions that have a confidence value below this
threshold will be ignored.

You may choose if you would like to pass in a path to an image or have the program pull
images from the `/picked_image` topic by using the DEBUG flag. If this is set to true, you
will have the choice to run the program with either a path to an image or using images
published to the `/picked_image` topic. If `DEBUG` is set to false the program will run with
the camera by default.

Affliction types that can be predicted:
- trauma_head
- trauma_torso
- trauma_lower_ext
- amputation_lower_ext
- trauma_upper_ext
- amputation_upper_ext
- severe_hemorrhage

ROS topic subscriptions:
- `/picked_image`

ROS topics for publishing
- `/model_predictions`

### button_press.py
This program also contains a constant, `DEBUG`, which when set to `True` allows you to
enter button presses using the terminal rather than using a controller. Change this
depending on whether or not you wish to use a controller.


### casualty.py
This file contains two constants, `TEAM_NAME` and `SYSTEM`.

`TEAM_NAME` should be set to whatever your team name is at a competition.
This will be used when creating reports.

`SYSTEM` represent a drones name and should be set to the name
of the robot this code is running on.
