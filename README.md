
## Overview
This project was worked on for use by Coordinated Robotics. Coordinated Robotics is a robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomous drones that can perform triage to assist first responders in mass casualty situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to approach it to save the most lives.

![rock-crawler-drone](./photos/bullwinkle.jpg)

With that in mind, this project focused on integrating AI and ML models and creating a pipeline for making predictions about a casualty, combining multiple predictions into one report, and submitting this report to a server to be reviewed. To implement this pipeline, the tasks mentioned above were broken up further into sub-tasks that could then be implemented using Python. To allow all of these separate programs to communicate, I used ROS's messaging system to perform IPC. The end result is a pipeline that allows other developers to easily integrate new ML and AI models into the pipeline, and have them automatically used in finalizing a report about a casualty with minimal configuration needed from the developer.

Alongside working on the pipeline I did also work on developing a computer vision model using YoloV8 by Ultralytics. To accomplish this, a custom dataset was created from data gathered at a test competition along with some data provided by DARPA. This dataset could then be used to train YoloV8 to make predictions on various injuries such as an injured arm or leg to name a couple.

![team-photo](./photos/team_photo.jpg)

Upon completing the pipeline and AI, I was able to test them at the first of three competitions for DTC last October. Though there is room for improvement, as there always is, these two technologies were rather successful and helped our team in placing second overall and first for teams not funded by DARPA.


## Prerequisites

### Software
Before getting started, you will want to make sure you ahve the following installed:
- Ubuntu 1.20.4
- ROS Noetic
  - May need to install AprilTag Package seperately
  - May need to install USB camera node
  - May need to isntall joystick node
- Python3
- YOLOv8 by Ultralytics
- CUDA for NVDIA graphicscards
- PyQt5 (if you wish to use the GUI)

### Hardware
Though not strictly required teh following is helpful should you want to use certain aspects of this project:
- Webcam
- Game controller (Xbox 360 is known to be supported)

## Getting started:
Before starting all of the ros nodes and using the pipeline there are a few things to check and configure:

### yolov8.py
There are a couple of things to check in the `yolov8.py` file.
First there is a constant `DEBUG`. When this is set to `True` the
program runs in debug mode, allowing you to choose whether you
want to pass in a path to an image or run YOLOv8 using an image from
the USB camera. If set to `False` the program defaults to using the USB
camera.

There is also a constant `WEIGHTS` which be set to the path to the
weights you wish to use while making predictions.

Lastly there is a constant, `CONFIDENCE_THRESHOLD`, which should be set
to a float value from 0 to 1 inclusive. This value tells the program what
the threshold confidence value should be to accept a prediction. As an example,
if the value was set to 0.63, only prediction with a confidence of
0.63 and above would be accepted

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
