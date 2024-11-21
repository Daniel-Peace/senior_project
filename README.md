## Overview
This project was worked on for use by Coordinated Robotics. Coordinated Robotics is a robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomous drones that can perform triage to assist first responders in mass casualty situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to approach it to save the most lives.

![rock-crawler-drone](./photos/bullwinkle.jpg)

###### Image from [DARPA TRIAGE CHALLENGE](https://triagechallenge.darpa.mil)


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
I want to give a breif description of what each node and class does, and how it may be configured:

### yolov8.py
This program implements [Ultralytics' YOLOv8](https://docs.ultralytics.com/models/yolov8/) to make predictions about possible affliction
someone may have. It provides the options of running the model using a path provided by the
user or by using images published to the `/picked_image` topic. The results of the model
prediction are stored in a ROS message of type `Casualty_prediction` and published to the
`model_predictions` topic.

The ROS message `Casualty_prediction` initializes to all 0s for all affliction values
With that, when the model can make predictions about an affliction but
does not, it should be assumed that the casualty does not have that
specific affliction and the field in the ROS message should be left as zero.

The model pulls the weights from a folder in the src directory named `weights`.
You can change the weights being used by changing the `WEIGHTS` constant to a path of
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

### timer.py
This program handles timers surrounding AprilTag detections and affliction predictions.
It cycles through the following timers:
- A countdown timer for when you are about to scan for an AprilTag
- A timer for scanning the AprilTag
- A countdown timer for when you are about to scan for afflictions
- A timer for scanning for afflictions

It publishing `Timer_state` ROS messages, which contain a timer's current status along with
the amount of time left, to:
- `/apriltag_countdown_timer_state`
- `/apriltag_scanning_timer_state`
- `/prediction_countdown_timer_state`
- `/prediction_scanning_timer_state`

It triggers these timers by detecting if a trigger on a controller is not being pressed.
It receives info about the triggers state from the `/button_status` topic.

The length of the timers can be changed using the following constants:
- `BUTTON_TIMER_LENGTH`
- `APRILTAG_TIMER_LENGTH`
- `PREDICTION_TIMER_LENGTH`

You will notice a few additional topics that this program publishes to. These are for use
by the GUI to allow some of its elements to update. These topics include:
- `/current_timer` (used to indicate which timer is active)
- `/loop_state` (used to determine which part of the pipeline loop we are in)

### send_report.py
This program is responsible for submitting a finalized prediction to the scoring
server using API endpoints provided by DARPA. It creates callback functions for
each report type and submits all ROS messages received from those topics to their
corresponding endpoint.
These callback functions check the following topics for reports:
- `/injury_report`
- `/critical_report`
- `/vitals_report`

It also periodically sends a request
for a status update. The responses from each HTTP request is published to a corresponding
ROS topic:
- `/critical_response`
- `/vitals_response`
- `/injury_response`
- `/status`

When sending requests to the server, DARPA is using a bearer token to verify who is
connecting. This must be included in the header of the request and can be changed by adding
in a another constant for the new token and then setting the `ACTIVE_TOKEN` equal to your new
token constant.

The same approach can be taken should you need to update the IP address of the server.

This program will also publish a `Response_statuses.msg` after each attempt to submit a
a report. This is currently only used by the GUI to inform the user whether or not all
reports were successfully submitted. They are published to the `/response_statuses` topic


### pick_image.py
This program keeps track of the current image published to the `/usb_cam/image_raw` topic.
When a `timer_status` message of `True` is received from the `/prediction_scanning_timer_state` topic,
it publishes the current image to the `picked_image` topic.

### model.py
This class contains members and functions for holding and working with information about the
integrated AI and ML models. When the pipeline is started, this class is used to store the
configuration data for each model in `model_configs.json`.

### finalize_predictions.py
This program is responsible for receiving all predictions from all models and creating
a finalized prediction about a casualty. It also receives the assigned AprilTag and
adds this to the final prediction. To accomplish this the program waits for a prediction
timer to start. Once a timer has started the program resets all variables and objects
in preparation for receiving predictions and the assigned AprilTag. Anytime during the
timer, the program can receive predictions and update each model object.
Once the timer has ended. The model weights 2 seconds both for final predictions to come
in, and for an apriltag to be assigned. After this, it and moves on to
finalizing all of the predictions. Upon completing this process, it calls the
publish_reports method on finalized_casualty which creates reports for each affliction
type and publishes them to their respective topics.

During the finalzation process this program checks the settings of each model to
detemine which model predicts which afflictions.

For all alertness affliction types there is currently only one model for each
type. This program therefore assigns those models predictions directly to the final
report rather than using a voting system of anykind.

For affliction categories using a voting system, or a weighted average, the weights are pulled
from `model_configs.json`

If any of the afflication categories fails to find a model that made a prediction for
said category, the model assigns whichever value would be considered "normal"
by default. For both heart rate and respiratory rate, these values can be modified using
the `DEFAULT_HR` and `DEFAULT_RR` constants.

The timout timers length can be adjusted using the `PREDICTION_TIMEOUT` and `APRILTAG_TIMEOUT`
constants.

In the event that an AprilTag is not received, it will assign -1 to the report and leave
it to `send_report.py` to handle.

### casualty.py
This class provides members for storing all data partaining to a casualty. It also
provides several methods for working on an instance of this class. You can change
which system and team name is used with the reports by changing the constants `TEAM_NAME`
and `SYSTEM` at the top of the program.

### assign_apriltag.py
This program tracks which AprilTag is clostest to camera and publishes the tag to `/assigned_apriltag` once
the AprilTag timer has started. Once the timer finishes, it stops publishing the Apriltag
and resets the program for the next apriltag scan. If the timer is canceled, it will also
reset the program. This program subscribes to `/tag_detections` which is provided by
