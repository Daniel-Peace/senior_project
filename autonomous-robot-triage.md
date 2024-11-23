---
layout: page
title: Autonomous Robot Triage
permalink: /projects/autonomous-robot-triage/
---

<style>
a.custom-link {
  color: #ed6a5a; /* Blue color */
  text-decoration: none; /* Remove underline */
  transition: color 0.3s ease;
}
a.custom-link:hover {
  color: #5ca4a9; /* Tomato red on hover */
  text-decoration: none; /* Underline on hover */
  transition: color 0.3s ease;
}
</style>

<img src="/senior_project/images/DTC-logo-icon.png" alt="DTC logo" style="display: block; margin: 0 auto;">

## Overview
This project was worked on for use by Coordinated Robotics. Coordinated Robotics is a robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomous drones that can perform triage to assist first responders in mass casualty situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to approach it to save the most lives.

<img src="/senior_project/images/bullwinkle.jpg" alt="bullwinkle" style="display: block; margin: 0 auto;border-radius: 10px;">

###### Image from [DARPA TRIAGE CHALLENGE](https://triagechallenge.darpa.mil)

With that in mind, this project focused on integrating AI and ML models and creating a pipeline for making predictions about a casualty, combining multiple predictions into one report, and submitting this report to a server to be reviewed. To implement this pipeline, the tasks mentioned above were broken up further into sub-tasks that could then be implemented using Python. To allow all of these separate programs to communicate, I used ROS's messaging system to perform IPC. The end result is a pipeline that allows other developers to easily integrate new ML and AI models into the pipeline, and have them automatically used in finalizing a report about a casualty with minimal configuration needed from the developer.

Alongside working on the pipeline I did also work on developing a computer vision model using YOLOv8 by Ultralytics. To accomplish this, a custom dataset was created from data gathered at a test competition along with some data provided by DARPA. This dataset could then be used to train YOLOv8 to make predictions on various injuries such as an injured arm or leg to name a couple.

<img src="/senior_project/images/team_photo.jpg" alt="team-photo" style="display: block; margin: 0 auto;border-radius: 10px;">

Upon completing the pipeline and AI, I was able to test them at the first of three competitions for DTC last October. Though there is room for improvement, as there always is, these two technologies were rather successful and helped our team in placing second overall and first for teams not funded by DARPA.

*Continue reading for a more detailed explination of this project*

## Implementation
### Technologies
To implement this project several technologies were used including:
- **[ROS](https://www.ros.org/){: .custom-link}** - ROS (Robotics Operating System) is a common technology in the world of robotics research. It provides a framework and backbone to build on and can connect all processes to create a cohesive system to run a robot.

- **[YoloV8](https://docs.ultralytics.com/models/yolov8/){: .custom-link}** - Created by Ultralytics, YOLOv8 is one of the leading computer vision AIs. It is a convolutional neural network which can be tailored to your specific needs through training.

- **[Python](https://www.python.org/){: .custom-link}** - When using ROS you have two main choices when it comes to which languages are supported: Python and C++. Since Python is common in data science, it was the natural choice for most of this project.

- **[labelImg](https://github.com/HumanSignal/labelImg){: .custom-link}** - This is a free open source program that allows you to label images and generate output files useful for training AI and ML models. This was ideal since custom datasets were used to tailor YOLOv8 to our specific needs.

### Approach
Though the goal of DTC is to have fully autonomous robots performing triage, to ease the teams into the challenge, DARPA allowed teams to have 5 operators controlling the drones for the first challenge. With that, however, they did prohibit the operators from directly telling the robots to perform their predictions. Also, the operators are not allowed to see any of the data pertaining to the casualties during the competition runs. This gave us some forced criteria when creating the pipeline for scanning and submitting reports about casualties.

The following cycle is the solution we arrived at:

<img src="/senior_project/images/program_loop.png" alt="program-loop" style="border-radius: 10px;">

1. **Waiting to assign AprilTag** - An operator drives a robot to a casualty’s AprilTag, which requires the operator to press the deadman trigger to allow the robot to move.

1. **Assigning AprilTag** - Once the robot is in a good position to scan the AprilTag the deadman trigger is released by the operator, implicitly telling the robot to start scanning the AprilTag. This starts a timer for 10 seconds which is visible to the operator through a UI.

1. **Waiting to scan for afflictions** - Once the AprilTag scanning is done, the operator then drives the robot to a location which provides a good angle to scan the casualty for injuries.

1. **Scanning for afflictions** - Again, once the robot is in a good position to scan, the deadman trigger is released by the operator. This again implicitly tells the robot to start scanning, but this time for afflictions.

1. **Finalizing Reports** - After completing the scan for injuries, all predictions from all AI and ML models running on the robot are gathered and combined into a single report. This is accomplished through weighted averages along with a weighted voting system.

1. **Publishing Reports** - Once a finalized report is created, the report is submitted to a HTTP server for review.

### Programmatic Implementation
With the overall approach determined, we can now look to implement this with actual code. To do this, the tasks required to complete each step of the program loop were broken up into separate scripts and programs, and connected using ROS's messaging system. The end result is the following structure:

<img src="/senior_project/images/program_structure.png" alt="program-loop" style="border-radius: 10px;">

#### Timer
As the name suggests, the timer node provides timer functionality. It provides a countdown timer whenever the user releases the deadman trigger. This informs the user that they are about to either complete an AprilTag scan, or a scan for afflictions. The node subscribes to `/button_status` to check the state of the deadman trigger. Depending on which timer is active, the node will publish the timer's status to one of the following topics:

- `/apriltag_countdown_status`
- `/apriltag_timer_status`
- `/prediction_countdown_status`
- `/prediction_timer_status`

These messages contain info about the timer such as the amount of time left as well as what state the timer is in such as started, finished, and canceled.

#### Scan and Assign AprilTag
<img src="/senior_project/images/manikin_with_apriltag.jpg" alt="Mankikin with AprilTag" style="display: block; margin: 0 auto; border-radius: 10px;">

This node is responsible for obtaining the AprilTag that corresponds to the casualty we want to scan. This is done by driving a robot close to the AprilTag and releasing the deadman trigger. This starts a 10 second timer. During that time, this node receives a list of tags being detected by ROS's provided AprilTag package. This list contains the tag ID along with information about where it is with respect to the camera that detected it. Using this distance information, the node is able to keep track of which tag is closest during the 10 second timer. Once the time is up the saved tag is sent in a ROS message using the
`/assigned_apriltag` topic to `finalize_predictions.py` for use in the final report.

#### Model Nodes
The nodes labeled Model 0 - 3 represent the various AI and ML models that make predictions about casualties. These models will largely be created by other team members who will then use the framework this pipeline provides for implementing their models on the robot. All models publish their predictions to the topic `/model_predictions`. DARPA defined the following categories for afflictions that a casualty can have:

- Critical
  - Severe Hemorrhage
  - Respiratory Distress
- Vitals
  - Heart Rate
  - Respiratory Rate
- Injury
  - Trauma Head
  - Trauma Torso
  - Trauma Lower Extremity
  - Trauma Upper Extremity
  - Alertness Ocular
  - Alertness Verbal
  - Alertness Motor

Each of these categories is then assigned an integer value to represent whether or not a casualty has a specific affliction and to what degree. To help consolidate all of these possible affliction values, models store their predictions in a custom class `casualty.py` which contains all relevant information about a casualty. Since some models will only be able to predict certain types of afflictions, they may assign -1 to any of the categories to inform `finalize_predictions.py` to skip that field for that model when creating the final report. All models should also subscribe to the `/prediction_timer_status` topic so that they can be informed when a prediction timer has started.

#### YOLOv8
Though the primary focus of this project was on creating the pipeline, I did also work on creating one of the above-mentioned models which focused on injury detection through computer vision using YOLOv8.

YOLOv8 is a convolutional neural network geared towards computer vision and object detection. Though there are pretrained weights that can be used for detecting various types of objects, triage tends to have less data available. With that, a custom dataset was created using provided videos from DARPA as well as data collected at a practice competition hosted in Georgia. The data provided and collected then needed to be broken up into images and cleaned up, keeping only the useful images.

Next, each image needed to be labeled so that YOLOv8 could be trained and identify what injury or affliction it should be finding in an image. Once this was done, the model could actually be trained on the data which results in a weight file that can be used for making predictions on future images.

The next step was implementing this model into the full pipeline. Since we want this model to make a prediction once the prediction timer starts, a helper node was created that keeps track of the current image being published by the robot's camera to the `/usb_cam/image_raw` topic. Once the timer starts, the current image is published to `/picked_image`. `yolov8.py`(the python script I wrote containing the YOLOv8 model) is subscribed to this topic and therefore receives the image and makes a prediction about what it sees in that image and publishes the results to `/model_predictions` to be used by `finalize_predictions.py`.

#### Finalize Predictions
This node handles most of the work for the pipeline. It receives all predictions from all models as well as all assigned AprilTags and timer messages. The timer messages are used to determine when the program should be expecting what data as well as when it should submit the finalized predictions and reset the data for the next prediction.

To minimize the amount of work for the integration of future AI and ML models, I created a JSON configuration file that this program pulls from to automate some aspects of integrating a new model. Below is the configuration file with two models defined in it:

```
{
   "models" : [
       {
           "name" : "Computer Vision",
           "weight": 1,
           "coherent_dependent" : false,
           "determines_coherent" : false
       },
       {
           "name" : "Auditory Model",
           "weight" : 3,
           "coherent_dependent" : true,
           "determines_coherent" : true
       }
   ]
}
```

To actually combine all of the predictions into a single report, a weighted voting system along with some weighted averages are used. The reason being is that if two models end up contradicting each other about what afflictions a person has, we need a way to determine which model to trust more. It was mentioned above that each of the affliction categories will be assigned an integer to represent if a casualty has an affliction and to what degree, there is a slight exception to this. While most of the affliction categories use values of 0 - 3 to represent various states of that type of affliction, the **heart rate** and **respiratory rate** categories actually store the rate value as an integer (e.g. 60 bpm). It is these two categories that use weighted averages. For all other categories, the weighted voting system is used. This works by keeping track of a sum. If a model thinks a casualty has a certain type of affliction, it adds its assigned weight to the sum. If it thinks it does not have that affliction, it subtracts its weight from the sum. After going through all of the models we check the sum. If it's greater than 0 we assume the casualty has that affliction, if it's equal to or less than 0 we assume the casualty does not have that affliction.

After going through each model for each category, the `publish_reports()` function is called from the `casualty.py` class. This function goes through each category and creates and publishes a report containing all required information that DARPA has described to one of the following topics:

- `/critical_report`
- `/vitals_report`
- `/injury_report`

From here `send_reports.py` takes over.

#### Send Reports
After assigning an AprilTag and making and finalizing predictions about a casualty, we need to actually send it to a server to be reviewed. This is accomplished by `send_reports.py`. For the competition, the server of interest is the scoring server where DARPA team members will review your submission and score you based on how accurately you predicted the casualties afflictions. In a real world situation, this server would most likely represent a base station of some kind where first responders could view the data sent by the robots and choose how best to respond. Either way, the process will be nearly the same.

This program subscribes to the following topics to receive reports about a casualty:

- `/critical_report`
- `/vitals_report`
- `/injury_report`

After receiving a report it can convert that data to a JSON format to be attached to a HTTP post request. This request is sent and the response checked and published to one of the following corresponding ROS topics:

- `/critical_response`
- `/vitals_response`
- `/injury_response`

This information can be used by a GUI to inform the operators if the report was successfully sent and received.

### System Performance
Though this competition will continue for several more years, we did get to test this pipeline out at the first of three competitions. Overall the pipeline worked great. We were able to integrate several AI and ML models and have their reports successfully combined into one final report and have this report sent off to the scoring server. There were no apparent errors or flaws for this first competition and our team was able to claim second place overall and first place among the teams not sponsored by DARPA.

Though the pipeline worked great, the computer vision model I developed still has room to improve. While the approach taken seems to be valid, it appears that we did not have sufficient data for the training of the model and as such our accuracy when predicting on subjects never seen before suffered a bit. I say the approach we took seems valid and promising because when using the 80 10 10 rule for training, the model's prediction accuracy when going through the test dataset was very high with an accuracy of 99.33%. This would seem to indicate that though the model may not be optimized yet, the approach taken could be useful in the future.

### Possible Future Improvements
As mentioned above, the Computer Vision model I developed has room to improve. I think there are several approaches that could be taken to improve its functionality. The first and easiest would be to simply increase the dataset size that it was trained on. The more casualties it has to learn from the higher chance it will have at predicting correctly on subjects it's never seen before. Another possible approach would be to break the problem down a bit further and have the model try to identify various limbs on a subject and further analyze those limbs for injuries. These are just a couple of the many approaches that could be taken to improve and solve this aspect of the competition.

As for the pipeline as a whole, while it did perform well, I think there are several places where it can be improved.

- **Finalizing Predictions:** While the current method of using weighted averages and a weighted voting system seems to work, a more nuanced approach may be possible and could be worth experimenting with. This is because in many situations some injuries may indicate a high probability of a person also having another injury. Things of this nature are not currently accounted for.

- **Configuration File:** Although there is some automation of adding new models to the pipeline,
there is definitely still room to make it even easier. I would like to expand the configuration
file to incorporate more details about a model that is being added so that the developer of that
model has less work to do when implementing the system.

- **Restructuring:** While the project is well-organized overall, I feel that there could be some improvements made to the overall structure of the program allowing it to become more modular and dynamic. This would be useful should future developers want to pull specific data out or plug other programs and features in. This would provide a slightly more future-proof setup.

### Acknowledgments
I want to thank the following people who greatly helped me in this project:

- **[Professor Jason Isaacs:](http://jtisaacs.com/){: .custom-link}** Aside from answering many many questions, Professor Isaacs helped guide me as I worked through this project pointing out possible areas to consider and paths I should probably take. He was a great help in getting started in not only ROS, but the world of AI and ML.

- **Kevin Knoedler:** Having never worked with ROS prior to this project, Kevin was a great help and allowed me to inundate him with questions about ROS and everything robotics to learn everything I could and needed in order to complete this project.

- **[DARPA:](https://www.darpa.mil/){: .custom-link}** Thank you to DAPRA for organizing this competition and providing data and locations for testing our robots.

---

##### *Click **[here](https://github.com/Daniel-Peace/senior_project){: .custom-link}** to view project repository*
