
# Overview
This project was worked on for use by Coordinated Robotics. Coordinated Robotics is a robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomous drones that can perform triage to assist first responders in mass casualty situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to approach it to save the most lives.

![rock-crawler-drone](./photos/bullwinkle.jpg)

With that in mind, this project focused on integrating AI and ML models and creating a pipeline for making predictions about a casualty, combining multiple predictions into one report, and submitting this report to a server to be reviewed. To implement this pipeline, the tasks mentioned above were broken up further into sub-tasks that could then be implemented using Python. To allow all of these separate programs to communicate, I used ROS's messaging system to perform IPC. The end result is a pipeline that allows other developers to easily integrate new ML and AI models into the pipeline, and have them automatically used in finalizing a report about a casualty with minimal configuration needed from the developer.

Alongside working on the pipeline I did also work on developing a computer vision model using YoloV8 by Ultralytics. To accomplish this, a custom dataset was created from data gathered at a test competition along with some data provided by DARPA. This dataset could then be used to train YoloV8 to make predictions on various injuries such as an injured arm or leg to name a couple.

![team-photo](./photos/team_photo.jpg)

Upon completing the pipeline and AI, I was able to test them at the first of three competitions for DTC last October. Though there is room for improvement, as there always is, these two technologies were rather successful and helped our team in placing second overall and first for teams not funded by DARPA.
