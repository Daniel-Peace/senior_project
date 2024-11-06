---
layout: page
title: AI/ML Pipeline
permalink: /projects/capstone/
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
This project was worked on for use by Coordianted Robotics. Coordianted Robotics is robotics team that works with students from CSUCI and is currently competing in DARPA's latest competition. This competition, DTC (DARPA Triage Challenge), is focused on the development of autonomus drones that can perform triage to assist first responders in mass casuatly situations. The goal is to use drones to provide data to first responders that will help them assess the situation and determine how best to apprach it to save the most lives.

With that in mind, this project focused on integrating AI/ML models and creating a pipline for making predictions about a casualty, combining multiple predictions into one report, and submitting this report to a server to be viewed.

## Implementation
### Technologies
To implement this project several technologies were used including:
- **[ROS](https://www.ros.org/){: .custom-link}** - ROS (Robotics Operating System) is a common technology in the world of robotics research. It provides a framework and backbone to build on and can connect all processes to create a cohesive system to run a robot.

- **[YoloV8](https://docs.ultralytics.com/models/yolov8/){: .custom-link}** - Created by Ultralytics, yoloV8 is one of the leading computer vision. It is a convolutional neural network which can be tailored to your specific needs through training.

- **[Python](https://www.python.org/){: .custom-link}** - When using ROS you have two main choices when it comes to which languages are supported: Python and C++. Since Python is common in data science, it was the natural choice for most of this project.

- **[labelImg](https://github.com/HumanSignal/labelImg){: .custom-link}** - This is a free open source program that allows you to label images and generate output files useful for training AI and ML models. This was ideal since custom datasets were used to tailor YoloV8 to our specific needs.

### Approach
Though the goal of DTC is to have fully autonomous robots performing triage, to ease the teams into the challenge and to not ask too much at once, DARPA allowed teams to have 5 operators controlling the drones for the first challenge. With that, however, they did prohibit the operators from directly telling the robots to perform its predictions. Also, the operators are not allowed to see any of the data pertaining to the casualties during the competition runs. This gave us some forced criteria when creating the pipeline for scanning and submitting reports about casualties.

The following cycle is the solution we arrived at:

<img src="/senior_project/images/program_loop.png" alt="program-loop" style="border-radius: 10px;">

1. **Waiting to assign AprilTag** - An operator drives a robot to a casualty’s AprilTag, which requires the operator to press the deadman trigger to allow the robot to move.

1. **Assigning AprilTag** - Once the robot is in a good position to scan the AprilTag the deadman trigger is released by the operator, implicitly telling the robot to start scanning the apriltag and a timer. This starts a timer for 10 seconds which is visible to the operator.

1. **Waiting to scan for afflications** - Once the Apriltag scanning is done the operator then drives the robot to a location which provides a good angle to scan the casualty for injuries.

1. **Scanning for affliction** - Again, once the robot is in a good position to scan the deadman trigger is released by the operator. This again implicitly tells the robot to start scanning, but this time for injuries.

1. **Finalizing Reports** - Publishing Reports - After completing the scan for injuries, all predictions from all AI and ML models running on the robot are gathered and combined into a single report.

1. **Publishing Reports** - Once a finalized report is created, the report is submitted to a server for review.