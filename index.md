---
layout: project
title: deixis
subtitle: Modeling the human perception of robotic deictic gestures.
---

<img src="images/deixis_thumbnail.jpg">

## Overview
Deixis is an HRI experiment that was developed by Ph.D candidates, Aaron St. Clair and Ross Mead. It is a system for situated head and arm motion specification playback for modeling human perception of robot deictic gestures. The experiment incorporates the Bandit-II as the gesturing robot, the SICK LMS for finding markers, Point Grey Firefly MV Cameras for head-tracking with the ARToolKit, and the Nintendo Wiimote for facilitating the procedure.

Modeled the human perception of robotic deictic gestures—multi-modal nonverbal communication—with salient objects to explore human-robot interaction. Utilized ROS to handle an experiment involving the Bandit II robot, marker pointers, SICK-LMS laser range finder, overhead and forward-facing cameras, head tracking, Wiimote controller, OpenCV, ARToolkit+. Ran experiments on 15+ human test subjects.

* What is a deictic gesture, behavior?
* Calibration and capture from cameras in a smartroom environment
* Modeling Non-verbal Communication in Human-Robot Task Collaborations
* Investigating Robot Deictic Behaviors

Practical models for gesture formulation in collocated task environments
In many collocated human-robot interaction scenarios, robots are required to accurately and unambiguously indicate an object or point of interest in the environment. Realistic, cluttered environments containing many visually salient targets can present a challenge for the observer of such pointing behavior. In this paper, we describe an experiment and results detailing the effects of visual saliency and pointing modality on human perceptual accuracy of a robot’s deictic gestures (head and arm pointing) and compare the results to the perception of human pointing.
This project was developed in summer 2010 during an NSF research fellowship under the direction of Aaron St. Clair and Dr. Maja Matarić. 

## Hardware
* Bandit robots
* Wiimote
* SICK laser range finder
* Cameras
* Overhead cameras

## Software
* ROS
* R plots

I programmed the Wiimote controller using the ROS wiimote package, which allows communication with ROS nodes through the ROS framework. Connecting through Bluetooth, the /joy node returns button, LED, rumble, and accelerometer/gyro data. My program uses this data, along with sensor input from the SICK Laser Measurement Sensor, to save waypoint coordinates for the Deixis Experiment (see above).

Started working on stereo camera calibration for Bandit. The stereo camera setup simulates human binocular vision, thus having the ability to capture three-dimensional images, such as me sitting at my desk.

The bag2video pacakge is a utilty that will consume a bag file of compressed images, and return a .avi file (hence the name, "bag2video"). It uses image_transport for publishing/subscribing to images between ROS nodes, and CvBridge to convert between ROS image messages and OpenCV IPLimages--all part of ROS. The videos I converted use ARTags, which are used for tracking head orientation through augmented reality. 
bag2video can also create a video file at a variable framerate, given a directory of .jpg images.

## Experiment
* Paid human test subjects $10 for an hour of their time to point
* Green laser pointers
* Pringles can

Software architecture for deictic gesture validation and validation user-studies
Developing an empirical understanding of people’s perception of robot deixis

## Results
Aaron's publications

## Misc.
* [Interaction Lab Webpage](http://robotics.usc.edu/~hieu/)
* <span class="author">Aaron B. St. Clair, Ross Mead and Maja J. Matarić</span>. "<span class="title">Investigating the Effects of Visual Saliency on Deictic Gesture Production by a Humanoid Robot</span>". <span class="pub_status">In</span> <span class="booktitle">IEEE 20th IEEE International Symposium in Robot and Human Interactive Communication</span> <span class="booktitle">(Ro-Man 2011)</span>, <span class="address">Atlanta, GA</span>, <span class="month">Jul</span> <span class="year">2011</span>
* My experiments contributed to this [paper](http://robotics.usc.edu/publications/media/uploads/pubs/731.pdf)
