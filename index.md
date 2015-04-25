---
layout: project
title: deixis
subtitle: Modeling the human perception of robotic deictic gestures.
---

<img src="http://niftyhedgehog.com/deixis/images/deixis_thumbnail.jpg">

## Overview
* What is a deictic gesture, behavior?
Deixis is a human-robot interaction (HRI) experiment that is a system for situated head and arm motion specification playback for modelling the human perception of robot deictic gestures. The experiment incorporates the "Bandit II" as the gesturing robot, the SICK LMS for finding markers, Point Grey Firefly MV Cameras for head-tracking with the ARToolKit, and a Nintendo Wiimote for facilitating the procedure.

Modeled the human perception of robotic deictic gestures—multi-modal nonverbal communication—with salient objects to explore human-robot interaction. Utilized ROS to handle an experiment involving the Bandit II robot, marker pointers, SICK-LMS laser range finder, overhead and forward-facing cameras, head tracking, Wiimote controller, OpenCV, ARToolkit+. Ran experiments on 15+ human test subjects.

* Calibration and capture from cameras in a smartroom environment
* Modeling Non-verbal Communication in Human-Robot Task Collaborations
* Investigating Robot Deictic Behaviors

Practical models for gesture formulation in collocated task environments
This project was developed in the summer of 2010 during an NSF research fellowship under the direction of Ph.D candidates, Aaron St. Clair and Ross Mead, and Dr. Maja Matarić. 

## Hardware
* [Bandit II](http://robotics.usc.edu/interaction/?l=Laboratory:Facilities#BanditII), an expressive humanoid robot
* [SICK LMS200](https://mysick.com/PDF/Create.aspx?ProductID=33755&Culture=en-US) laser range finder
* [Point Grey Firefly MV](http://www.ptgrey.com/firefly-mv-usb2-cameras) cameras for overhead tracking
* Nintendo [Wiimote](http://en.wikipedia.org/wiki/Wii_Remote) controller for wireless facilitation of the experiment
* Plexiglass screen with salient points


## Software
* ROS
* R plots

### Wiimote
I programmed the Wiimote controller using the ROS wiimote package, which allows communication with ROS nodes through the ROS framework. Connecting through Bluetooth, the /joy node returns button, LED, rumble, and accelerometer/gyro data. My program uses this data, along with sensor input from the SICK Laser Measurement Sensor, to save waypoint coordinates for the Deixis Experiment (see above).

### Camera Calibration
Started working on stereo camera calibration for Bandit. The stereo camera setup simulates human binocular vision, thus having the ability to capture three-dimensional images, such as me sitting at my desk.

### Bag Files
The bag2video package is a utility that will consume a bag file of compressed images, and return a .avi file (hence the name, "bag2video"). It uses image_transport for publishing/subscribing to images between ROS nodes, and CvBridge to convert between ROS image messages and OpenCV IPLimages--all part of ROS. The videos I converted use ARTags, which are used for tracking head orientation through augmented reality. 
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
