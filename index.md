---
layout: project
title: deixis
subtitle: Modeling the human perception of robotic deictic gestures.
---

<img src="http://niftyhedgehog.com/deixis/images/bandit_point4.jpg">

## Overview
Deixis was a human-robot interaction (HRI) study, which incorporated human trials to investigate the perception of robotic deictic gestures. A "deictic gesture" is a form of nonverbal communication that typically serves to focus an observer's attention to a specific object or location in the environment. For example, humans will often employ nonverbal modalities such as facial expressions, eye gaze, head orientation, and arm gestures to signal intent "here" or "there".

When humans and robots are involved in a collaborative task in a shared environment, deictic gestures are essential for sustaining effective communication and natural interactions&mdash;without a reliance on speech. Deixis is an experimental study of human perception of a robot’s deictic gestures under a set of different environmental visual saliency conditions and pointing modalities using an upper-torso humanoid robot. It attempts to gain an empirical understanding of how to map well-studied human gestures to robots of varying capabilities and embodiments.

This project was developed in the summer of 2010 during an NSF research fellowship under the direction of Ph.D candidates, Aaron St. Clair and Ross Mead, and Dr. Maja Matarić. 


## Hardware
The Deixis experiment utilized "Bandit" as the gesturing robot, the SICK LMS for finding markers, overhead and forward-facing cameras for head-tracking, and a Wiimote for facilitating the experiment.

* [Bandit II](http://robotics.usc.edu/interaction/?l=Laboratory:Facilities#BanditII), an expressive upper-torso humanoid robot
* [SICK LMS200](https://mysick.com/PDF/Create.aspx?ProductID=33755&Culture=en-US) laser range finder
* [Point Grey Firefly MV](http://www.ptgrey.com/firefly-mv-usb2-cameras) cameras for overhead tracking
* Nintendo [Wiimote](http://en.wikipedia.org/wiki/Wii_Remote) controller for wireless facilitation of the experiment

## Software
The software aspect of this project required several several sub-projects to The Robot Operating System (ROS) framework 

* [ROS](http://www.ros.org/) for robot control, marker finding, data collection, and Wiimote integration
* [OpenCV](http://opencv.org/) for camera image manipulation
* [ARToolKit](http://artoolkit.sourceforge.net/) for head tracking
* [R](http://www.r-project.org/) to generate statistical plots


### Wiimote
I programmed the Wiimote controller using the ROS wiimote package, which allows communication with ROS nodes through the ROS framework. Connecting through Bluetooth, the /joy node returns button, LED, rumble, and accelerometer/gyro data. My program uses this data, along with sensor input from the SICK Laser Measurement Sensor, to save waypoint coordinates for the Deixis Experiment (see above).

### Camera Calibration
Started working on stereo camera calibration for Bandit. The stereo camera setup simulates human binocular vision, thus having the ability to capture three-dimensional images, such as me sitting at my desk.
overhead and forward-facing cameras,

<img src="http://niftyhedgehog.com/deixis/images/stereo_left.png" width="300">
<img src="http://niftyhedgehog.com/deixis/images/stereo_right.png" width="300">

<img src="http://niftyhedgehog.com/deixis/images/stereo_rviz.png">

<img src="http://niftyhedgehog.com/deixis/images/stereo_rviz2.png">


### Bag Files
The bag2video package is a utility that will consume a bag file of compressed images, and return a .avi file (hence the name, "bag2video"). It uses image_transport for publishing/subscribing to images between ROS nodes, and CvBridge to convert between ROS image messages and OpenCV IPLimages--all part of ROS. The videos I converted use ARTags, which are used for tracking head orientation through augmented reality. 
bag2video can also create a video file at a variable framerate, given a directory of .jpg images.

### Marker Finder
<img src="http://niftyhedgehog.com/deixis/images/laser_ss.png">

### Head Tracking


## Experiment
An experiment and results detailing the effects of visual saliency and pointing modality on human perceptual accuracy of a robot’s deictic gestures (head and arm pointing) and compare the results to the perception of human pointing.
Realistic, cluttered environments containing many visually salient targets can present a challenge for the observer of such pointing behavior. 

conducted an initial pilot study with our upper-torso humanoid robot, Bandit (Figure 1) sitting faceto- face with participants and gesturing to locations on a transparent screen between them. We varied distance and angle to target, distance and angle to viewer, and pointing modality,We conducted a factorized experiment over three robot pointing modalities: the head with 2 degrees-of-freedom (DOF), the arm with 7 DOF, and both together (i.e., head+arm) with two saliency conditions: a blank (or nonsalient) environment and an environment with several highly and equally visually salient targets. 
salient objects would affect people’s interpretations of the points. Specifically, we anticipated that people would “snap to” the salient objects, thus reducing error for points whose targets were on or near markers, whereas in the non-salient condition there were no points of reference to bias estimates

* Paid human test subjects $10 for an hour of their time to point
* Green laser pointers
* Pringles can

Software architecture for deictic gesture validation and validation user-studies
Developing an empirical understanding of people’s perception of robot deixis 

In the experiments, the participant is seated facing Bandit at a distance of 6 feet (1.8 meters). The robot and the participant are separated by a transparent, acrylic screen measuring 12 feet by 8 feet (2.4 by 3.6 meters) (see Figure 1). The screen covers a horizontal field of view from approximately -60 to 60 degrees and a vertical field of view -45 to 60 degrees. The robot performs a series of deictic gestures and the participant is asked to estimate their referent location on the screen. The robot is posed using a closedform inverse kinematic solution; however, a small firmware “dead-band” in each joint sometimes introduces error in reaching a desired pose. To monitor where the robot actually pointed, we computed forward kinematics using angles from encoder feedback, which we verified were accurate in separate controlled testing. All gestures were static and held indefinitely until the participant estimated a location, after which the robot returned to a home location (looking straight forward with its hands at its sides) before performing the next gesture. Participants were given a laser pointer to mark their estimated location for each gesture. These locations were recorded using a laser rangefinder placed facing upwards at the base of the screen. For each gesture, an experimenter placed a fiducial marker over the indicated location, which was subsequently localized within approximately 1 cm using the rangefinder data. The entire experiment was controlled via a single Nintendo Wiimote™, with which the experimenter could record marked locations and advance the robot to point to the next referent target.

The robot gestured to locations by moving its head, arm, or both together.
All the gestures were static, meaning the robot left the home position, reached the gesture position, and held it indefinitely until the participant chose a point, after which it returned to the home position for the next gesture. This was intended to minimize any possible timing effects by giving participants as long as they needed to estimate a given gesture.
The screen itself was presented with two visual saliency conditions: one in which it was completely empty (i.e., nonsalient) and one in which it was affixed with eight round markers distributed at random (i.e., salient). In the salient case, the markers were all 6 inches (15 cm) in diameter and were identical in shape and color. Experiments were conducted in two phases, reflecting the two saliency conditions. In the salient condition, the robot’s gestures included 60 points toward salient targets and 60 points chosen to be on the screen, but not necessarily at a salient target. In the non-salient condition, 74 of the points within the bounds of the screen were chosen pseudo-randomly and the remaining 36 were chosen to sample a set of 4 calibration locations with each pointing modality.
A total of 40 runs of the experiment were conducted as described, with 20 (12 female, 8 male) participating in the non-salient condition and 20 (11 female, 9 male) participants in the salient condition.

<img src="http://niftyhedgehog.com/deixis/images/experiment_setup.png">

<img src="http://niftyhedgehog.com/deixis/images/bandit_modality.png">


## Results
Aaron's publications

## Misc.
* [Interaction Lab Webpage](http://robotics.usc.edu/~hieu/)
* <span class="author">Aaron B. St. Clair, Ross Mead and Maja J. Matarić</span>. "<span class="title">Investigating the Effects of Visual Saliency on Deictic Gesture Production by a Humanoid Robot</span>". <span class="pub_status">In</span> <span class="booktitle">IEEE 20th IEEE International Symposium in Robot and Human Interactive Communication</span> <span class="booktitle">(Ro-Man 2011)</span>, <span class="address">Atlanta, GA</span>, <span class="month">Jul</span> <span class="year">2011</span>
* My experiments contributed to this [paper](http://robotics.usc.edu/publications/media/uploads/pubs/731.pdf)

<img src="http://niftyhedgehog.com/deixis/images/bandit_trio.jpg">
