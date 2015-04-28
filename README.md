deixis
======

<img src="http://niftyhedgehog.com/deixis/images/bandit_point4.jpg">

## Overview
Deixis was a study of human-robot interaction (HRI), which incorporated human trials to investigate the perception of robotic deictic gestures. A "deictic gesture" is a form of nonverbal communication that typically serves to focus an observer's attention to a specific object or location in the environment. Humans will often employ nonverbal modalities such as facial expression, eye gaze, head orientation, and arm gesture to signal intent "here" or "there". For example, when dining at a foreign restaurant, I will extend my index finger toward a menu item to signal my intent to order "this" cheeseburger.

Similarly, when humans and robots are involved in a collaborative task in a shared environment, robots must utilize deictic gestures to sustain effective communication and natural interaction. Robots cannot always rely on speech to communicate. Using an upper-torso humanoid robot, Deixis is an experimental study of human perception of a robot’s deictic gestures under a set of different environmental visual saliency conditions and pointing modalities. It attempts to gain an empirical understanding of how to map well-studied human gestures to robots of varying capabilities and embodiments.

This project was developed in the summer of 2010 during an NSF research fellowship under the direction of Ph.D candidates, Aaron St. Clair and Ross Mead, and Dr. Maja Matarić. 


## Hardware
The Deixis project utilized "Bandit" as the gesturing robot, the SICK LMS for finding markers, overhead- and forward-facing cameras for head-tracking, and a Wiimote for facilitating the experiment.

* [Bandit II](http://robotics.usc.edu/interaction/?l=Laboratory:Facilities#BanditII), an expressive upper-torso humanoid robot
* [SICK LMS200](https://mysick.com/PDF/Create.aspx?ProductID=33755&Culture=en-US) laser rangefinder
* [Point Grey Firefly MV](http://www.ptgrey.com/firefly-mv-usb2-cameras) cameras for overhead tracking
* Nintendo [Wiimote](http://en.wikipedia.org/wiki/Wii_Remote) controller for wireless facilitation of the experiment

## Software
This project was software-intensive, requiring the integration of several sub-projects for deictic gesture validation and a user-studies experiment. It heavily utilized the Robot Operating System (ROS) framework for controlling Bandit's movements, collecting data, and manipulating data output.

* [ROS](http://www.ros.org/) for robot control, marker finding, data collection, Wiimote integration
* [OpenCV](http://opencv.org/) for camera image manipulation
* [ARToolKit](http://artoolkit.sourceforge.net/) for head tracking
* [R](http://www.r-project.org/) to generate statistical plots

### Camera Calibration
The Bandit robot has cameras situated in its left and right eyes. This stereo camera setup simulates human binocular vision to perceive depth, thus being able to capture three-dimensional images. When the cameras are calibrated, 3D information can be extracted by examination of the relative positions of objects in the left and right images.

The cameras must be calibrated properly to relieve the images of distortions. This is performed using ROS' checkerboard [camera calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) technique. The raw stereo camera image pairs can then be rectified and de-mosaic'd to generate dispaity maps and point clouds (stereo_image_proc).

Here is an example of me sitting at my desk. Raw camera images are collected from the left and right cameras to generate a disparity map and a point cloud, which can be viewed in RViz to display depth information.

<img src="http://niftyhedgehog.com/deixis/images/stereo_left.png" width="300">
<img src="http://niftyhedgehog.com/deixis/images/stereo_right.png" width="300">

<img src="http://niftyhedgehog.com/deixis/images/stereo_disparity.png">

<img src="http://niftyhedgehog.com/deixis/images/stereo_rviz.png">

<img src="http://niftyhedgehog.com/deixis/images/stereo_rviz2.png">

### Marker Finder
The marker_finder is a program used to log a human's perception of robot gestures. It utilizes the SICK laser measurement sensor to capture the exact position on a screen of where a human believes a robot is pointing. The SICK LMS is oriented facing upwards so that it can capture the proximity of objects sticking out from the screen, scanning 180-degrees from the floor. 

In the image below, the rectangular outline represents the screen. The two symmetric yellow square points represent the standoff legs at the base of the screen. For the experiment, I used a Pringles can to serve as the "marker", which is detected in the generated point cloud as a cluster of points. The absolute position of the marker is logged and represented by the red square point.

<img src="http://niftyhedgehog.com/deixis/images/laser_ss.png">

### Head Tracking
Head orientation was captured using a head-mounted ARTag. The ARTag's square physical markers is tracked in real time, allowing the calculation of camera position and orientation relative to the marker. 

<video src="http://niftyhedgehog.com/deixis/images/ARTag_test.mp4" width="100%" height="300px" controls="controls" loop="loop"></video>

### Bag Files
A "bag" is a ROS file format for storing ROS message data. In the Deixis experiment, several ROS topics are published including image data from cameras, point clouds, joystick button presses, and robot joint angles. All of this data can be recorded and played back using a tool like [rosbag](http://wiki.ros.org/rosbag).

The bag2video package is a utility that will consume a bag file of compressed images, and return a .avi file (hence the name, "bag2video") of variable framerate. It uses image_transport for publishing/subscribing to images between ROS nodes, and CvBridge to convert between ROS image messages and OpenCV IPLimages. The videos I converted use ARTags, which are used for tracking head orientation through augmented reality.

### Wiimote
A Wiimote controller was used to facilitate the experiment and save waypoint coordinates with sensor input from the SICK LMS. The program used the ROS wiimote package to communicate over a Bluetooth connection, returning button, LED, rumble, and accelerometer/gyro data on the /joy ROS node.


## Experiment
<img src="http://niftyhedgehog.com/deixis/images/experiment_setup.png">

The Deixis experiment was designed to observe the effects of visual saliency and pointing modality on human perceptual accuracy of a robot’s deictic gestures. In the experiments, the participant is seated facing Bandit at a distance of 6 feet (1.8 meters). The robot and the participant are separated by a transparent, acrylic screen measuring 12' x 8' (2.4m x 3.6m). The robot then performs a series of deictic gestures and the participant is asked to estimate their referent location on the screen. 

Participants were given a laser pointer to mark their estimated location for each gesture. These locations were recorded using a laser rangefinder placed facing upwards at the base of the screen. For each gesture, an experimenter placed a fiducial marker over the indicated location, which was subsequently localized within approximately 1 cm using the rangefinder data. The entire experiment was controlled via a single Nintendo Wiimote, with which the experimenter could record marked locations and advance the robot to point to the next referent target.

Bandit gestured to locations on the transparent screen using different pointing modalities such as moving its head (2 DOF), arm (7 DOF), or both together. The robot is posed using a closedform inverse kinematic solution; however, a small firmware "dead-band" in each joint sometimes introduces error in reaching a desired pose. To monitor where the robot actually pointed, we computed forward kinematics using angles from encoder feedback, which we verified were accurate in a separate controlled testing. All gestures were static and held indefinitely until the participant estimated a location, after which the robot returned to a home location (looking straight forward with its hands at its sides) before performing the next gesture. This was intended to minimize any possible timing effects by giving participants as long as they needed to estimate a given gesture.

<img src="http://niftyhedgehog.com/deixis/images/bandit_modality.png">

In cluttered environments containing many visually salient targets, it can be challenging for observers to decipher between objects. The Deixis experiment tested two saliency conditions: a blank (or non-salient) environment and an environment with several highly and equally visually salient targets. It was expected that salient objects would affect people’s interpretations of the points. Specifically, that participants would "snap to" the salient objects, thus reducing error for points whose targets were on or near markers, whereas in the non-salient condition there were no points of reference to bias estimates.

Experiments were conducted in two phases, reflecting the two saliency conditions. In the salient condition, the robot’s gestures included 60 points toward salient targets and 60 points chosen to be on the screen, but not necessarily at a salient target. In the non-salient condition, 74 of the points within the bounds of the screen were chosen pseudo-randomly and the remaining 36 were chosen to sample a set of 4 calibration locations with each pointing modality. 


## Results
My work with Deixis contributed to this [publication](http://robotics.usc.edu/publications/media/uploads/pubs/731.pdf), which contains statistical data analysis of the experiments.

<span class="author">Aaron B. St. Clair, Ross Mead and Maja J. Matarić</span>. "<span class="title">Investigating the Effects of Visual Saliency on Deictic Gesture Production by a Humanoid Robot</span>". <span class="pub_status">In</span> <span class="booktitle">IEEE 20th IEEE International Symposium in Robot and Human Interactive Communication</span> <span class="booktitle">(Ro-Man 2011)</span>, <span class="address">Atlanta, GA</span>, <span class="month">Jul</span> <span class="year">2011</span>

Additional detail can be found at [robotics.usc.edu/~hieu](http://robotics.usc.edu/~hieu/).

<img src="http://niftyhedgehog.com/deixis/images/bandit_trio.jpg">
