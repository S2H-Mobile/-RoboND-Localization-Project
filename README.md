[//]: # (Image References)

[screenshot]: ./screenshot_rviz_rover.png

# RoboND-Localization-Project, *Where am I?*
![Screenshot in RViz with Rover model at the goal position.][screenshot]

This repository contains a solution of the robot localization project for the Robotics Nanodegree. It consists of two ROS packages, ``udacity_bot`` and ``rover``. Each package defines a mobile robot and a navigation goal. The maps of the 2D environment are located in the ``map`` folder. The [writeup report](https://github.com/S2H-Mobile/RoboND-Localization-Project/blob/master/writeup/writeup_where_am_i.pdf) contains a description of a series of parameter tuning experiments.

## Setup
This project runs on Ubuntu Linux with ROS Kinetic.
- Install the ROS navigation stack:
``` bash
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ rospack profile
$ sudo apt-get install ros-kinetic-amcl
```
- Set up a catkin workspace.
- Clone the repository into the source folder of the catkin workspace.
- Make the workspace:
``` bash
$ catkin_make
```

## Usage
1. Launch the environment.
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> udacity_world.launch
```
A Gazebo and a RViz simulation environment is launched.

2. Launch the AMCL node.
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> amcl.launch
```

3.  Launch the navigation goal node.
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun <udacity_bot navigation_goal, rover rover_navigation_goal>
```
The selected mobile robot starts navigating towards the target location.
