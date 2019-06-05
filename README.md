# RoboND-Localization-Project
This is a solution of the project *Where am I?* for the Robotics Nanodegree. The repository contains two ROS packages, ``udacity_bot`` and ``rover``. Each package defines a mobile robot and a navigation goal. The 2D environment maps are contained in the ``map`` folder.

In the [writeup report](https://github.com/S2H-Mobile/RoboND-Localization-Project/blob/master/writeup/writeup_where_am_i.pdf), several parameter tuning experiments are described.

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
1. Launch a console
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> udacity_world.launch
```

2. Launch a console
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> amcl.launch
```

3.  Launch a console
``` bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun udacity_bot navigation_goal
```

A Gazebo and a RViz simulation launch and the selected mobile robot starts navigating towards the target location.
