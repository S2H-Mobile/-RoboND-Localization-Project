# RoboND-Localization-Project
This is a solution of the *Where am I?* project as part of the Robotics Nanodegree. The repository contains ROS packages that describe two mobile robots ``udacity_bot`` and ``rover``, a ``map`` environment, and a [writeup report]
(https://github.com/S2H-Mobile/RoboND-Localization-Project/blob/master/writeup/writeup_where_am_i.pdf](https://github.com/S2H-Mobile/RoboND-Localization-Project/blob/master/writeup/writeup_where_am_i.pdf).

## Setup
This project runs on Ubuntu Linux with ROS Kinetic installed. Perform the following steps:
- Setup a catkin workspace.
- Clone the repository into the source folder of the catkin workspace.
- Open a console and execute
```
$ catkin_make
$ source devel/setup.bash
```

## Usage
1. Launch a console
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> udacity_world.launch
```

2. Launch a console
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch <rover, udacity_bot> amcl.launch
```

3.  Launch a console
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun udacity_bot navigation_goal
```