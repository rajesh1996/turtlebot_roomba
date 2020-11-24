## Obstcle Avoidance - Turtlebot
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/rajesh1996/beginner_tutorials/blob/master/LICENSE)

## Author
Rajeshwar N S

## Overview
Turtlebot performing roomba like movement with obstacle avoidance

## Dependencies
* Ubuntu-18.04
* ROS Melodic

## Create catkin workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

## Build Instructions
* Clone the below repoitory in catkin_ws
```
$ cd ~/catkin_ws/src
$ https://github.com/rajesh1996/turtlebot_roomba.git
```
* Source the directory and then build it
```
$ cd ..
$ source ~/catkin_ws/devel/setup.bash
$ catkin_make
```

## Run Instructions
* Launch the node
```
$ roslaunch turtlebot_roomba roomba_turtle.launch
```
* Comment out the built in turtlebot3 world in the launch file and uncomment the below lines in launch file to spawn the bot in a custom world file

## Record ROSbag
* start roscore and type the following in a new terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_roomba roomba_turtle.launch start_record:=true
```
* This command will ouput a bag file containing the messages in the results dir

## Play rosbag file
* Comment out the turtlebot_roomba node in launch file and Start roscore and in a new terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_roomba roomba_turtle.launch
```
* In an new terminal, play the stored rosbag in the results dir. Now we will be able to see the bot in gazebo performing recorded movements from bag file
```
source ~/catkin_ws/devel/setup.bash
cd catkin_ws/src/turtlebot_roomba/results
rosbag play roomba.bag
```





