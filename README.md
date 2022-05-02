# Robot search and rescue

## Intro

The objective: search an unknown environment for a specific target using cooperative robots


[<img src="images/illustration.png" width="450"/>](images/illustration.png)

Project details so far:
* We are planning on using ROS with python and Gazebo
* Deploy three holonomic robots with lidar sensors and wheel encoders
* The map is unknown and has obstacles. The robots start close to each other at one side of the map
* Use SLAM for localization and mapping
* Employ cooperative exploration approach using dynamic Voronoi partitions
* Robots explore map cooperatively until target is found
* The explored map is shared between the robots to minimize duplication
* Use pure pursuit for path following
* Implement collision avoidance with the help of costmap_2d ROS package

## Installation

please follow instructions in the install.md file

## Running

```
cd ~/search_rescue
source setup.bash
roslaunch platform_start.launch
```
