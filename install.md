Use Ubuntu 18.04 and ROS1 Melodic
Install TurtleBot3 via Debian Packages:

$ sudo apt-get install ros-melodic-dynamixel-sdk
$ sudo apt-get install ros-melodic-turtlebot3-msgs
$ sudo apt-get install ros-melodic-turtlebot3

instal ROS-melodic-SLAM:

$ sudo apt-get install ros-melodic-joy 
$ sudo apt-get install ros-melodic-teleop-twist-joy
$ sudo apt-get install ros-melodic-teleop-twist-keyboard
$ sudo apt-get install ros-melodic-laser-proc
$ sudo apt-get install ros-melodic-rgbd-launch
$ sudo apt-get install ros-melodic-depthimage-to-laserscan
$ sudo apt-get install ros-melodic-rosserial-Arduino
$ sudo apt-get install ros-melodic-rosserial-python
$ sudo apt-get install ros-melodic-rosserial-server
$ sudo apt-get install ros-melodic-rosserial-client
$ sudo apt-get install ros-melodic-rosserial-msgs
$ sudo apt-get install ros-melodic-amcl
$ sudo apt-get install ros-melodic-map-server
$ sudo apt-get install ros-melodic-move-base
$ sudo apt-get install ros-melodic-urdf
$ sudo apt-get install ros-melodic-xacro
$ sudo apt-get install ros-melodic-compressed-image-transport
$ sudo apt-get install ros-melodic-rqt-image-view
$ sudo apt-get install ros-melodic-gmapping
$ sudo apt-get install ros-melodic-navigation
$ sudo apt-get install ros-melodic-multirobot-map-merge

Catkin_make:
$ cd ~/search_rescue && catkin_make

For testing:

$ source setup.bash

$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

If the following error occurs after roslaunch do the following:

[Err] [REST.cc:205] Error in REST request
libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'

Open ~/.ignition/fuel/config.yaml, and replace api.ignitionfuel.org with fuel.ignitionrobotics.org
