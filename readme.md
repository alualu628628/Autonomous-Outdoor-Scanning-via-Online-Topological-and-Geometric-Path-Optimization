# Autonomous Outdoor 3D Point Clouds Scanning via Path Planning
## Keyword
Husky robot; Unmanned vehicle; LiDAR; 3D Point Clouds; Autonomous driving; Logistics vehicles

## Introduction
This is a simple version (based on ros gazebo) for outdoor mobile robot Husky to explore the unknown outdoor scenes. This implementation is on ROS melodic version. 

The related work is published in Autonomous Outdoor Scanning via Online Topological and Geometric Path Optimization, IEEE Transactions on Intelligent Transportation Systems, DOI: 10.1109/TITS.2020.3039557. 

## Installation
Makesure you have a ubuntu18.04 system with high version cmake, gcc, g++

The system is based on [ROS - Melodic](http://www.ros.org/) and particularly the [Gazebo](gazebosim.org). 
 
Please follow the step belows in order to install the package along with its dependences:

- Install ROS-Melodic full desktop ([guide](https://wiki.ros.org/melodic/Installation/Ubuntu)).

- Install the grid_map package that can be optionally used for mapping ([guide](https://github.com/ANYbotics/grid_map )):
```
sudo apt-get install ros-melodic-grid-map
```
cd your own ros work space where this package is placed

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```
- Set the environment variables:
1. set the environment variable at `~/.bashrc` as:
```
gedit ~/.bashrc 
```
2. set the environment variable of husky (custom already embedded in our melodic verison):
```
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```

## Running
setup the environment variables of your ros private workspace
Run `source devel/setup.bash` in project root in order to add ros package path.
Instead, We strongly recommend writing the variables into `~/.bashrc` so that you don't need to set up environment variables any more when you run this project.

launches customized husky gazebo (`gazebo`).
```
roslaunch husky_gazebo husky_playpen.launch
```

launches customized husky visualization (`rviz`).
```
roslaunch husky_viz view_robot.launch
```

LOAM can be used to SLAM (mapping and location robot) as follow(gp-insac algorithm is launched parallelly, which is to devide ground, obstacle and boundary points):
```
roslaunch loam_velodyne loam_velodyne.launch
```

For mapping with topological and geometrical information. It should be noted that this node has a countdown of ten seconds. If it still have not received the data from LOAM node after the countdown, it will be automatically terminated. 
```
roslaunch topo_confidence_map mapping.launch
```

To automatically send the goal of mapping results to husky moving action 
```
roslaunch husky_navigation_goals send_goal.launch 
```


## Parameters
Output the result in your own path, please modify below codes in `/topo_confidence_map/launch/mapping.launch`:
```
<arg name="fileoutputpath" default="/home/yourname/"/>
```
<br>SLAM and ground detection parameters are list in `loam_velodyne/launch/loam_velodyne.launch`.
<br>Mapping parameters are list in `topo_confidence_map/launch/mapping.launch`.


