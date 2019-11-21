# Husky Simylation

This is a simulated system (based on ros gazebo) for outdoor mobile robot Husky to explore the unknown outdoor scenes.

## Installation
The version is based on [ROS - Kinetic](http://www.ros.org/) and particularly the [Gazebo](gazebosim.org). 
Though this system in Ubuntu 16.04 is avaliable, we still suggest building this system on Ubuntu 14.04 due to the stability. 
Please follow the following steps in order to install the package along with its dependences:

- Install ROS-Kinetic full desktop ([guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)).

- Install Husky official package ([guide](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky))

- Install the grid_map package that can be optionally used for mapping ([guide](https://github.com/ANYbotics/grid_map))(i.e.,sudo apt-get install ros-kinetic-grid-map)

- check whether other dependences of Husky package is need. Assuming Husky package is cloned under `$HUSKY_DIR$`:
```
cd $HUSKY_DIR$
rosdep install --from-path src --ignore-src --rosdistro=kinetic -y
```
- Install this package:
```
cd $HUSKY_DIR$
catkin_make -DCMAKE_BUILD_TYPE=Release
```
- Set the environment variables:
1. set the environment variable at `~/.bashrc` as:
```
gedit ~/.bashrc 
```

2. add (write) the correct path of customized and official husky description in gazebo and others
```
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
export HUSKY_URDF_EXTRAS=$(rospack find husky_custom_description)/urdf/custom_description.urdf.xacro
```



## Running
setup the environment variables
Run `source devel/setup.bash` in project root in order to .
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

For mapping with topological and geometrical information 
```
roslaunch topo_confidence_map mapping.launch
```

To automatically send the goal of mapping results to husky moving action 
```
roslaunch husky_navigation_goals send_goal.launch 
```

Other launch files can be used for different other outdoor environments e.g., `husky_wild.launch`, `husky_terrain.launch`.


## Parameters
output the result in your own path, please modify below codes in '/simulation_ws/src/topo_confidence_map/launch/mapping.launch'
```
<arg name="fileoutputpath" default="/home/yourname/"/>
```










