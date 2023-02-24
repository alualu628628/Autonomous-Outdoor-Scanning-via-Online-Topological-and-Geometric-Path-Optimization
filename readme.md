# Autonomous Outdoor 3D Point Clouds Scanning via Path Planning
## Keyword
Husky robot; Unmanned vehicle; LiDAR; 3D Point Clouds; Autonomous driving; Logistics vehicles

## Introduction
This is a simple version (based on ros gazebo) for outdoor mobile robot Husky to explore the unknown outdoor scenes. This implementation is on ROS indigo version, and a kenetic or melodic(recommended) based implementation can be found in other branch of this depository. The indigo version is more stable because it is consistent with Husky office packages.

The related work is published in Autonomous Outdoor Scanning via Online Topological and Geometric Path Optimization, IEEE Transactions on Intelligent Transportation Systems, DOI: 10.1109/TITS.2020.3039557. 

## Installation
Makesure you have a ubuntu14.04 system with high version cmake(>=3.1.3), gcc(>=5.5), g++(>=5.5), which are to support c++ 14 standard (LOAM needs, see [LOAM](https://github.com/laboshinl/loam_velodyne))

The system is based on [ROS - Indigo](http://www.ros.org/) and particularly the [Gazebo](gazebosim.org). 
 
Please follow the following steps in order to install the package along with its dependences:

- Install ROS-Indigo full desktop ([guide](http://wiki.ros.org/indigo/Installation/Ubuntu)).

- Install Husky official package ([guide](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky ))

- Install the grid_map package that can be optionally used for mapping ([guide](https://github.com/ANYbotics/grid_map )):
```
sudo apt-get install ros-indigo-grid-map
```
Please note that this package upon is located in the main server of ubuntu, not in some mirror servers. Alternatively, you can also build it from source.

- Install ROS-Indigo move base package if you need:
```
sudo apt-get install ros-indigo-navigation
```

- check whether other dependences of Husky package is need. Assuming Husky package is cloned under `$HUSKY_DIR$`:
```
cd $HUSKY_DIR$
rosdep install --from-path src --ignore-src --rosdistro=indigo -y
```
- Install this package (this system):
```
cd $HUSKY_DIR$
catkin_make -DCMAKE_BUILD_TYPE=Release
```
- Set the environment variables:
1. set the environment variable at `~/.bashrc` as:
```
gedit ~/.bashrc 
```
2. set the environment variable of husky_custom_description for your own custom robot model:
```
export HUSKY_URDF_EXTRAS=$(rospack find husky_custom_description)/urdf/custom_description.urdf.xacro
```
&nbsp;&nbsp;&nbsp;&nbsp;So far, there should have been two environment variables on the `~/.bashrc` file as shown below:
```
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
export HUSKY_URDF_EXTRAS=$(rospack find husky_custom_description)/urdf/custom_description.urdf.xacro
```
3. Modify HUSKY official file, which would cause a bug when loading customized robot urdf file. (see [Husky_description_issue](https://answers.ros.org/question/297415/invalid-param-tag-husky-simulation/)) Insert the `--inorder` option in `/husky_gazebo/launch/spawn_husky.launch` as shown below:
```
<param name="robot_description" 
       command="$(find xacro)/xacro '$(arg husky_gazebo_description)'
                laser_enabled:=$(arg laser_enabled)
                ur5_enabled:=$(arg ur5_enabled)
                kinect_enabled:=$(arg kinect_enabled)
                --inorder
                " />
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


## Issues
Issues occur mainly because the version of gcc, g++, or other third-party libraries in customized system are too old. The following env configuration problems and corresponding solutions may be helpful: 

<br>1. Could not find a package configuration file provided by "move_base_msgs" with any of the following names:

    move_base_msgsConfig.cmake
    move_base_msgs-config.cmake


solution: 
```
sudo apt-get install ros-indigo-navigation
```

<br>2. CMake 3.1.3 or higher is required (This is a requirement from LOAM package). 
Update your cmake, see [CMAKE INSTALL](https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu).
**WARNING**: this command `sudo apt remove cmake` would also remove ros system from your computer.

<br>3. Eigen3, this is a requirement from LOAM package, project has been updated and this issues seems to be solved. 
Could not find a package configuration file provided by "Eigen3" with any of the following names:

    Eigen3Config.cmake
    eigen3-config.cmake

solution: 
Type in the terminal
```
locate FindEigen3.cmake 
```
copy the FindEigen3.cmake to `Husky_Simulation/loam_velodyne/`.

In `Husky_Simulation/loam_velodyne/CMakeLists.txt` insert sentence `set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})`:
```
.....
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}) //add this sentence
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
.....
```
<br>4. MultiScanRegistration.cpp:178:76: error: parameter declared ‘auto’ (This is a requirement from LOAM package). 
This is because the current CMake, g++ or gcc can not automatically recognize the C++ 11/14 standard syntax, which appears in 'loam_velodyne' package.
First step, update your gcc and g++ version to support c++11/14 standard. Assuming the current gcc and g++ version is 4.8, type in terminal as below to get a  5.0 version:
```
sudo apt-get install python-software-properties
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update

sudo apt-get install gcc-5         
sudo apt-get install g++-5
sudo apt-get install gcc-5-multilib 
sudo apt-get install g++-5-multilib
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 30
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 30
sudo update-alternatives --config g++ 
sudo update-alternatives --config gcc
```

Second step, in order to configure catkin_make for automatically compiling c++11, type in terminal as below:
```
sudo vim /opt/ros/<yourversion>/share/catkin/cmake/toplevel.cmake
```
insert this sentence at anywhere of toplevel.cmake
```
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
```
<br>5. /usr/include/boost/math/constants/constants.hpp:273:3: error: unable to find numeric literal operator ‘operator"" Q’. This is a requirement from LOAM package, see details in [Same issue](https://github.com/laboshinl/loam_velodyne/issues/90), type in terminal as below:

```
sudo vim /opt/ros/<yourversion>/share/catkin/cmake/toplevel.cmake
```
insert this sentence at anywhere of `toplevel.cmake`:
```
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
```
If you follow the solution of issue 4, cover `set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")` as `set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")`
