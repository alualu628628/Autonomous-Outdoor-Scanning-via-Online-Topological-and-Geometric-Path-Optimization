cmake_minimum_required(VERSION 2.8.3)
project(topo_confidence_map)

## Compile as C++11
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)
# supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             pcl_ros
             roscpp
             rospy
             sensor_msgs
             std_msgs
             nav_msgs
             tf
             grid_map_core
             grid_map_ros
             grid_map_filters
             grid_map_loader
             grid_map_msgs
             #grid_map_octomap
             grid_map_rviz_plugin
             grid_map_visualization
             geometry_msgs
             sensor_msgs
             #octomap_msgs
             filters
)

message(STATUS "Found Octomap (version ${octomap_VERSION}): ${OCTOMAP_INCLUDE_DIRS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

## Specify additional locations of header files
if(${octomap_VERSION} VERSION_LESS 1.8) # ROS Indigo and Jade.
  add_definitions(-DOCTOMAP_VERSION_BEFORE_ROS_KINETIC)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIR}
  )
else() # ROS Kinetic and above.
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIRS}
  )
endif()

aux_source_directory(src DIR_SRCS)

###### build ########
add_executable(topo_confidence_map ${DIR_SRCS})


target_link_libraries(topo_confidence_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})



