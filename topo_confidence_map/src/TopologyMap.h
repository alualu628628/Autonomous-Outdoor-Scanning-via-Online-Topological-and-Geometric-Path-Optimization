
#ifndef TOPOLOGYMAP_H
#define TOPOLOGYMAP_H
#include <cmath>
#include <queue>
#include <vector>
#include <string>


//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//octomap related
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

//pcl related
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"  


//grid_map related
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>//transfor octomap to grid

#include "OctoGrid.h"



using namespace grid_map;

namespace topology_map{

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation layer.
 * The grid map is published and can be viewed in Rviz.
 */
class TopologyMap{

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TopologyMap(ros::NodeHandle & node,
              ros::NodeHandle & nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TopologyMap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool ReadParameters(ros::NodeHandle & nodeHandle);


  void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);


  void ConvertAndPublishMap();


  void CircleNeighboringGrids(pcl::PointXYZ & oRobotPoint);

 private:

  //**input and output topic related**
  //input topics:
  ros::Subscriber m_oOdomSuber;//the subscirber is to hear (record) odometry from gazebo
  std::string m_sOdomTopic;  //the topic name of targeted odometry (robot trajectory)

  //output topics:
  //! Grid map publisher.
  ros::Publisher m_oGridMapPublisher;
 
  //! Octomap publisher.
  ros::Publisher m_oOctomapPublisher;

  // point cloud publisher for test
  ros::Publisher m_oCloudPublisher;

  //**service related**
  //the name of source octomap service
  std::string m_oOctomapServiceTopic;
  //Octomap service client
  ros::ServiceClient m_oOctoMapClient;//m_oOctoMapClient<->oOctomapServerVec<->m_oOctomapServiceTopic

  //**frenquency related**
  //the raw frequency of odometry topic (50hz, or 10hz in normal)
  double m_dOdomRawHz;
  //the down sampling frequency of odometry topic (5hz in normal)
  double m_dSamplingHz;
  //the frenquecy of updating otcomap (and also publish confidence at the same time)
  //thus, it also denotes the frequecny of publishing confidence map 
  double m_dOctomapFreshHz;
  //odometry sampling number is equal to int(m_dOdomRawHz / m_dSamplingHz);
  int m_iOdomSampingNum; 
  //received octomap updated time interval is equal to int(m_dOdomRawHz / m_dOctomapFreshHz);
  int m_iMapFreshNum; 
  //the frame count of trajectory point
  int m_iTrajFrameNum;

  //**point cloud related**
  //the positions of robot
  std::queue<pcl::PointXYZ> m_vOdomCloud;//I dont think it is necessary to use a circle vector
  
  //! Bounding box of octomap to convert.
  float m_fMinBoundX;
  float m_fMaxBoundX;
  float m_fMinBoundY;
  float m_fMaxBoundY;
  float m_fMinBoundZ;
  float m_fMaxBoundZ;


  double m_dRbtLocalRadius;


  //! Grid map data.
  grid_map::GridMap m_oFeatureMap;

};

} /* namespace */


#endif