
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
//#include <octomap/octomap.h>
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/GetOctomap.h>
//#include <octomap_msgs/conversions.h>

//pcl related
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"  


//grid_map related
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
//#include <grid_map_octomap/GridMapOctomapConverter.hpp>//transfor octomap to grid

#include "OctoGrid.h"



using namespace grid_map;

namespace topology_map{

struct MapIndex{

  grid_map::Index oTwoIndex;
  int iOneIdx;

};




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

  //Initialize a fixed Grid Map
  void InitializeGridMap(const float & fRobotX, const float & fRobotY);

  inline bool CheckInSidePoint(const pcl::PointXYZ & oPoint);
  //handle the trajectory information
  //confidence calculation is triggered as soon as receving odom data
  void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

  void HandleGroundClouds(const sensor_msgs::PointCloud2 & vGroundRosData);

  void HandleBoundClouds(const sensor_msgs::PointCloud2 & vBoundRosData);

  void HandleObstacleClouds(const sensor_msgs::PointCloud2 & vObstacleRosData);

  //update octomap octree nodes
  void UpdatingOctomapNodes();

  void GenerateCircleMask();

  int PointoOneDIdx(pcl::PointXYZ & oPoint);

  int TwotoOneDIdx(const grid_map::Index & oIndex);

  void OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,const int & iOneDIdx);

  void ComputeTravelFeature(const pcl::PointXYZ & oRobot);

  //search neighboring grids within a circle region
  void CircleNeighborhood(std::vector<MapIndex> & vNearbyGrids,
                          const pcl::PointXYZ & oRobotPoint);
   

  //**output function**

  //publish grid map
  void PublishGridMap();

  //publish recevied octomap
  //void PublishOctoMap(octomap::OcTree* pOctomap);
  
  //publish point clouds
  void PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud);

 private:

  //**input topic related**
  //input topics:
  ros::Subscriber m_oOdomSuber;//the subscirber is to hear (record) odometry from gazebo

  ros::Subscriber m_oGroundSuber;//the subscirber is to hear (record) odometry from gazebo
  ros::Subscriber m_oBoundSuber;//the subscirber is to hear (record) odometry from gazebo
  ros::Subscriber m_oObstacleSuber;//the subscirber is to hear (record) odometry from gazebo

  std::string m_sOdomTopic;  //the topic name of targeted odometry (robot trajectory)

  std::string m_sGroundTopic; //the topic name of targeted odometry (robot trajectory)

  std::string m_sBoundTopic; //the topic name of targeted odometry (robot trajectory)

  std::string m_sObstacleTopic; //the topic name of targeted odometry (robot trajectory)
		
  //input service:
  //std::string m_oOctomapServiceTopic;//the name of source octomap service
  //m_oOctoMapClient<->oOctomapServerVec<->m_oOctomapServiceTopic
  //ros::ServiceClient m_oOctoMapClient;//Octomap service client

  //**output topics related**

  ros::Publisher m_oGridMapPublisher;//! Grid map publisher.

  //ros::Publisher m_oOctomapPublisher; //! Octomap publisher.
  //publishing point cloud is only for test
  ros::Publisher m_oCloudPublisher;// point cloud publisher for test

  //**frenquency related**
  
  double m_dOdomRawHz;//the raw frequency of odometry topic (50hz, or 10hz in normal)
  
  double m_dSamplingHz;//the raw frequency of odometry topic (50hz, or 10hz in normal)
  
  //double m_dOctomapFreshHz;//the frenquecy of updating otcomap (and also publish confidence at the same time)
  
  //sampling number

  int m_iOdomSampingNum;   //odometry sampling number is equal to int(m_dOdomRawHz / m_dSamplingHz);

  int m_iPCSmplNum; //point clouds sampling number for all classification point clouds
  
  //int m_iMapFreshNum; //received octomap updated time interval is equal to int(m_dOdomRawHz / m_dOctomapFreshHz);

  //the frame count of trajectory point
  int m_iTrajFrameNum;

  int m_iGroundFrames;

  int m_iBoundFrames;

  int m_iObstacleFrames;

  //**point cloud related**
  //the positions of robot
  std::queue<pcl::PointXYZ> m_vOdomCloud;//I dont think it is necessary to use a circle vector

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_vGroundCloud;//ground point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_vBoundCloud;//boundary point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_vObstacleCloud;//obstacle point clouds

  //record the octree node point clouds (it can be seems as the down sampling of scanning point clouds)
  //pcl::PointCloud<pcl::PointXYZ> m_vNodeCloud;

  //point index in gridmap
  //grid cell x<grid cell y < node point id in vNodeCloud>>
  //std::vector<std::vector<std::vector<int> > > m_vMapPointIndex;//record the point index in grid map cells

  std::vector<std::vector<int> > m_vGroundPntMapIdx;//ground point index in grid map
  std::vector<std::vector<int> > m_vBoundPntMapIdx;//boundary point index in grid map
  std::vector<std::vector<int> > m_vObstlPntMapIdx;//obstacle point index in grid map

  //**Grid map data**
  
  double m_dRbtLocalRadius;//construted maximum range of map 
  //note that the map size is fixed, thus it must be initialied large enough to pick the scene 
  double m_dMapMaxRange;///<it indicates the half length of bounding box (map)

  int m_iGridRawNum;//grid number in raws

  std::string m_sMapFrameID;//grid map ID
 
  grid_map::Position3 m_oMinCorner; //Bounding box minimum corner of map.

  grid_map::Position3 m_oMaxCorner; //Bounding box maximum corner of map.

  double m_dResolution; //resolution of map pixels/cells 
  //the map - main body 
  grid_map::GridMap m_oFeatureMap;//grid map
  //the grid map initialization flag indicates whether the map has been simply established
  bool m_bGridMapReadyFlag;

  std::vector<MapIndex> m_vSearchMask;
  //

};

} /* namespace */


#endif


