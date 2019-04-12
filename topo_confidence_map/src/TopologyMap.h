
#ifndef TOPOLOGYMAP_H
#define TOPOLOGYMAP_H

#include <queue>
#include <string>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include "pcl_ros/transforms.h"  
#include <pcl_conversions/pcl_conversions.h>

#include "ConfidenceMap.h"

//octomap related
//#include <octomap/octomap.h>
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/GetOctomap.h>
//#include <octomap_msgs/conversions.h>

//#include <grid_map_octomap/GridMapOctomapConverter.hpp>//transfor octomap to grid

namespace topology_map{

//this class below is to compute topological guidance map for SLAM
class TopologyMap{

 public:

  //*************Initialization function*************
  //Constructor
  TopologyMap(ros::NodeHandle & node,
              ros::NodeHandle & nodeHandle);

  //Destructor
  virtual ~TopologyMap();

  //Reads and verifies the ROS parameters.
  bool ReadParameters(ros::NodeHandle & nodeHandle);

  //Initialize a fixed Grid Map
  void InitializeGridMap(const float & fRobotX, const float & fRobotY);

  //generate local neighboring index (generate a mask)
  void GenerateCircleMask();
  //*************Traversing / retrieving function*************
  //check inside points
  inline bool CheckInSidePoint(const pcl::PointXYZ & oPoint);

  //search neighboring grids within a circle region
  void CircleNeighborhood(std::vector<MapIndex> & vNearbyGrids,
                          const pcl::PointXYZ & oRobotPoint);

  //extract the point clouds from the given neighboring grids
  void DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                        pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
                          pcl::PointCloud<pcl::PointXYZ> & vNearAllClouds,
                                   std::vector<int> & vNearGroundGridIdxs,
                                const std::vector<MapIndex> & vNearByIdxs);

  //*************handler function*************
  //handle the trajectory information
  //ConfidenceValue calculation is triggered as soon as receving odom data
  void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

  //handle the ground point clouds topic
  void HandleGroundClouds(const sensor_msgs::PointCloud2 & vGroundRosData);

  //handle the boundary point cloud topic
  void HandleBoundClouds(const sensor_msgs::PointCloud2 & vBoundRosData);

  //handle the obstacle point cloud topic
  void HandleObstacleClouds(const sensor_msgs::PointCloud2 & vObstacleRosData);

  //update octomap octree nodes
  //void UpdatingOctomapNodes();

  //*************Feature calculation function (Subject function)*************
  void ComputeConfidence(const pcl::PointXYZ & oRobotPos);
  
  //*************Output function*************
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
  
  //double m_dOctomapFreshHz;//the frenquecy of updating otcomap (and also publish ConfidenceValue at the same time)
  
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pBoundCloud;//boundary point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pObstacleCloud;//obstacle point clouds

  //record the octree node point clouds (it can be seems as the down sampling of scanning point clouds)
  //pcl::PointCloud<pcl::PointXYZ> m_vNodeCloud;

  //point index in gridmap
  //grid cell x<grid cell y < node point id in vNodeCloud>>
  //std::vector<std::vector<std::vector<int> > > m_vMapPointIndex;//record the point index in grid map cells

  //std::vector<std::vector<int> > m_vGroundPntMapIdx;//ground point index in grid map
  std::vector<std::vector<int> > m_vBoundPntMapIdx;//boundary point index in grid map
  std::vector<std::vector<int> > m_vObstlPntMapIdx;//obstacle point index in grid map

  //**Grid map data**
  
  double m_dRbtLocalRadius;//construted maximum range of map 
  //note that the map size is fixed, thus it must be initialied large enough to pick the scene 
  double m_dMapMaxRange;///<it indicates the half length of bounding box (map)

  std::string m_sMapFrameID;//grid map ID
 
  grid_map::Position3 m_oMinCorner; //Bounding box minimum corner of map.

  grid_map::Position3 m_oMaxCorner; //Bounding box maximum corner of map.

  double m_dResolution; //resolution of map pixels/cells 

  //the map - main body 
  grid_map::GridMap m_oFeatureMap;//grid map

  Confidence m_oCnfdnSolver;//confidence object

  std::vector<ConfidenceValue> m_vConfidenceMap;//Confidence value map

  //the grid map initialization flag indicates whether the map has been simply established
  bool m_bGridMapReadyFlag;

  //main body of data
  std::vector<MapIndex> m_vSearchMask;
  
};

} /* namespace */


#endif

