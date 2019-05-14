
#ifndef TOPOLOGYMAP_H
#define TOPOLOGYMAP_H
#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include "pcl_ros/transforms.h"  
#include <pcl_conversions/pcl_conversions.h>

//#include "ConfidenceMap.h"
#include "Astar.h"
#include "LocalPathOptimization.h"

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
  bool ReadTopicParams(ros::NodeHandle & nodeHandle);

  //Initialize a fixed Grid Map
  void InitializeGridMap(const pcl::PointXYZ & oRobotPos);

  //down sample the point clouds with grid idxs
  void SamplingPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
                          std::vector<std::vector<int> > & vPointMapIdx,
                                                       int iSmplNum = 3);

  //down sampling the point clouds with grid idxs and corresponding labels
  void SamplingPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
                          std::vector<std::vector<int> > & vPointMapIdx,
                                        std::vector<int> & vCloudLabels,
                                                       int iSmplNum = 3);

  
  //*************Traversing / retrieving function*************


  //extract the point clouds from the given neighboring grids
  void DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                                   std::vector<int> & vNearGroundGridIdxs,
                        pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
                         pcl::PointCloud<pcl::PointXYZ> & vNearObstClouds,                                           
                                const std::vector<MapIndex> & vNearByIdxs,
                                                    const int & iNodeTime);

  void DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                        pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
                          pcl::PointCloud<pcl::PointXYZ> & vNearAllClouds,
                                   std::vector<int> & vNearGroundGridIdxs,
                                const std::vector<MapIndex> & vNearByIdxs);

  void DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                        pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
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
  void ComputeConfidence(const pcl::PointXYZ & oCurrRobotPos,
                         const pcl::PointXYZ & oPastRobotPos);

  void ComputeConfidence(const pcl::PointXYZ & oCurrRobotPos);
  
  //*************Output function*************
  //publish grid map
  void PublishGridMap();

  //publish recevied octomap
  //void PublishOctoMap(octomap::OcTree* pOctomap);
  
  //publish point clouds
  void PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud);

  void PublishGoalOdom(pcl::PointXYZ & oGoalPoint);

  //output data in file
  void OutputCoverRateFile(const int & iTravelableNum);
  void OutputTrajectoryFile(const nav_msgs::Odometry & oTrajectory);

 private:

  //**input topic related**
  //output file name
  std::string m_sFileHead;

  std::stringstream m_sCoverFileName; ///<full name of output txt that records the point clouds//defind it in the function
  bool m_bCoverFileFlag; //whether the outfile got a full name or not 
  std::ofstream m_oCoverFile;

  std::stringstream m_sOutTrajFileName;///<full name of output txt that records the trajectory point 
  bool m_bOutTrajFileFlag;//whether the outfile got a full name or not 
  std::ofstream m_oTrajFile;

  //input topics:
  ros::Subscriber m_oOdomSuber;//the subscirber is to hear (record) odometry from gazebo

  ros::Subscriber m_oGroundSuber;//the subscirber is to hear (record) odometry from gazebo
  ros::Subscriber m_oBoundSuber;//the subscirber is to hear (record) odometry from gazebo
  ros::Subscriber m_oObstacleSuber;//the subscirber is to hear (record) odometry from gazebo

  std::string m_sOdomTopic;  //the topic name of targeted odometry (robot trajectory)

  std::string m_sGroundTopic; //the topic name of targeted odometry (robot trajectory)

  std::string m_sBoundTopic; //the topic name of targeted odometry (robot trajectory)

  std::string m_sObstacleTopic; //the topic name of targeted odometry (robot trajectory)

  //**output topics related**

  ros::Publisher m_oGridMapPublisher;//! Grid map publisher.

  //ros::Publisher m_oOctomapPublisher; //! Octomap publisher.
  //publishing point cloud is only for test
  ros::Publisher m_oCloudPublisher;// point cloud publisher for test

  ros::Publisher m_oGoalPublisher;// goal odometry information publisher

  //**frenquency related**
  
  double m_dOdomRawHz;//the raw frequency of odometry topic (50hz, or 10hz in normal)
  
  double m_dSamplingHz;//the raw frequency of odometry topic (50hz, or 10hz in normal)

  float m_fViewZOffset;//z offset of odom to lidar sensor
  
  //sampling number

  int m_iOdomSampingNum;   //odometry sampling number is equal to int(m_dOdomRawHz / m_dSamplingHz);

  int m_iPCSmplNum; //point clouds sampling number for all classification point clouds
  
  //int m_iMapFreshNum; //received octomap updated time interval is equal to int(m_dOdomRawHz / m_dOctomapFreshHz);

  int m_iComputedFrame;

  int m_iPastOdomNum; //the past view interval number for occlusion calculation

  int m_iShockNum;//the shock duration that robot can tolerate 

  //the frame count of trajectory point
  unsigned int m_iTrajFrameNum;

  unsigned int m_iGroundFrames;

  unsigned int m_iBoundFrames;

  unsigned int m_iObstacleFrames;

  unsigned int m_iNodeTimes;//the times of node to be visited

  //**point cloud related**
  //the positions of robot
  std::queue<pcl::PointXYZ> m_vOdomViews;//I dont think it is necessary to use a circle vector
  std::queue<pcl::PointXYZ> m_vOdomShocks;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pBoundCloud;//boundary point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pObstacleCloud;//obstacle point clouds
  std::vector<int> m_vObstNodeTimes;//records the acquired times (node times) of each obstacle point

  //std::vector<std::vector<int> > m_vGroundPntMapIdx;//ground point index in grid map
  std::vector<std::vector<int> > m_vBoundPntMapIdx;//boundary point index in grid map
  std::vector<std::vector<int> > m_vObstlPntMapIdx;//obstacle point index in grid map

  //the map - main body 
  ExtendedGM m_oGMer;

  Confidence m_oCnfdnSolver;//confidence object

  std::vector<ConfidenceValue> m_vConfidenceMap;//Confidence value map

  //the grid map initialization flag indicates whether the map has been simply established
  bool m_bGridMapReadyFlag;

  //a node object
  OP m_oOPSolver;

  //a Astar estimator
  Astar m_oAstar;

  //a local path optimer
  PathOptimization oLclPthOptimer;

  //some paramters which is no more important
  //the radius which initial the travelable region around robot at original location
  float m_fInitialR;

  //follow anchor path or not
  //flag indicate the path selection
  bool m_bAnchorGoalFlag; 
  
  //goal point of global path planning
  pcl::PointXYZ m_oNodeGoal;

  //anchor goals - local path
  pcl::PointCloud<pcl::PointXYZ> m_vAncherGoals;

  //count visited anchor in a trip
  int m_iAncherCount;
  
};

} /* namespace */


#endif

