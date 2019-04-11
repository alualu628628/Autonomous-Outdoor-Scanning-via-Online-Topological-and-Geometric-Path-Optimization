
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

//status
struct Confidence{

	//distance based term
	float travelTerm;
	//visibility based term
	float visiTerm;
	//quality
	float qualTerm;
	//Weighted total of those two terms above
	float totalValue;

	//labels of obstacle, travelable region and boundary
	//0 nothing 
	//1 obstacles
	//2 ground points
	//3 boundary
	short label;

	//travelable or not (can the robot reaches this grid right now)
	//-1 indicates it is an unknown grid
	//0 indicates this grid is ground but not reachable now
	//1 indicates this grid is a travelable region grid
	//2 is the new scanned grids (input) without growing
	//3 indicates the grid has been computed
	//4 indicates this grid is a off groud grid (not reachable forever)
	short travelable;

	//quality computed flag
	bool qualFlag;
	//minimum computed flag
	bool nodeGenFlag;
	//center point of a map grid 
	pcl::PointXYZ oCenterPoint;

	//constructor
	Confidence() {

		travelTerm = 1.0;//start with 1, which means no need to go there
		visiTerm = 0.0;
		qualTerm = 0.0;
		totalValue = 1.0;//start with 1, which means no need to go there
		label = 0;//start with nothing
		travelable = -1;//start with unknown
		nodeGenFlag = false;	//start with undone	
		qualFlag = true;//start with undone
		oCenterPoint.x = 0.0;//start from 0, which will be re-define in InitializeGridMap
		oCenterPoint.y = 0.0;//start from 0
		oCenterPoint.z = 0.0;//start from 0

	};

};

//index of grid cell
struct MapIndex{

  //2D index 
  grid_map::Index oTwoIndex;//cell index in a matrix
  //1D index
  int iOneIdx; //cell index in a sequance

};

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
  //transform point position to 1 Dimension index
  int PointoOneDIdx(pcl::PointXYZ & oPoint);
  //transform point position to 1D and 2D indexes
  MapIndex PointoAllTypeIdx(pcl::PointXYZ & oPoint);
  //transform 2D index to 1D index
  int TwotoOneDIdx(const grid_map::Index & oIndex);
  int TwotoOneDIdx(const int & iIndexX, const int & iIndexY);
  //transform 1D index to 2D indexes
  void OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,const int & iOneDIdx);

  //search neighboring grids within a circle region
  void CircleNeighborhood(std::vector<MapIndex> & vNearbyGrids,
                          const pcl::PointXYZ & oRobotPoint);

  //*************handler function*************
  //handle the trajectory information
  //confidence calculation is triggered as soon as receving odom data
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
  void ComputeTravelFeature(const pcl::PointXYZ & oRobot);
  
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_vBoundCloud;//boundary point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_vObstacleCloud;//obstacle point clouds

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

  int m_iGridRawNum;//grid number in raws

  std::string m_sMapFrameID;//grid map ID
 
  grid_map::Position3 m_oMinCorner; //Bounding box minimum corner of map.

  grid_map::Position3 m_oMaxCorner; //Bounding box maximum corner of map.

  double m_dResolution; //resolution of map pixels/cells 

  //the map - main body 
  grid_map::GridMap m_oFeatureMap;//grid map

  std::vector<Confidence> m_vConfidenceMap;//confidence value map

  //the grid map initialization flag indicates whether the map has been simply established
  bool m_bGridMapReadyFlag;

  //main body of data
  std::vector<MapIndex> m_vSearchMask;
  
};

} /* namespace */


#endif


//************details of grid label value in m_vMapGridLabel
// cover means rewrite if new sematic object appears in this grid
// 0 nothing or unknown (0 is covered by 1)
// 1 obstacles (1 is covered by 2)
// 2 ground points (2 is covered by 3)
// 3 boundary 
// for example, boundary grid can not be defined as another classification, but obstacle can be recovered by ground or obstacles 
// the "-" negetive means the grid has been computed in node generation

//************details of grid status value in m_vMapGridTravel
//status indicates whether the gird is travelable or not (can the robot reaches this grid right now)
// -1 indicates it is an unknown grid
// 0 indicates this grid is ground but not reachable now
// 1 indicates this grid is a travelable region grid
// 2 is the new scanned grids (input) without growing
// 3 indicates the grid has been computed
// 4 indicates this grid is a off groud grid (not reachable forever)