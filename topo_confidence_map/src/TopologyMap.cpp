#include "TopologyMap.h"


/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

namespace topology_map{




TopologyMap::TopologyMap(ros::NodeHandle & node,
                         ros::NodeHandle & nodeHandle):m_oFeatureMap(grid_map::GridMap({"elevation"})),
                                                                                     m_iTrajFrameNum(0),
                                                                                     m_iOdomSampingNum(25),
                                                                                     m_iMapFreshNum(50){

  //read parameters
  ReadParameters(nodeHandle);
   
  m_oFeatureMap.setBasicLayers({"elevation"});
  

  //subscribe topic 
  m_oOctoMapClient = nodeHandle.serviceClient<octomap_msgs::GetOctomap>(m_oOctomapServiceTopic);

  //subscribe (hear) the odometry information
  m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 0, &HandleTrajectory, this);

  //publish topic
  m_oGridMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  m_oOctomapPublisher = nodeHandle.advertise<octomap_msgs::Octomap>("octomap", 1, true);

}

/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

TopologyMap::~TopologyMap()
{
}

/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

bool TopologyMap::ReadParameters(ros::NodeHandle & nodeHandle)
{
  //input topic
  nodeHandle.param("octomap_service_topic", m_oOctomapServiceTopic, std::string("/octomap_binary"));
  nodeHandle.param("odom_in_topic", m_oOdomTopic, std::string("/odometry/filtered"));

  //frequncy
  nodeHandle.param("odometry_rawfreq", m_dOdomRawHz, 50.0);
  nodeHandle.param("odomsampling_freq", m_dSamplingHz, 2.0);

  m_iOdomSampingNum = int(m_dOdomRawHz / m_dSamplingHz); 

  nodeHandle.param("freshoctomap_freq", m_dOctomapFreshHz, 1.0);
  
  m_iMapFreshNum = int(m_dOdomRawHz / m_dSamplingHz); 

  //input map 
  nodeHandle.param("min_x", m_fMinBoundX, NAN);
  nodeHandle.param("max_x", m_fMaxBoundX, NAN);
  nodeHandle.param("min_y", m_fMinBoundY, NAN);
  nodeHandle.param("max_y", m_fMaxBoundY, NAN);
  nodeHandle.param("min_z", m_fMinBoundZ, NAN);
  nodeHandle.param("max_z", m_fMaxBoundZ, NAN);

  //confidence map
  //neighboring region size
  nodeHandle.param("robot_local_r", m_dRbtLocalRadius, 5.0);

  return true;
}


/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but timestamp values 
Return: none
Others: none
*************************************************/
void TopologyMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
  
  bool bUpdateFlag = false;
  bool bSamplingFlag = false;

  //receive the global map to obtain the newest environment information
  if(!(m_iTrajFrameNum % m_iMapFreshNum)){
    std::cout<<"debug 1"<<std::endl;
    ConvertAndPublishMap();

  }                                                                                     

  //record the robot position and update its neighboring map
  if(!(m_iTrajFrameNum % m_iOdomSampingNum)){

    bUpdateFlag = true;

    //save the odom value
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z

    //make the sequence size smaller than a value
    if(m_vOdomCloud.size()>10000)
       m_vOdomCloud.pop();

    std::cout<<"debug 2"<<std::endl;

    CircleNeighboringGrids(m_vOdomCloud.back());


  }

  //if the frame count touches the least common multiple of m_iOdomSampingNum and 
  if( bUpdateFlag && bSamplingFlag ){

    std::count<< "m_iTrajFrameNum: " << m_iTrajFrameNum << std::endl;
    m_iTrajFrameNum = 0;
  }

  m_iTrajFrameNum++;

}

/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

void TopologyMap::CircleNeighboringGrids(pcl::PointXYZ & oRobotPoint)
{
  ROS_INFO("Running circle iterator demo.");

  Position oRobotPos(oRobotPoint.x, oRobotPoint.y);

  for (grid_map::CircleIterator iterator(m_oFeatureMap, oRobotPos, m_dRbtLocalRadius);
      !iterator.isPastEnd(); ++iterator) {
       m_oFeatureMap.at("elevation", *iterator) = 2.0;

  }

}


/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

void TopologyMap::ConvertAndPublishMap(){

  octomap_msgs::GetOctomap oOctomapServerVec;
  if (!m_oOctoMapClient.call(oOctomapServerVec)) {
    ROS_ERROR_STREAM("Failed to call service: " << m_oOctomapServiceTopic);
    return;
  }

  // creating octree
  octomap::OcTree* pOctomap = nullptr;
  octomap::AbstractOcTree* pOCTree = octomap_msgs::msgToMap(oOctomapServerVec.response.map);
  if (pOCTree) {
    pOctomap = dynamic_cast<octomap::OcTree*>(pOCTree);
  } else {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;

  pOctomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  pOctomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

  if(!std::isnan(m_fMinBoundX))
    min_bound(0) = m_fMinBoundX;
  if(!std::isnan(m_fMaxBoundX))
    max_bound(0) = m_fMaxBoundX;
  if(!std::isnan(m_fMinBoundY))
    min_bound(1) = m_fMinBoundY;
  if(!std::isnan(m_fMaxBoundY))
    max_bound(1) = m_fMaxBoundY;
  if(!std::isnan(m_fMinBoundZ))
    min_bound(2) = m_fMinBoundZ;
  if(!std::isnan(m_fMaxBoundZ))
    max_bound(2) = m_fMaxBoundZ;

  std::vector<std::vector<std::vector<int>>> vMapPointIndex;
  pcl::PointCloud<pcl::PointXYZ> vCloud;

  bool bConverterRes = GridOctoConverter::FromOctomap(*pOctomap, 
                                                    "elevation", 
                                                  m_oFeatureMap, 
                                                 vMapPointIndex,
                                                         vCloud,
                                                     &min_bound,
                                                     &max_bound);


  if (!bConverterRes) {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }

  m_oFeatureMap.setFrameId(oOctomapServerVec.response.map.header.frame_id);

  // Publish as grid map.
  grid_map_msgs::GridMap oGridMapMessage;
  grid_map::GridMapRosConverter::toMessage(m_oFeatureMap, oGridMapMessage);
  m_oGridMapPublisher.publish(oGridMapMessage);

  // Also publish as an octomap msg for visualization
  octomap_msgs::Octomap oOctomapMessage;
  octomap_msgs::fullMapToMsg(*pOctomap, oOctomapMessage);
  oOctomapMessage.header.frame_id = m_oFeatureMap.getFrameId();
  m_oOctomapPublisher.publish(oOctomapMessage);
}

} /* namespace */

