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
                         ros::NodeHandle & nodeHandle)
                                     :m_oFeatureMap(grid_map::GridMap({"elevation"})){

  //read parameters
  ReadParameters(nodeHandle);
   
  m_oFeatureMap.setBasicLayers({"elevation"});

  m_oOctoMapClient = nodeHandle.serviceClient<octomap_msgs::GetOctomap>(m_oOctomapServiceTopic);

  //subscribe (hear) the odometry information
  m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 2, &HandleTrajectory, this);

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
  nodeHandle.param("octomap_service_topic", m_oOctomapServiceTopic, std::string("/octomap_binary"));
  nodeHandle.param("min_x", m_minBoundX, NAN);
  nodeHandle.param("max_x", m_maxBoundX, NAN);
  nodeHandle.param("min_y", m_minBoundY, NAN);
  nodeHandle.param("max_y", m_maxBoundY, NAN);
  nodeHandle.param("min_z", m_minBoundZ, NAN);
  nodeHandle.param("max_z", m_maxBoundZ, NAN);
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
  
  

  oTrajPoint.position.x = oTrajectory.pose.pose.position.x;//z in loam is x
  oTrajPoint.position.y = oTrajectory.pose.pose.position.y;//x in loam is y
  oTrajPoint.position.z = oTrajectory.pose.pose.position.z;//y in loam is z
  //save record time
  oTrajPoint.oTimeStamp =  oTrajectory.header.stamp;



}
void IteratorsDemo::demoCircleIterator()
{
  ROS_INFO("Running circle iterator demo.");
  map_.clearAll();
  publish();

  Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
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

  if(!std::isnan(m_minBoundX))
    min_bound(0) = m_minBoundX;
  if(!std::isnan(m_maxBoundX))
    max_bound(0) = m_maxBoundX;
  if(!std::isnan(m_minBoundY))
    min_bound(1) = m_minBoundY;
  if(!std::isnan(m_maxBoundY))
    max_bound(1) = m_maxBoundY;
  if(!std::isnan(m_minBoundZ))
    min_bound(2) = m_minBoundZ;
  if(!std::isnan(m_maxBoundZ))
    max_bound(2) = m_maxBoundZ;

  bool bConverterRes = grid_map::GridMapOctomapConverter::fromOctomap(*pOctomap, "elevation", m_oFeatureMap, &min_bound, &max_bound);
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


IteratorsDemo::IteratorsDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(vector<string>({"type"}))
{
  ROS_INFO("Grid map iterators demo node started.");
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

  // Setting up map.
  map_.setGeometry(Length(1.0, 1.0), 0.05, Position(0.0, 0.0));
  map_.setFrameId("map");

  publish();
  ros::Duration duration(2.0);
  duration.sleep();

  demoGridMapIterator();
  demoSubmapIterator();
  demoCircleIterator();
  demoEllipseIterator();
  demoSpiralIterator();
  demoLineIterator();
  demoPolygonIterator();
  demoSlidingWindowIterator();
}

IteratorsDemo::~IteratorsDemo() {}

void IteratorsDemo::demoGridMapIterator()
{
  ROS_INFO("Running grid map iterator demo.");
  map_.clearAll();
  publish();

  // Note: In this example we locally store a reference to the map data
  // for improved performance. See `iterator_benchmark.cpp` and the
  // README.md for a comparison and more information.
  grid_map::Matrix& data = map_["type"];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();
    data(i) = 1.0;
    publish();
    ros::Duration duration(0.01);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoSubmapIterator()
{
  ROS_INFO("Running submap iterator demo.");
  map_.clearAll();
  publish();

  Index submapStartIndex(3, 5);
  Index submapBufferSize(12, 7);

  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoCircleIterator()
{
  ROS_INFO("Running circle iterator demo.");
  map_.clearAll();
  publish();

  Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}



void IteratorsDemo::publish()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridMapPublisher_.publish(message);
  ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}