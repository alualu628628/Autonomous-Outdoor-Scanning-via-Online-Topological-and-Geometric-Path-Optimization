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
                                     :map_(grid_map::GridMap({"elevation"}))
{
  readParameters(nodeHandle);
  client_ = nodeHandle.serviceClient<octomap_msgs::GetOctomap>(octomapServiceTopic_);
  map_.setBasicLayers({"elevation"});
  gridMapPublisher_ = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  octomapPublisher_ = nodeHandle.advertise<octomap_msgs::Octomap>("octomap", 1, true);
}

TopologyMap::~TopologyMap()
{
}

bool TopologyMap::readParameters(ros::NodeHandle & nodeHandle)
{
  nodeHandle.param("octomap_service_topic", octomapServiceTopic_, std::string("/octomap_binary"));
  nodeHandle.param("min_x", minX_, NAN);
  nodeHandle.param("max_x", maxX_, NAN);
  nodeHandle.param("min_y", minY_, NAN);
  nodeHandle.param("max_y", maxY_, NAN);
  nodeHandle.param("min_z", minZ_, NAN);
  nodeHandle.param("max_z", maxZ_, NAN);
  return true;
}

void TopologyMap::convertAndPublishMap()
{
  octomap_msgs::GetOctomap srv;
  if (!client_.call(srv)) {
    ROS_ERROR_STREAM("Failed to call service: " << octomapServiceTopic_);
    return;
  }

  // creating octree
  octomap::OcTree* octomap = nullptr;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(srv.response.map);
  if (tree) {
    octomap = dynamic_cast<octomap::OcTree*>(tree);
  } else {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
  if(!std::isnan(minX_))
    min_bound(0) = minX_;
  if(!std::isnan(maxX_))
    max_bound(0) = maxX_;
  if(!std::isnan(minY_))
    min_bound(1) = minY_;
  if(!std::isnan(maxY_))
    max_bound(1) = maxY_;
  if(!std::isnan(minZ_))
    min_bound(2) = minZ_;
  if(!std::isnan(maxZ_))
    max_bound(2) = maxZ_;
  bool res = grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map_, &min_bound, &max_bound);
  if (!res) {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }
  map_.setFrameId(srv.response.map.header.frame_id);

  // Publish as grid map.
  grid_map_msgs::GridMap gridMapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
  gridMapPublisher_.publish(gridMapMessage);

  // Also publish as an octomap msg for visualization
  octomap_msgs::Octomap octomapMessage;
  octomap_msgs::fullMapToMsg(*octomap, octomapMessage);
  octomapMessage.header.frame_id = map_.getFrameId();
  octomapPublisher_.publish(octomapMessage);
}

} /* namespace */


