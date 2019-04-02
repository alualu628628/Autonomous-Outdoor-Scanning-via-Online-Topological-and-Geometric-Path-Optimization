
#ifndef TOPOLOGYMAP_H
#define TOPOLOGYMAP_H
#include <cmath>
#include <string>

//ros related
#include <ros/ros.h>

//octomap related
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

//grid_map related
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>//transfor octomap to grid

using namespace grid_map;

namespace topology_map{

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation layer.
 * The grid map is published and can be viewed in Rviz.
 */
class TopologyMap
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TopologyMap(ros::NodeHandle & node,
              ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TopologyMap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters(ros::NodeHandle & nodeHandle);

  void convertAndPublishMap();

 private:

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Octomap publisher.
  ros::Publisher octomapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.
  std::string octomapServiceTopic_;

  //! Octomap service client
  ros::ServiceClient client_;

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

} /* namespace */


#endif