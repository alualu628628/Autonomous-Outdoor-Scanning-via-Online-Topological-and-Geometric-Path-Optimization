/*====================================================================
 // OctoGrid.h, modified by the GridOctoConverter.hpp
 //
 // Created on: May 1, 2017, Author: Jeff Delmerico, Peter Fankhauser,  University of Zürich
 // Modified on: April 3, 2019, Author: Pengdi Huang
 =====================================================================*/

#ifndef OCTOGRID_H
#define OCTOGRID_H

//sd
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

//pcl -point cloud library in ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

//grid _map
#include <grid_map_core/grid_map_core.hpp>

//octomap
#include <octomap/octomap.h>



namespace topology_map {

/*!
 * Conversions between grid maps and Octomap types.
 */
class GridOctoConverter{

  public:
  /*!
   * Default constructor.
   */
  GridOctoConverter();

  /*!
   * Destructor.
   */
  virtual ~GridOctoConverter();

  /*!
   * Converts an Octomap to a grid map in the same coordinate frame, with a
   * cell resolution equal to the leaf voxel size in the Octomap. Only creates
   * a layer for elevation.
   * This changes the geometry of the grid map and deletes all layer contents.
   * Note: Bounding box coordinates are not checked for sanity - if you provide
   * values outside of the gridmap, undefined behavior may result.
   * @param[in] octomap the octomap.
   * @param[in] layer the layer that is filled with the octomap data.
   * @param[out] gridMap the grid map to be initialized.
   * @param[out] vMapPointIndex the node index in map cell.
   * @param[out] vCloud the octree node point clouds.
   * @param[in] minPoint (optional) minimum coordinate for bounding box.
   * @param[in] maxPoint (optional) maximum coordinate for bounding box.
   * @return true if successful, false otherwise.
   */
  static bool FromOctomap(const octomap::OcTree& octomap,
                          const std::string& layer,
                          grid_map::GridMap& gridMap,
                          std::vector<std::vector<std::vector<int>>> & vMapPointIndex,
                          pcl::PointCloud<pcl::PointXYZ> & vCloud,
                          const grid_map::Position3* minPoint = nullptr,
                          const grid_map::Position3* maxPoint = nullptr);

};

} /* namespace */

#endif