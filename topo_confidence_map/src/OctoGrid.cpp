/*====================================================================
 // Modified by the GridOctoConverter.cpp
 // Created on: May 1, 2017, Author: Jeff Delmerico, Peter Fankhauser,  University of Zürich
 // Modified on: April 3, 2019, Author: Pengdi Huang
 =====================================================================*/

#include "OctoGrid.h"

namespace topology_map{

GridOctoConverter::GridOctoConverter(){

}

GridOctoConverter::~GridOctoConverter(){

}

bool GridOctoConverter::UpdateFromOctomap(const octomap::OcTree& octomap,
                                                const std::string& layer,
                                             grid_map::GridMap& oGridMap,
             std::vector<std::vector<std::vector<int>>> & vMapPointIndex,
                                 pcl::PointCloud<pcl::PointXYZ> & vCloud,
                              const grid_map::Position3* oMinCornerGrMap,
                              const grid_map::Position3* oMaxCornerGrMap){

  if (octomap.getTreeType() != "OcTree") {
    std::cerr << "Octomap conversion only implemented for standard OcTree type." << std::endl;
    return false;
  }

  // Copy octomap in order to expand any pruned occupied cells and maintain constness of input.
  octomap::OcTree octomapCopy(octomap);

  // Iterate through leaf nodes and project occupied cells to elevation map.
  // On the first pass, expand all occupied cells that are not at maximum depth.
  unsigned int max_depth = octomapCopy.getTreeDepth();

  // Adapted from octomap octree2pointcloud.cpp.
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (octomap::OcTree::iterator it = octomapCopy.begin(); it != octomapCopy.end(); ++it) {
      if (octomapCopy.isNodeOccupied(*it) && it.getDepth() < max_depth) {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin();
                                            it != collapsed_occ_nodes.end(); ++it) {
      #if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
        (*it)->expandNode();
      #else
        octomapCopy.expandNode(*it);
      #endif
    }
    // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
  } while (collapsed_occ_nodes.size() > 0);

  // Set up grid map geometry.
  // TODO Figure out whether to center map.
  //double resolution = octomapCopy.getResolution();
  grid_map::Position3 minOcBound;
  grid_map::Position3 maxOcBound;
  octomapCopy.getMetricMin(minOcBound(0), minOcBound(1), minOcBound(2));
  octomapCopy.getMetricMax(maxOcBound(0), maxOcBound(1), maxOcBound(2));

  grid_map::Length oOcLength = grid_map::Length(maxOcBound(0) - minOcBound(0), maxOcBound(1) - minOcBound(1));
  
  //less points will be negected
  if (!(octomapCopy.size() && oOcLength(0) && oOcLength(1))){

    return false;

  }

  octomap::point3d oQueryMinBbx = octomap::point3d((*oMinCornerGrMap)(0), (*oMinCornerGrMap)(1), (*oMinCornerGrMap)(2));
  octomap::point3d oQueryMaxBbx = octomap::point3d((*oMaxCornerGrMap)(0), (*oMaxCornerGrMap)(1), (*oMaxCornerGrMap)(2));


  // Add elevation layer
  //oGridMap.add(layer);
  //oGridMap.setBasicLayers({layer});

  //clear old data
  for(int i = 0; i != vMapPointIndex.size(); ++i){

    for(int j = 0; j != vMapPointIndex[i].size(); ++j){

      vMapPointIndex[i][j].clear();//clear the point index in grid map

    }//end for j

  }//end for i
 
  vCloud.clear(); 
  
  int iNodePointCount = 0;

  // For each voxel, if its elevation is higher than the existing value for the
  // corresponding grid map cell, overwrite it.
  // std::cout << "Iterating from " << min_bbx << " to " << max_bbx << std::endl;
  grid_map::Matrix& gridMapData = oGridMap[layer];
  for(octomap::OcTree::leaf_bbx_iterator it = octomapCopy.begin_leafs_bbx(oQueryMinBbx, oQueryMaxBbx),
          end = octomapCopy.end_leafs_bbx(); it != end; ++it) {

    if (octomapCopy.isNodeOccupied(*it)) {
      octomap::point3d octoPos = it.getCoordinate();
      grid_map::Position position(octoPos.x(), octoPos.y());

     
      //get the point position
      pcl::PointXYZ oNodePoint;
      oNodePoint.x = octoPos.x();
      oNodePoint.y = octoPos.y();
      oNodePoint.z = octoPos.z();
      
      //get the 2d index of corresponding grid
      grid_map::Index index;
      oGridMap.getIndex(position, index);
      
      //get the 1d index of point
      vCloud.push_back(oNodePoint);
      vMapPointIndex[index(0)][index(1)].push_back(iNodePointCount);
      iNodePointCount++;
      
      // If no elevation has been set, use current elevation.
      if (!oGridMap.isValid(index)) {
        gridMapData(index(0), index(1)) = octoPos.z();
      
      // Check existing elevation, keep higher.
      }else {

        if (gridMapData(index(0), index(1)) < octoPos.z())
          gridMapData(index(0), index(1)) = octoPos.z();

      }//end else

    }//end if

  }//end for

  return true;
}


} /* namespace */



/*==============================raw code backup==================================
bool GridOctoConverter::FromOctomap(const octomap::OcTree& octomap,
                                          const std::string& layer,
                                        grid_map::GridMap& gridMap,
       std::vector<std::vector<std::vector<int>>> & vMapPointIndex,
                           pcl::PointCloud<pcl::PointXYZ> & vCloud,
                               const grid_map::Position3* oMinCornerGrMap,
                               const grid_map::Position3* oMaxCornerGrMap){

  if (octomap.getTreeType() != "OcTree") {
    std::cerr << "Octomap conversion only implemented for standard OcTree type." << std::endl;
    return false;
  }

  // Copy octomap in order to expand any pruned occupied cells and maintain constness of input.
  octomap::OcTree octomapCopy(octomap);

  // Iterate through leaf nodes and project occupied cells to elevation map.
  // On the first pass, expand all occupied cells that are not at maximum depth.
  unsigned int max_depth = octomapCopy.getTreeDepth();

  // Adapted from octomap octree2pointcloud.cpp.
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (octomap::OcTree::iterator it = octomapCopy.begin(); it != octomapCopy.end(); ++it) {
      if (octomapCopy.isNodeOccupied(*it) && it.getDepth() < max_depth) {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin();
                                            it != collapsed_occ_nodes.end(); ++it) {
      #if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
        (*it)->expandNode();
      #else
        octomapCopy.expandNode(*it);
      #endif
    }
    // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
  } while (collapsed_occ_nodes.size() > 0);

  // Set up grid map geometry.
  // TODO Figure out whether to center map.
  double resolution = octomapCopy.getResolution();
  grid_map::Position3 minBound;
  grid_map::Position3 maxBound;
  octomapCopy.getMetricMin(minBound(0), minBound(1), minBound(2));
  octomapCopy.getMetricMax(maxBound(0), maxBound(1), maxBound(2));

  // User can provide coordinate limits to only convert a bounding box.
  octomap::point3d minBbx(minBound(0), minBound(1), minBound(2));
  if (oMinCornerGrMap) {
    minBbx = octomap::point3d((*oMinCornerGrMap)(0), (*oMinCornerGrMap)(1), (*oMinCornerGrMap)(2));
    minBound = grid_map::Position3(minBbx.x(), minBbx.y(), minBbx.z());
  }
  octomap::point3d maxBbx(maxBound(0), maxBound(1), maxBound(2));
  if (oMaxCornerGrMap) {
    maxBbx = octomap::point3d((*oMaxCornerGrMap)(0), (*oMaxCornerGrMap)(1), (*oMaxCornerGrMap)(2));
    maxBound = grid_map::Position3(maxBbx.x(), maxBbx.y(), maxBbx.z());
  }

  grid_map::Length length = grid_map::Length(maxBound(0) - minBound(0), maxBound(1) - minBound(1));
  grid_map::Position position = grid_map::Position((maxBound(0) + minBound(0)) / 2.0,
                                                   (maxBound(1) + minBound(1)) / 2.0);
  

  if (!(octomapCopy.size() && length(0) && length(1))){

    return false;

  }

  gridMap.setGeometry(length, resolution, position);
  // std::cout << "grid map geometry: " << std::endl;
  // std::cout << "Length: [" << length(0) << ", " << length(1) << "]" << std::endl;
  // std::cout << "Position: [" << position(0) << ", " << position(1) << "]" << std::endl;
  // std::cout << "Resolution: " << resolution << std::endl;
  
  // Add elevation layer
  gridMap.add(layer);
  gridMap.setBasicLayers({layer});

  vMapPointIndex.clear();
  vCloud.clear();

  //set the point index vector with same sizes
  if(gridMap.getSize()(0) && gridMap.getSize()(1)){
    
    std::vector<int> vOneGridIdx;
    std::vector<std::vector<int>> vColGridVec;

    for(int i = 0; i != gridMap.getSize()(1); ++i)
      vColGridVec.push_back(vOneGridIdx);


    for(int i = 0; i != gridMap.getSize()(0); ++i)
      vMapPointIndex.push_back(vColGridVec);

     
  }//end if
 
  
  int iNodePointCount = 0;

  // For each voxel, if its elevation is higher than the existing value for the
  // corresponding grid map cell, overwrite it.
  // std::cout << "Iterating from " << min_bbx << " to " << max_bbx << std::endl;
  grid_map::Matrix& gridMapData = gridMap[layer];
  for(octomap::OcTree::leaf_bbx_iterator it = octomapCopy.begin_leafs_bbx(minBbx, maxBbx),
          end = octomapCopy.end_leafs_bbx(); it != end; ++it) {

    if (octomapCopy.isNodeOccupied(*it)) {
      octomap::point3d octoPos = it.getCoordinate();
      grid_map::Position position(octoPos.x(), octoPos.y());

     
      //get the point position
      pcl::PointXYZ oNodePoint;
      oNodePoint.x = octoPos.x();
      oNodePoint.y = octoPos.y();
      oNodePoint.z = octoPos.z();
      
      //get the 2d index of corresponding grid
      grid_map::Index index;
      gridMap.getIndex(position, index);
      
      //get the 1d index of point
      vCloud.push_back(oNodePoint);
      vMapPointIndex[index(0)][index(1)].push_back(iNodePointCount);
      iNodePointCount++;
      
      // If no elevation has been set, use current elevation.
      if (!gridMap.isValid(index)) {
        gridMapData(index(0), index(1)) = octoPos.z();
      }
      // Check existing elevation, keep higher.
      else {
        if (gridMapData(index(0), index(1)) < octoPos.z()) {
          gridMapData(index(0), index(1)) = octoPos.z();
        }
      }
    }
  }

  return true;
}
===========================================================*/

