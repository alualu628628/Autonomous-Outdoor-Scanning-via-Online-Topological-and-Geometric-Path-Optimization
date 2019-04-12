#ifndef EXTENDEDGRIDMAP_H 
#define EXTENDEDGRIDMAP_H 

#include <cmath>
#include <vector>

//pcl related
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

//grid_map related
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>


namespace topology_map {



//index of grid cell
struct MapIndex{

  //2D index 
  grid_map::Index oTwoIndex;//cell index in a matrix
  //1D index
  int iOneIdx; //cell index in a sequance

};

class ExtendedGM{

public:

	ExtendedGM();

	~ExtendedGM();

	//transform point position to 1 Dimension index
	static int PointoOneDIdx(const pcl::PointXYZ & oPoint,
		                     const grid_map::GridMap & oFeatureMap);

	//transform point position to 1D and 2D indexes
	static MapIndex PointoAllTypeIdx(const pcl::PointXYZ & oPoint,
		                             const grid_map::GridMap & oFeatureMap);

	//transform 2D index to 1D index
	static int TwotoOneDIdx(const grid_map::Index & oIndex);
	static int TwotoOneDIdx(const int & iIndexX, 
		                    const int & iIndexY);

	//transform 1D index to 2D indexes
	static void OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,
		                            const int & iOneDIdx);


	//grid number in raws
	static int iGridRawNum;

private:



};


}

#endif