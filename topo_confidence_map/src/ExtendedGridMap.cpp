#include "ExtendedGridMap.h"

namespace topology_map {


int ExtendedGM::iGridRawNum = 0;//Placeholder in ExtendedGridMap.h


ExtendedGM::ExtendedGM(){


}


ExtendedGM::~ExtendedGM(){



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

int ExtendedGM::PointoOneDIdx(const pcl::PointXYZ & oPoint,
	                          const grid_map::GridMap & oFeatureMap) {

	grid_map::Position oPosition(oPoint.x, oPoint.y);

	//get the 2d index of corresponding grid
	grid_map::Index oIndex;
	oFeatureMap.getIndex(oPosition, oIndex);

	//get the 1d index of point
	//function: ix * size(cols) + iy
	return oIndex(0) * iGridRawNum + oIndex(1);

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

MapIndex ExtendedGM::PointoAllTypeIdx(const pcl::PointXYZ & oPoint,
	                                         const grid_map::GridMap & oFeatureMap) {

	MapIndex oAllTypeIdx;

	grid_map::Position oPosition(oPoint.x, oPoint.y);

	//get the 2d index of corresponding grid
	oFeatureMap.getIndex(oPosition, oAllTypeIdx.oTwoIndex);

	//get the 1d index of point
	//function: ix * size(cols) + iy
	oAllTypeIdx.iOneIdx = oAllTypeIdx.oTwoIndex(0) * iGridRawNum + oAllTypeIdx.oTwoIndex(1);

	return oAllTypeIdx;

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

int ExtendedGM::TwotoOneDIdx(const grid_map::Index & oIndex) {

	//get the 1d index of point
	//function: ix * size(cols) + iy
	return oIndex(0) * iGridRawNum + oIndex(1);

}

//reload TwotoOneDIdx function
int ExtendedGM::TwotoOneDIdx(const int & iIndexX, const int & iIndexY){
    //function: ix * size(cols) + iy
	return iIndexX * iGridRawNum + iIndexY;

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

void ExtendedGM::OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,
	const int & iOneDIdx) {

	oTwoDIdx(1) = iOneDIdx % iGridRawNum;
	oTwoDIdx(0) = (iOneDIdx - oTwoDIdx(1)) / iGridRawNum;

}


}