#include "ExtendedGridMap.h"

namespace topology_map {


int ExtendedGM::iGridRawNum = 0;//Placeholder in ExtendedGridMap.h


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
ExtendedGM::ExtendedGM():m_oFeatureMap({ "elevation" }){



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

void ExtendedGM::GetParam(const double & dMapMaxRange,
	                       const double & dResolution,
	                          const double & dMinMapZ,
	                          const double & dMaxMapZ,
	                  const std::string & sMapFrameID){

	//get input
	m_dMapMaxRange = dMapMaxRange;//map range, map size

	m_dMapResolution = dResolution;//map resolution, cell size

	//z is initialized in ReadParameters()	
	m_oMinCorner(2) = dMinMapZ;
	m_oMaxCorner(2) = dMaxMapZ;

	//frame id
	m_oFeatureMap.setFrameId(sMapFrameID);

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
void ExtendedGM::GenerateMap(const pcl::PointXYZ & oRobotPos){

	m_oMapOriginalPos(0) = oRobotPos.x;
	m_oMapOriginalPos(1) = oRobotPos.y;
	m_oMapOriginalPos(2) = oRobotPos.z;

	//corner of map's bounding box
	m_oMinCorner(0) = m_oMapOriginalPos(0) - float(m_dMapMaxRange);
	m_oMaxCorner(0) = m_oMapOriginalPos(0) + float(m_dMapMaxRange);
	m_oMinCorner(1) = m_oMapOriginalPos(1) - float(m_dMapMaxRange);
	m_oMaxCorner(1) = m_oMapOriginalPos(1) + float(m_dMapMaxRange);
	m_oMinCorner(2) = m_oMapOriginalPos(2) + m_oMinCorner(2);
	m_oMaxCorner(2) = m_oMapOriginalPos(2) + m_oMaxCorner(2);
	//z is initialized in ReadParameters()

	//build the map
	m_oFeatureMap.setGeometry(grid_map::Length(2.0*m_dMapMaxRange, 2.0*m_dMapMaxRange),
		                                                              m_dMapResolution, 
		                                                             m_oMapOriginalPos);

	//dispaly
	ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", m_oFeatureMap.getLength().x(),
			m_oFeatureMap.getLength().y(), m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1));
	

	//set feature layer
	m_oFeatureMap.setBasicLayers({"elevation"});

	//add other map layer
	m_oFeatureMap.add("traversability", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	m_oFeatureMap.add("boundary", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	m_oFeatureMap.add("observability", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	m_oFeatureMap.add("confidence", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	m_oFeatureMap.add("travelable", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	m_oFeatureMap.add("label", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));

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
Function: OneDIdxtoPoint
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
void  ExtendedGM::OneDIdxtoPoint(pcl::PointXYZ & oPoint,
    	                          const int & iQueryIdx,
    	          const grid_map::GridMap & oFeatureMap){

	grid_map::Index oGridIdx;
	ExtendedGM::OneDtoTwoDIdx(oGridIdx, iQueryIdx);

	//find the 1d and 2d index
	grid_map::Position oGridPos;
	oFeatureMap.getPosition(oGridIdx, oGridPos);

	oPoint.x = oGridPos(0);
	oPoint.y = oGridPos(1);

}
//reload a grid_map::Position type point output
void  ExtendedGM::OneDIdxtoPoint(grid_map::Position & oPointPos,
    	                                  const int & iQueryIdx,
    	                  const grid_map::GridMap & oFeatureMap){

	grid_map::Index oGridIdx;
	ExtendedGM::OneDtoTwoDIdx(oGridIdx, iQueryIdx);

	//find the 1d and 2d index
	oFeatureMap.getPosition(oGridIdx, oPointPos);

}


/*************************************************
Function: TwoDIdxtoPoint
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

void ExtendedGM::TwoDIdxtoPoint(pcl::PointXYZ & oPoint,
    	                       const grid_map::Index & oGridIdx,
		                       const grid_map::GridMap & oFeatureMap){

	//find the 1d and 2d index
	grid_map::Position oGridPos;
	oFeatureMap.getPosition(oGridIdx, oGridPos);

	//get position in pcl type
	oPoint.x = oGridIdx(0);
	oPoint.y = oGridIdx(1);

}
//reload a grid_map::Position type output
void ExtendedGM::TwoDIdxtoPoint(grid_map::Position & oPointPos,
    	                       const grid_map::Index & oGridIdx,
		                       const grid_map::GridMap & oFeatureMap){

	//get the position from map
	oFeatureMap.getPosition(oGridIdx, oPointPos);

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

/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
privare nodem_dMapMaxRange
flag of generating output file
original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

std::vector<MapIndex> ExtendedGM::GenerateCircleMask(const double & dRadius) {

	//define output
	std::vector<MapIndex> vSearchMask;

	//fRadiusGridsNum means how many grid in the radius axis 
	float fRadiusGridsNum = dRadius / m_dMapResolution;

	//actual raduis grids number
	float fRealRGNum = fRadiusGridsNum + 0.5;
	float fMaskStartGrid = float(floor(-fRadiusGridsNum));
	float fMaskEndGrid = float(ceil(fRadiusGridsNum));

	//in the square area (2d index)
	//for x id
	for (float i = fMaskStartGrid; i <= fMaskEndGrid; i = i + 1.0) {
		//for y id
		for (float j = fMaskStartGrid; j <= fMaskEndGrid; j = j + 1.0) {
			//put into mask members
			if (sqrt(pow(i, 2.0f) + pow(j, 2.0f)) <= fRealRGNum) {
				//the output index is in mask coordiante system
				MapIndex oMaskMemberIdx;
				oMaskMemberIdx.oTwoIndex(0) = int(i);
				oMaskMemberIdx.oTwoIndex(1) = int(j);
				//the index should be tranfored to global coordinate based on query grid when use mask
				vSearchMask.push_back(oMaskMemberIdx);
			}//end if sqrt
		}//end j
	}//end i

	return vSearchMask;

}


/*************************************************
Function: TopologyMap
Description: constrcution function for TopologyMap class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
privare nodem_dMapMaxRange
flag of generating output file
original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

void ExtendedGM::CircleNeighborhood(std::vector<MapIndex> & vNearbyGrids,
	                               const grid_map::GridMap & oFeatureMap,
	                           const std::vector<MapIndex> & vSearchMask,
	                                   const pcl::PointXYZ & oQueryPoint) {

	vNearbyGrids.clear();

	grid_map::Position oQueryPos(oQueryPoint.x, oQueryPoint.y);
	grid_map::Index oQueryIdx;
	oFeatureMap.getIndex(oQueryPos, oQueryIdx);

	//seach in given neighboring grids
	for (int i = 0; i != vSearchMask.size(); ++i) {

		MapIndex oOneNearGridIdx;
		//get nearby grid idx on x
		oOneNearGridIdx.oTwoIndex(0) = oQueryIdx(0) + vSearchMask[i].oTwoIndex(0);
		if (oOneNearGridIdx.oTwoIndex(0) < 0 || oOneNearGridIdx.oTwoIndex(0) >= oFeatureMap.getSize()(0))
			break;//if the neighboring grid is outside of the map
        //get nearby grid idx on y axis
		oOneNearGridIdx.oTwoIndex(1) = oQueryIdx(1) + vSearchMask[i].oTwoIndex(1);
		if (oOneNearGridIdx.oTwoIndex(1) < 0 || oOneNearGridIdx.oTwoIndex(1) >= oFeatureMap.getSize()(1))
			break;//if the neighboring grid is outside of the map

		oOneNearGridIdx.iOneIdx = TwotoOneDIdx(oOneNearGridIdx.oTwoIndex);
		//compute the nearby grid
		vNearbyGrids.push_back(oOneNearGridIdx);

	}

}
//reload 
void ExtendedGM::CircleNeighborhood(std::vector<int> & vNearbyGrids,
	                          const grid_map::GridMap & oFeatureMap,
	                      const std::vector<MapIndex> & vSearchMask,
	                                      const int & iQueryGridIdx) {
						                           
		                                       
    //clear output
	vNearbyGrids.clear();

    //get the two dimension index
	grid_map::Index oQueryIdx;
	OneDtoTwoDIdx(oQueryIdx,iQueryGridIdx);

	//seach in given neighboring grids
	for (int i = 0; i != vSearchMask.size(); ++i) {

		MapIndex oOneNearGridIdx;
		//get nearby grid idx on x
		oOneNearGridIdx.oTwoIndex(0) = oQueryIdx(0) + vSearchMask[i].oTwoIndex(0);
		if (oOneNearGridIdx.oTwoIndex(0) < 0 || oOneNearGridIdx.oTwoIndex(0) >= oFeatureMap.getSize()(0))
			break;//if the neighboring grid is outside of the map
        //get nearby grid idx on y axis
		oOneNearGridIdx.oTwoIndex(1) = oQueryIdx(1) + vSearchMask[i].oTwoIndex(1);
		if (oOneNearGridIdx.oTwoIndex(1) < 0 || oOneNearGridIdx.oTwoIndex(1) >= oFeatureMap.getSize()(1))
			break;//if the neighboring grid is outside of the map

		int iOneNearIdx = TwotoOneDIdx(oOneNearGridIdx.oTwoIndex);
		//compute the nearby grid
		vNearbyGrids.push_back(iOneNearIdx);

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
bool ExtendedGM::CheckInSidePoint(const pcl::PointXYZ & oPoint) {

	//if smaller than minimum corner
	if (oPoint.x - m_oMinCorner(0) <= 0)
		return false;
	if (oPoint.y - m_oMinCorner(1) <= 0)
		return false;
	if (oPoint.z - m_oMinCorner(2) <= 0)
		return false;
	//if larger than maximum corner
	if (oPoint.x - m_oMaxCorner(0) >= 0)
		return false;
	if (oPoint.y - m_oMaxCorner(1) >= 0)
		return false;
	if (oPoint.z - m_oMaxCorner(2) >= 0)
		return false;

	//point is inside map region
	return true;

}




}/*name space*/