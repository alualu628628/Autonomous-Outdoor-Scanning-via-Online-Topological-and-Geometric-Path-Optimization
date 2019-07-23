#include "ExtendedGridMap.h"

namespace topology_map {


int ExtendedGM::iGridRawNum = 0;//Placeholder in ExtendedGridMap.h


/*************************************************
Function: ExtendedGM
Description: constrcution function for ExtendedGM class
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: Initial naming the grid_map layer as "elevation"
*************************************************/
ExtendedGM::ExtendedGM():m_oFeatureMap({ "elevation" }){



}

/*************************************************
Function: ExtendedGM
Description: destrcution function for ExtendedGM class
Calls: none
Called By: none
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
ExtendedGM::~ExtendedGM(){



}


/*************************************************
Function: GetParam
Description: get parameter value for private variances to generate a grid map
Calls: none
Called By: external call
Table Accessed: none
Table Updated: none
Input: dMapMaxRange - the half of length of map (map is 2d square)
	   dResolution - the resolution of map
	   dMinMapZ - the minimum elevation value the map can received
	   dMaxMapZ - the maximum elevation value the map can received
	   sMapFrameID - the robot structure name (called frame in ros) of map
Output: get value
Return: none
Others: none
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
Function: GenerateMap
Description: generate a grid map
Calls: none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oRobotPos - the current robot position (odometry)
Output: none
Return: none
Others: none
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
	m_oFeatureMap.add("quality", grid_map::Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));

}



/*************************************************
Function: PointoOneDIdx
Description: convert a point coordinate to a 1D grid index
             where 1D means the index is a linear sequence in grid_map
             //index(1D) = ix * size(cols) + iy
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oPoint - point position
	   oFeatureMap - the grid_map
Output: 1d index
Return: a int index variance
Others: none
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
Function: PointoAllTypeIdx
Description: convert a point coordinate to a 1D and 2D grid index
             where 1D means the index is a linear sequence in grid_map
             and 2D index consists of a row number and a column number
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oPoint - point position
	   oFeatureMap - the grid_map
Output: 1d and 2d indinces
Return: a MapIndex tyed index variance, which consists of 1d and 2d index
Others: none
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
Description: convert 1D grid index to a point coordinate
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oPoint - a point position
       iQueryIdx - a 1d grid index
       oFeatureMap - the grid map
Output: oPoint - query grid center position
Return: none
Others: none
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
Description: convert 2D grid index to a point coordinate
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oPoint - a point position
       oGridIdx - a 2d grid index
       oFeatureMap - the grid map
Output: oPoint - query grid center position
Return: none
Others: none
*************************************************/
void ExtendedGM::TwoDIdxtoPoint(pcl::PointXYZ & oPoint,
    	                       const grid_map::Index & oGridIdx,
		                       const grid_map::GridMap & oFeatureMap){

	//find the 1d and 2d index
	grid_map::Position oGridPos;
	oFeatureMap.getPosition(oGridIdx, oGridPos);

	//get position in pcl type
	oPoint.x = oGridPos(0);
	oPoint.y = oGridPos(1);

}
//reload a grid_map::Position type output
void ExtendedGM::TwoDIdxtoPoint(grid_map::Position & oPointPos,
    	                       const grid_map::Index & oGridIdx,
		                       const grid_map::GridMap & oFeatureMap){

	//get the position from map
	oFeatureMap.getPosition(oGridIdx, oPointPos);

}


/*************************************************
Function: TwotoOneDIdx
Description: convert 2D grid index to 1D grid index
Calls: all none
Called By: external call
           CircleNeighborhood()
Table Accessed: none
Table Updated: none
Input: oIndex - 2D index
Output: a 1D index
Return: a int type variance
Others: none
*************************************************/
int ExtendedGM::TwotoOneDIdx(const grid_map::Index & oIndex) {

	//get the 1d index of point
	//function: ix * size(cols) + iy
	return oIndex(0) * iGridRawNum + oIndex(1);

}
//reload TwotoOneDIdx function with a individal x, y input
int ExtendedGM::TwotoOneDIdx(const int & iIndexX, const int & iIndexY){
    //function: ix * size(cols) + iy
	return iIndexX * iGridRawNum + iIndexY;

}


/*************************************************
Function: OneDtoTwoDIdx
Description: convert 1D grid index to 2D grid index
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: iOneDIdx - 1D grid index
Output: oTwoDIdx - 2D gird index
Return: none
Others: none
*************************************************/
void ExtendedGM::OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,
	                           const int & iOneDIdx) {

	oTwoDIdx(1) = iOneDIdx % iGridRawNum;
	oTwoDIdx(0) = (iOneDIdx - oTwoDIdx(1)) / iGridRawNum;

}



/*************************************************
Function: GenerateCircleMask
Description: generate a circle mask for searching
             this generated mask should be maintained in memory so that it dont need a repeatl generation
             the value of mask gird is in local 
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: dRadius - the radius of circle mask
Output: vSearchMask - a mask with local grid index related to the center grid
Return: vSearchMask - a MapIndex type vector
Others: none
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
Function: CircleNeighborhood
Description: search neighborhood index (1d and 2d together) of a given query point
Calls: TwotoOneDIdx()
Called By: external call
Table Accessed: none
Table Updated: none
Input: oFeatureMap - the grid_map
	   vSearchMask - the generated searhing mask
	   oQueryPoint - a query point position
Output: vNearbyGrids - the grid index which is near the robot
Return: none
Others: none
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
			continue;//if the neighboring grid is outside of the map
        //get nearby grid idx on y axis
		oOneNearGridIdx.oTwoIndex(1) = oQueryIdx(1) + vSearchMask[i].oTwoIndex(1);
		if (oOneNearGridIdx.oTwoIndex(1) < 0 || oOneNearGridIdx.oTwoIndex(1) >= oFeatureMap.getSize()(1))
			continue;//if the neighboring grid is outside of the map

		oOneNearGridIdx.iOneIdx = TwotoOneDIdx(oOneNearGridIdx.oTwoIndex);
		//compute the nearby grid
		vNearbyGrids.push_back(oOneNearGridIdx);

	}

}
//reload with grid index input and 1d index output
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
Function: CheckInSidePoint
Description: check whether a point is inside the map (grid map)
Calls: all none
Called By: external call
Table Accessed: none
Table Updated: none
Input: oPoint - a query point
Output: a flag indicates the point is inside the map (true) or not (false)
Return: a bool variance
Others: none
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