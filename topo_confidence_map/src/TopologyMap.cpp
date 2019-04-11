#include "TopologyMap.h"


namespace topology_map {


//*********************************Initialization function*********************************
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
TopologyMap::TopologyMap(ros::NodeHandle & node,
	                     ros::NodeHandle & nodeHandle) :m_oFeatureMap(grid_map::GridMap({ "elevation" })),
	                     m_vBoundCloud(new pcl::PointCloud<pcl::PointXYZ>),
	                     m_vObstacleCloud(new pcl::PointCloud<pcl::PointXYZ>),
	                     m_iTrajFrameNum(0),
	                     m_iGroundFrames(0),
	                     m_iBoundFrames(0),
	                     m_iObstacleFrames(0),
	                     m_iOdomSampingNum(25),
	                     m_bGridMapReadyFlag(false) {

	//read parameters
	ReadParameters(nodeHandle);

	//generate circle mask region based on the given local searching radius and resolution
	GenerateCircleMask();

	//subscribe topic 
	//m_oOctoMapClient = nodeHandle.serviceClient<octomap_msgs::GetOctomap>(m_oOctomapServiceTopic);

	//subscribe (hear) the odometry information
	m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &TopologyMap::HandleTrajectory, this);

	//subscribe (hear) the point cloud topic with different classifications from ground extratcion 
	m_oGroundSuber = nodeHandle.subscribe(m_sGroundTopic, 1, &TopologyMap::HandleGroundClouds, this);
	m_oBoundSuber = nodeHandle.subscribe(m_sBoundTopic, 1, &TopologyMap::HandleBoundClouds, this);
	m_oObstacleSuber = nodeHandle.subscribe(m_sObstacleTopic, 1, &TopologyMap::HandleObstacleClouds, this);

	//publish topic
	m_oGridMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("test_clouds", 1, true);

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

TopologyMap::~TopologyMap() {

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

bool TopologyMap::ReadParameters(ros::NodeHandle & nodeHandle) {
	//input topic
	nodeHandle.param("odom_in_topic", m_sOdomTopic, std::string("/odometry/filtered"));

	nodeHandle.param("ground_pc_topic", m_sGroundTopic, std::string("/ground_points"));

	nodeHandle.param("boundary_pc_topic", m_sBoundTopic, std::string("/boundary_points"));

	nodeHandle.param("obstacle_pc_topic", m_sObstacleTopic, std::string("/obstacle_points"));

	//intput service
	//nodeHandle.param("octomap_service_topic", m_oOctomapServiceTopic, std::string("/octomap_binary"));

	//frequncy
	nodeHandle.param("odometry_rawfreq", m_dOdomRawHz, 50.0);

	nodeHandle.param("odomsampling_freq", m_dSamplingHz, 2.0);

	m_iOdomSampingNum = int(m_dOdomRawHz / m_dSamplingHz);
	ROS_INFO("Set odometry down sampling times as [%d]", m_iOdomSampingNum);

	//
	//nodeHandle.param("freshoctomap_freq", m_dOctomapFreshHz, 0.5);

	//m_iMapFreshNum = int(m_dOdomRawHz / m_dOctomapFreshHz); 
	//ROS_INFO("Set updating octomap times as [%d]",m_iMapFreshNum);

	//point cloud sampling number
	nodeHandle.param("pointframe_smplnum", m_iPCSmplNum, 1);

	//**map parameters**
	//map neighboring region size
	nodeHandle.param("robot_local_r", m_dRbtLocalRadius, 5.0);

	//map range/map size
	nodeHandle.param("gridmap_maxrange", m_dMapMaxRange, 250.0);
	if (m_dMapMaxRange <= 0)//defend a zero input
		m_dMapMaxRange = 50.0;

	//map id
	nodeHandle.param("gridmap_frameid", m_sMapFrameID, std::string("map"));

	//map limited height
	double dMinMapZ;
	nodeHandle.param("min_mapz", dMinMapZ, -2.0);
	m_oMinCorner(2) = dMinMapZ;

	double dMaxMapZ;
	nodeHandle.param("max_mapz", dMaxMapZ, 7.0);
	m_oMaxCorner(2) = dMaxMapZ;

	//map cell size
	nodeHandle.param("gridmap_resolution", m_dResolution, 0.1);

	return true;

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

void TopologyMap::InitializeGridMap(const float & fRobotX,
	const float & fRobotY) {

	//corner of map's bounding box
	m_oMinCorner(0) = fRobotX - float(m_dMapMaxRange);
	m_oMaxCorner(0) = fRobotX + float(m_dMapMaxRange);
	m_oMinCorner(1) = fRobotY - float(m_dMapMaxRange);
	m_oMaxCorner(1) = fRobotY + float(m_dMapMaxRange);
	//z is initialized in ReadParameters()

	//build the map
	m_oFeatureMap.setGeometry(Length(2.0*m_dMapMaxRange, 2.0*m_dMapMaxRange),
		m_dResolution, Position(fRobotX, fRobotY));

	//dispaly
	ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", m_oFeatureMap.getLength().x(),
			m_oFeatureMap.getLength().y(), m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1));

	//set feature layer
	//m_oFeatureMap.setBasicLayers({"elevation"});

	//add other map layer
	//m_oFeatureMap.add("traversability", Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	//m_oFeatureMap.add("boundary", Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	//m_oFeatureMap.add("observability", Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));
	//m_oFeatureMap.add("confidence", Matrix::Zero(m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1)));

	m_vConfidenceMap.clear();
    Confidence oGridCnfd;

	for(int i = 0; i != m_oFeatureMap.getSize()(0)*m_oFeatureMap.getSize()(1); ++i)
	    m_vConfidenceMap.push_back(oGridCnfd);

	//set the point index vector with same sizes
	//std::vector<int> vOneGridIdx;
	//std::vector<std::vector<int>> vColGridVec;

	//for (int i = 0; i != m_oFeatureMap.getSize()(1); ++i)
	//	vColGridVec.push_back(vOneGridIdx);

	//for(int i = 0; i != m_oFeatureMap.getSize()(0); ++i)
	//  m_vMapPointIndex.push_back(vColGridVec);

	//resize the point index size (rawnum * m_oFeatureMap.getSize()(1) + colnum)
	//m_vGroundPntMapIdx.resize(m_oFeatureMap.getSize()(0)*m_oFeatureMap.getSize()(1));
	m_vBoundPntMapIdx.resize(m_oFeatureMap.getSize()(0)*m_oFeatureMap.getSize()(1));
	m_vObstlPntMapIdx.resize(m_oFeatureMap.getSize()(0)*m_oFeatureMap.getSize()(1));

	//get grid number in cols
	m_iGridRawNum = m_oFeatureMap.getSize()(1);

	//frame id
	m_oFeatureMap.setFrameId(m_sMapFrameID);

	//initial elevation map and center point clouds
	for (int i = 0; i != m_oFeatureMap.getSize()(0); ++i) {//i

		for (int j = 0; j != m_oFeatureMap.getSize()(1); ++j) {//j

			grid_map::Index oGridIdx;
			oGridIdx(0) = i;
			oGridIdx(1) = j;
			int iGridIdx = TwotoOneDIdx(oGridIdx);

			ROS_INFO("i=[%d],j=[%d],idx=[%d].",i,j,iGridIdx);

			//find the 1d and 2d index
			Position oGridPos;
			m_oFeatureMap.getPosition(oGridIdx, oGridPos);
			//center point
			m_vConfidenceMap[iGridIdx].oCenterPoint.x = oGridPos.x();
			m_vConfidenceMap[iGridIdx].oCenterPoint.y = oGridPos.y();
		}//end i

	}//end j


	m_bGridMapReadyFlag = true;

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
void TopologyMap::GenerateCircleMask() {

	ROS_INFO("Generate circle successfully.");
	//fRadiusGridsNum means how many grid in the radius axis 
	float fRadiusGridsNum = m_dRbtLocalRadius / m_dResolution;

	//actual raduis grids number
	float fRealRGNum = fRadiusGridsNum + 0.5;
	float fMaskStartGrid = float(floor(-fRadiusGridsNum));
	float fMaskEndGrid = float(ceil(fRadiusGridsNum));

	m_vSearchMask.clear();

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
				m_vSearchMask.push_back(oMaskMemberIdx);
			}//end if sqrt
		}//end j
	}//end i

}

//*********************************Traversing / retrieving function*********************************
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
inline bool TopologyMap::CheckInSidePoint(const pcl::PointXYZ & oPoint) {

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

int TopologyMap::PointoOneDIdx(pcl::PointXYZ & oPoint) {

	grid_map::Position oPosition(oPoint.x, oPoint.y);

	//get the 2d index of corresponding grid
	grid_map::Index oIndex;
	m_oFeatureMap.getIndex(oPosition, oIndex);

	//get the 1d index of point
	//function: ix * size(cols) + iy
	return oIndex(0) * m_iGridRawNum + oIndex(1);

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

MapIndex TopologyMap::PointoAllTypeIdx(pcl::PointXYZ & oPoint) {

	MapIndex oAllTypeIdx;

	grid_map::Position oPosition(oPoint.x, oPoint.y);

	//get the 2d index of corresponding grid
	m_oFeatureMap.getIndex(oPosition, oAllTypeIdx.oTwoIndex);

	//get the 1d index of point
	//function: ix * size(cols) + iy
	oAllTypeIdx.iOneIdx = oAllTypeIdx.oTwoIndex(0) * m_iGridRawNum + oAllTypeIdx.oTwoIndex(1);

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

int TopologyMap::TwotoOneDIdx(const grid_map::Index & oIndex) {

	//get the 1d index of point
	//function: ix * size(cols) + iy
	return oIndex(0) * m_iGridRawNum + oIndex(1);

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

void TopologyMap::OneDtoTwoDIdx(grid_map::Index & oTwoDIdx,
	const int & iOneDIdx) {

	oTwoDIdx(1) = iOneDIdx % m_iGridRawNum;
	oTwoDIdx(0) = (iOneDIdx - oTwoDIdx(1)) / m_iGridRawNum;

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

void TopologyMap::CircleNeighborhood(std::vector<MapIndex> & vNearbyGrids,
	const pcl::PointXYZ & oRobotPoint) {

	ROS_INFO("Running circle iterator demo.");
	vNearbyGrids.clear();

	Position oRobotPos(oRobotPoint.x, oRobotPoint.y);
	grid_map::Index oRobotIdx;
	m_oFeatureMap.getIndex(oRobotPos, oRobotIdx);

	pcl::PointCloud<pcl::PointXYZ> vCloud;

	//seach in given neighboring grids
	for (int i = 0; i != m_vSearchMask.size(); ++i) {

		MapIndex oOneNearGridIdx;
		//get nearby grid idx on x,y axis,respectively
		oOneNearGridIdx.oTwoIndex(0) = oRobotIdx(0) + m_vSearchMask[i].oTwoIndex(0);
		oOneNearGridIdx.oTwoIndex(1) = oRobotIdx(1) + m_vSearchMask[i].oTwoIndex(1);
		oOneNearGridIdx.iOneIdx = TwotoOneDIdx(oOneNearGridIdx.oTwoIndex);
		//compute the nearby grid
		vNearbyGrids.push_back(oOneNearGridIdx);

	}

}



//*********************************Initialization function*********************************


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
void TopologyMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory) {

	//bool bMapUpdateFlag = false;
	bool bSamplingFlag = false;

	if (!m_bGridMapReadyFlag)
		//initial the grid map based on the odometry center
		InitializeGridMap(oTrajectory.pose.pose.position.x,
			oTrajectory.pose.pose.position.y);


	if (!m_bGridMapReadyFlag)
		return;

	//receive the global map to obtain the newest environment information
	//if(!(m_iTrajFrameNum % m_iMapFreshNum)){
	//  
	//  //ROS_INFO("Grid Map Updating."); 
	//  
	//  bMapUpdateFlag = true;
	//
	//  UpdatingOctomapNodes();
	//
	//}                                                                                     

	//record the robot position and update its neighboring map
	if (!(m_iTrajFrameNum % m_iOdomSampingNum)) {

		//ROS_INFO("Nearby region searching.");  

		bSamplingFlag = true;

		//save the odom value
		pcl::PointXYZ oOdomPoint;
		oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
		oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
		oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z
		m_vOdomCloud.push(oOdomPoint);

		//make the sequence size smaller than a value
		if (m_vOdomCloud.size() > 10000)
			m_vOdomCloud.pop();

		//std::cout<<"debug 2"<<std::endl;

		if (m_bGridMapReadyFlag)
			ComputeTravelFeature(m_vOdomCloud.back());


	}

	//if the frame count touches the least common multiple of m_iOdomSampingNum and 
	//if( bMapUpdateFlag && bSamplingFlag ){
	if (bSamplingFlag) {
		//ROS_INFO("loop touches [%d] and reset at 0.", m_iTrajFrameNum);
		m_iTrajFrameNum = 0;
	}

	m_iTrajFrameNum++;

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

void TopologyMap::HandleGroundClouds(const sensor_msgs::PointCloud2 & vGroundRosData) {

	if (m_bGridMapReadyFlag) {

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZ> vOneGCloud;
		////message from ROS type to PCL type
		pcl::fromROSMsg(vGroundRosData, vOneGCloud);

		grid_map::Matrix& gridMapData = m_oFeatureMap["elevation"];

		//get right point clouds from LOAM output
		for (int i = 0; i != vOneGCloud.size(); ++i) {
			//sampling 2 time by given sampling value
			if (!(i % (m_iPCSmplNum * 2))) {
				//check the point
				if (CheckInSidePoint(vOneGCloud.points[i])) {
					//to point clouds
					MapIndex oAllTypeIdx = PointoAllTypeIdx(vOneGCloud.points[i]);

						// If no elevation has been set, use current elevation.
						if (!m_vConfidenceMap[oAllTypeIdx.iOneIdx].label) {
							//to center point cloud
							m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z = vOneGCloud.points[i].z;
							//to grid layer
							gridMapData(oAllTypeIdx.oTwoIndex(0), oAllTypeIdx.oTwoIndex(1)) = vOneGCloud.points[i].z;

							m_vConfidenceMap[oAllTypeIdx.iOneIdx].label = 2;
						
						}else{
							//moving average
							float fMeanZ = (m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z + vOneGCloud.points[i].z) / 2.0f;
							//to center point cloud
							m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z = fMeanZ;
							//to grid layer
							gridMapData(oAllTypeIdx.oTwoIndex(0), oAllTypeIdx.oTwoIndex(1)) = fMeanZ;

							if (m_vConfidenceMap[oAllTypeIdx.iOneIdx].label < 2)
								m_vConfidenceMap[oAllTypeIdx.iOneIdx].label = 2;

						}//end else


				}//end if (CheckInSidePoint(vOneGCloud.points[i]))
			}//end if (!(i%m_iPCSmplNum))
		}//end for (int i = 0; i != vOneGCloud.size();

	}//end if m_bGridMapReadyFlag

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

void TopologyMap::HandleBoundClouds(const sensor_msgs::PointCloud2 & vBoundRosData) {
	//if grid map is built
	if (m_bGridMapReadyFlag) {

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZ> vOneBCloud;
		////message from ROS type to PCL type
		pcl::fromROSMsg(vBoundRosData, vOneBCloud);

		//get boundary points between the ground region and obstacle region
		for (int i = 0; i != vOneBCloud.size(); ++i) {
			//sampling
			if (!(i%m_iPCSmplNum)) {

				if (CheckInSidePoint(vOneBCloud.points[i])) {
					m_vBoundCloud->points.push_back(vOneBCloud.points[i]);
					int iPointIdx = PointoOneDIdx(vOneBCloud.points[i]);
					//to point idx
					m_vBoundPntMapIdx[iPointIdx].push_back(m_iBoundFrames);
					//label grid
					m_vConfidenceMap[iPointIdx].label = 3;

					m_iBoundFrames++;
				}
			}//end if i%m_iPCSmplNum

		}//end for

	}//if m_bGridMapReadyFlag

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

void TopologyMap::HandleObstacleClouds(const sensor_msgs::PointCloud2 & vObstacleRosData) {

	if (m_bGridMapReadyFlag) {

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZ> vOneOCloud;
		////message from ROS type to PCL type
		pcl::fromROSMsg(vObstacleRosData, vOneOCloud);

		//get obstacle points
		for (int i = 0; i != vOneOCloud.size(); ++i) {
			//sampling
			if (!(i%m_iPCSmplNum)) {

				if (CheckInSidePoint(vOneOCloud.points[i])) {
					m_vObstacleCloud->points.push_back(vOneOCloud.points[i]);
					int iPointIdx = PointoOneDIdx(vOneOCloud.points[i]);
					//to point idx
					m_vObstlPntMapIdx[iPointIdx].push_back(m_iObstacleFrames);
					//label grid
					if(!m_vConfidenceMap[iPointIdx].label)
						m_vConfidenceMap[iPointIdx].label = 1;

					m_iObstacleFrames++;
				}
			}//end i%m_iPCSmplNum
		}//end for i

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

//void TopologyMap::UpdatingOctomapNodes() {
//
//	octomap_msgs::GetOctomap oOctomapServerVec;
//	if (!m_oOctoMapClient.call(oOctomapServerVec)) {
//		ROS_ERROR_STREAM("Failed to call service: " << m_oOctomapServiceTopic);
//		return;
//	}
//
//	// creating octree
//	octomap::OcTree* pOctomap = nullptr;
//	octomap::AbstractOcTree* pOCTree = octomap_msgs::msgToMap(oOctomapServerVec.response.map);
//	if (pOCTree) {
//		pOctomap = dynamic_cast<octomap::OcTree*>(pOCTree);
//	}
//	else {
//		ROS_ERROR("Failed to call convert Octomap.");
//		return;
//	}
//
//	//ROS_INFO("Updating point clouds from octomap topic."); 
//	//copy node points within query bounding box (oMinCorner - oMaxCorner) to grid map
//	bool bConverterRes = GridOctoConverter::UpdateFromOctomap(*pOctomap,
//		"elevation",
//		m_oFeatureMap,
//		m_vMapPointIndex,
//		m_vNodeCloud,
//		&m_oMinCorner,
//		&m_oMaxCorner);
//
//	if (!bConverterRes) {
//		ROS_ERROR("Failed to call convert Octomap.");
//		return;
//	}
//
//}

//*************Feature calculation function (Subject function)*************

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

void TopologyMap::ComputeTravelFeature(const pcl::PointXYZ & oRobotIdx) {
	//m_oFeatureMap.at("elevation", *oIterator) = 2.0;

	std::vector<MapIndex> vNearByIdx;
	CircleNeighborhood(vNearByIdx, oRobotIdx);
	pcl::PointCloud<pcl::PointXYZ> vCloud;

	for (int i = 0; i != vNearByIdx.size(); ++i) {

		if(m_vConfidenceMap[vNearByIdx[i].iOneIdx].label == 2)
			vCloud.push_back(m_vConfidenceMap[vNearByIdx[i].iOneIdx].oCenterPoint);

	}

	PublishPointCloud(vCloud);
	//PublishGridMap();

}

//*********************************Traversing / retrieving function*********************************
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

void TopologyMap::PublishGridMap(){
  // Publish as grid map.

  ros::Time oNowTime = ros::Time::now();
  m_oFeatureMap.setTimestamp(oNowTime.toNSec());

  grid_map_msgs::GridMap oGridMapMessage;

  grid_map::GridMapRosConverter::toMessage(m_oFeatureMap, oGridMapMessage);

  m_oGridMapPublisher.publish(oGridMapMessage);

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

//void TopologyMap::PublishOctoMap(octomap::OcTree* pOctomap){
//  // Also publish as an octomap msg for visualization
//  octomap_msgs::Octomap oOctomapMessage;
//
//  octomap_msgs::fullMapToMsg(*pOctomap, oOctomapMessage);
//
//  oOctomapMessage.header.frame_id = m_oFeatureMap.getFrameId();
//
//  m_oOctomapPublisher.publish(oOctomapMessage);
//
//}


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

void TopologyMap::PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud){
  //publish obstacle points
  sensor_msgs::PointCloud2 vCloudData;

  pcl::toROSMsg(vCloud, vCloudData);

  vCloudData.header.frame_id = m_oFeatureMap.getFrameId();

  vCloudData.header.stamp = ros::Time::now();

  m_oCloudPublisher.publish(vCloudData);

}




} /* namespace */



