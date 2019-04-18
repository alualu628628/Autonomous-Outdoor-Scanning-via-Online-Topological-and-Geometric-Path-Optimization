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
	                     ros::NodeHandle & nodeHandle):
                         m_oCnfdnSolver(12.0,4.2,5),
	                     m_pBoundCloud(new pcl::PointCloud<pcl::PointXYZ>),
	                     m_pObstacleCloud(new pcl::PointCloud<pcl::PointXYZ>),
	                     m_iTrajFrameNum(0),
	                     m_iGroundFrames(0),
	                     m_iBoundFrames(0),
	                     m_iObstacleFrames(0),
	                     m_iComputedFrame(0),
	                     m_iOdomSampingNum(25),
	                     m_bGridMapReadyFlag(false) {

	//read parameters
	ReadTopicParams(nodeHandle);

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

bool TopologyMap::ReadTopicParams(ros::NodeHandle & nodeHandle) {
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

	nodeHandle.param("pastview_interval", m_dPastViewIntvl, 5.0);
	m_iIntervalNum = int(m_dPastViewIntvl * m_dSamplingHz);
	if(m_iIntervalNum <= 0)
	   m_iIntervalNum = 1;//at least one point (in this case, this point will be front and back point of quene)


    double dViewZOffset;
	nodeHandle.param("pastview_zoffset", dViewZOffset, 0.0);
	m_fViewZOffset = float(dViewZOffset);

	//point cloud sampling number
	nodeHandle.param("pointframe_smplnum", m_iPCSmplNum, 1);

	//***************grid map parameters***************
	//map range/map size
	double dMapMaxRange;
	nodeHandle.param("gridmap_maxrange", dMapMaxRange, 250.0);
	if (dMapMaxRange <= 0)//defend a zero input
		dMapMaxRange = 50.0;
	
	//map cell size
	double dResolution;
	nodeHandle.param("gridmap_resolution", dResolution, 0.1);
	if (dResolution <= 0)
		dResolution = 0.1;

	//map limited height
	double dMinMapZ;
	nodeHandle.param("min_mapz", dMinMapZ, -2.0);
	double dMaxMapZ;
	nodeHandle.param("max_mapz", dMaxMapZ, 7.0);

	//map id
	std::string sMapFrameID;
	nodeHandle.param("gridmap_frameid", sMapFrameID, std::string("map"));

    //generate grid map
	m_oGMer.GetParam(dMapMaxRange,
		              dResolution,
		                 dMinMapZ,
		                 dMaxMapZ,
		              sMapFrameID);

	//robot's neighborhood searching radius
	double dRbtLocalRadius;
	nodeHandle.param("robot_local_r", dRbtLocalRadius, 5.0);
	m_oCnfdnSolver.SetSigmaValue(dRbtLocalRadius);

	//generate robot neighborhood searching mask 
	m_oGMer.m_vRobotSearchMask.clear();
	m_oGMer.m_vRobotSearchMask = m_oGMer.GenerateCircleMask(dRbtLocalRadius);

	//node neighborhood searching radius
	double dNodeRadiusRate;
	nodeHandle.param("nodegenerate_rate", dNodeRadiusRate, 0.5);

	//generate node generation neighborhood mask
	double dNodeRadius =  dRbtLocalRadius * dNodeRadiusRate;
	m_oGMer.m_vNodeMadeMask.clear();
	m_oGMer.m_vNodeMadeMask = m_oGMer.GenerateCircleMask(dNodeRadius);

	//generate region grow neighborhood mask
	//eight unicom
	m_oGMer.m_vGrowSearchMask.clear();
	for(int i = -1; i != 2; ++i ){

		for(int j = -1 ; j != 2 ; ++j){
			MapIndex oMaskMemberIdx;
			oMaskMemberIdx.oTwoIndex(0) = i;
			oMaskMemberIdx.oTwoIndex(1) = j;
			m_oGMer.m_vGrowSearchMask.push_back(oMaskMemberIdx); 
		}//end j
	}//end i

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

void TopologyMap::InitializeGridMap(const pcl::PointXYZ & oRobotPos) {

	//generate grid map
    m_oGMer.GenerateMap(oRobotPos);

	//get grid number in cols
	ExtendedGM::iGridRawNum = m_oGMer.m_oFeatureMap.getSize()(1);
	ROS_INFO("Set grid raw number as [%d]", ExtendedGM::iGridRawNum);

	//generate confidence map
	m_vConfidenceMap.clear();

    ConfidenceValue oGridCnfd;
	for(int i = 0; i != m_oGMer.m_oFeatureMap.getSize()(0)*m_oGMer.m_oFeatureMap.getSize()(1); ++i)
	    m_vConfidenceMap.push_back(oGridCnfd);


	m_vBoundPntMapIdx.resize(m_oGMer.m_oFeatureMap.getSize()(0)*m_oGMer.m_oFeatureMap.getSize()(1));
	m_vObstlPntMapIdx.resize(m_oGMer.m_oFeatureMap.getSize()(0)*m_oGMer.m_oFeatureMap.getSize()(1));

	//initial elevation map and center point clouds
	for (int i = 0; i != m_oGMer.m_oFeatureMap.getSize()(0); ++i) {//i

		for (int j = 0; j != m_oGMer.m_oFeatureMap.getSize()(1); ++j) {//j

			grid_map::Index oGridIdx;
			oGridIdx(0) = i;
			oGridIdx(1) = j;
			int iGridIdx = ExtendedGM::TwotoOneDIdx(oGridIdx);

			//find the 1d and 2d index
			grid_map::Position oGridPos;
			m_oGMer.m_oFeatureMap.getPosition(oGridIdx, oGridPos);
			//center point
			m_vConfidenceMap[iGridIdx].oCenterPoint.x = oGridPos.x();
			m_vConfidenceMap[iGridIdx].oCenterPoint.y = oGridPos.y();
		}//end i

	}//end j


	m_bGridMapReadyFlag = true;

}




//*********************************Traversing / retrieving function*********************************



/*************************************************
Function: DevidePointClouds
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

void TopologyMap::DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                                  pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
                                    pcl::PointCloud<pcl::PointXYZ> & vNearAllClouds,
	                                         std::vector<int> & vNearGroundGridIdxs,
                                          const std::vector<MapIndex> & vNearByIdxs){

	//prepare and clear
	vNearGrndClouds.clear();
    vNearBndryClouds.clear();
    vNearAllClouds.clear();
	vNearGroundGridIdxs.clear();

	pcl::PointCloud<pcl::PointXYZ> vNearObstClouds;//nearby obstacle point clouds

	//to each nearby grids
	for (int i = 0; i != vNearByIdxs.size(); ++i) {

		int iNearGridId = vNearByIdxs[i].iOneIdx;
		//assign to cooresponding point clouds based on its label
        switch (m_vConfidenceMap[iNearGridId].label){

        	case 1 : //the grid is a obstacle grid
        	    for (int j = 0; j != m_vObstlPntMapIdx[iNearGridId].size(); ++j)
        	    	vNearObstClouds.points.push_back(m_pObstacleCloud->points[m_vObstlPntMapIdx[iNearGridId][j]]);
            break;

            case 2 : //the grid is a ground grid
                vNearGrndClouds.points.push_back(m_vConfidenceMap[iNearGridId].oCenterPoint);
        	    vNearGroundGridIdxs.push_back(iNearGridId);
        	    //if the obstacle is large (perhaps some obstacles above the ground,e.g.,leafs points, high vegetation)
        	    if(m_vObstlPntMapIdx[iNearGridId].size() > 20){
        	    	for (int j = 0; j != m_vObstlPntMapIdx[iNearGridId].size(); ++j)
        	    		vNearObstClouds.points.push_back(m_pObstacleCloud->points[m_vObstlPntMapIdx[iNearGridId][j]]);
        	    }
            break;

            case 3 : //the grid is a boundary grid
                for (int j = 0; j != m_vBoundPntMapIdx[iNearGridId].size(); ++j)
				    vNearBndryClouds.points.push_back(m_pBoundCloud->points[m_vBoundPntMapIdx[iNearGridId][j]]);
				for (int j = 0; j != m_vObstlPntMapIdx[iNearGridId].size(); ++j)
        	    	vNearObstClouds.points.push_back(m_pObstacleCloud->points[m_vObstlPntMapIdx[iNearGridId][j]]);
            break;

            default:
            break;
        }

    }

    //make a all label point clouds (for occlusion detection)
    vNearAllClouds.reserve(vNearGrndClouds.size() + vNearBndryClouds.size() + vNearObstClouds.size());

    for(int i = 0; i != vNearGrndClouds.size(); ++i)
    	vNearAllClouds.push_back(vNearGrndClouds.points[i]);

    for(int i = 0; i != vNearBndryClouds.size(); ++i)
    	vNearAllClouds.push_back(vNearBndryClouds.points[i]);

    for(int i = 0; i != vNearObstClouds.size(); ++i)
    	vNearAllClouds.push_back(vNearObstClouds.points[i]);

}
//reload without generating vNearAllClouds
void TopologyMap::DevidePointClouds(pcl::PointCloud<pcl::PointXYZ> & vNearGrndClouds,
                                  pcl::PointCloud<pcl::PointXYZ> & vNearBndryClouds,
	                                         std::vector<int> & vNearGroundGridIdxs,
                                          const std::vector<MapIndex> & vNearByIdxs){

	//prepare and clear
	vNearGrndClouds.clear();
    vNearBndryClouds.clear();
	vNearGroundGridIdxs.clear();

	pcl::PointCloud<pcl::PointXYZ> vNearObstClouds;//nearby obstacle point clouds

	//to each nearby grids
	for (int i = 0; i != vNearByIdxs.size(); ++i) {

		int iNearGridId = vNearByIdxs[i].iOneIdx;
		//assign to cooresponding point clouds based on its label
        switch (m_vConfidenceMap[iNearGridId].label){

        	case 1 : //the grid is a obstacle grid
        	    for (int j = 0; j != m_vObstlPntMapIdx[iNearGridId].size(); ++j)
        	    	vNearObstClouds.points.push_back(m_pObstacleCloud->points[m_vObstlPntMapIdx[iNearGridId][j]]);
            break;

            case 2 : //the grid is a ground grid
                vNearGrndClouds.points.push_back(m_vConfidenceMap[iNearGridId].oCenterPoint);
        	    vNearGroundGridIdxs.push_back(iNearGridId);
            break;

            default:
            break;
        }//end switch

    }//end for int

}

//*********************************Processing function*********************************
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
void TopologyMap::SamplingPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                                  std::vector<std::vector<int> > & vPointMapIdx,
	                                  int iSmplNum){

    //point clouds sampling based on the idx vector
	for(int i = 0; i != vPointMapIdx.size(); ++i){
		//retain at least one point in grid
		if(vPointMapIdx[i].size()){

			std::vector<int> vOneSmplIdx;
		        //sampling
			for(int j = 0; j != vPointMapIdx[i].size(); j = j + iSmplNum)
				vOneSmplIdx.push_back(vPointMapIdx[i][j]);
			

            vPointMapIdx[i].clear();
            for(int j = 0; j != vOneSmplIdx.size(); ++j)
                vPointMapIdx[i].push_back(vOneSmplIdx[j]);

        }//end if

    }//end for


    pcl::PointCloud<pcl::PointXYZ> vSmplClouds;

    //save each corresponding points
    for(int i = 0; i != vPointMapIdx.size(); ++i){
    	for(int j = 0; j != vPointMapIdx[i].size(); ++j){
			vSmplClouds.push_back(pCloud->points[vPointMapIdx[i][j]]);	
        }
	}

    //clear the point vector and save the new sampled points
	pCloud->points.clear();
	for(int i = 0; i != vSmplClouds.size(); ++i)
		pCloud->points.push_back(vSmplClouds.points[i]);


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
void TopologyMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory) {

	//bool bMapUpdateFlag = false;
	bool bSamplingFlag = false;

	if (!m_bGridMapReadyFlag){
		//initial the grid map based on the odometry center

		pcl::PointXYZ oOdomPoint;
		oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
		oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
		oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z
		InitializeGridMap(oOdomPoint);
	}
	
	if (!m_bGridMapReadyFlag)
		return;                                                                                

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
		//m_vOdomCloud in fact is a history odometry within a given interval
		//it only report the current robot position (back) and past (given time before) position (front) 
		if (m_vOdomCloud.size() > m_iIntervalNum)
			m_vOdomCloud.pop();

		//compute the confidence map on the constructed map with surronding point clouds
		if (m_bGridMapReadyFlag){
			//frequency of visibility calculation should be low
			if(m_iComputedFrame%3){

				//compute the distance and boundary feature
				ComputeConfidence(m_vOdomCloud.back());
		    }else{

		    	//compute all features
		    	ComputeConfidence(m_vOdomCloud.back(), m_vOdomCloud.front());
			}//end else
		}//end if m_bGridMapReadyFlag


	}

	//if the frame count touches the least common multiple of m_iOdomSampingNum and 
	//if( bMapUpdateFlag && bSamplingFlag ){
	if (bSamplingFlag) {
		//ROS_INFO("loop touches [%d] and reset at 0.", m_iTrajFrameNum);
		m_iTrajFrameNum = 0;
		m_iComputedFrame++;
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

		//get right point clouds from LOAM output
		for (int i = 0; i != vOneGCloud.size(); ++i) {
			//sampling 2 time by given sampling value
			if (!(i % (m_iPCSmplNum * 2))) {
				//check the point
				if (m_oGMer.CheckInSidePoint(vOneGCloud.points[i])) {
					//to point clouds
					MapIndex oAllTypeIdx = ExtendedGM::PointoAllTypeIdx(vOneGCloud.points[i],m_oGMer.m_oFeatureMap);

						// If no elevation has been set, use current elevation.
						if (!m_vConfidenceMap[oAllTypeIdx.iOneIdx].label) {
							//to center point cloud
							m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z = vOneGCloud.points[i].z;
							//to grid layer

							m_vConfidenceMap[oAllTypeIdx.iOneIdx].label = 2;
						
						}else{
							//moving average
							float fMeanZ = (m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z + vOneGCloud.points[i].z) / 2.0f;
							//to center point cloud
							m_vConfidenceMap[oAllTypeIdx.iOneIdx].oCenterPoint.z = fMeanZ;
							//to grid layer
							
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

				if (m_oGMer.CheckInSidePoint(vOneBCloud.points[i])) {
					m_pBoundCloud->points.push_back(vOneBCloud.points[i]);
					int iPointIdx = ExtendedGM::PointoOneDIdx(vOneBCloud.points[i],m_oGMer.m_oFeatureMap);
					//to point idx
					m_vBoundPntMapIdx[iPointIdx].push_back(m_iBoundFrames);
					//label grid
					m_vConfidenceMap[iPointIdx].label = 3;

					m_iBoundFrames++;
				}
			}//end if i%m_iPCSmplNum

		}//end for


		if(m_pObstacleCloud->points.size()>3000000){
			SamplingPointClouds(m_pBoundCloud, m_vBoundPntMapIdx);
		}//end if if(m_pObstacleCloud->points.size()>X)

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

				if (m_oGMer.CheckInSidePoint(vOneOCloud.points[i])) {
					m_pObstacleCloud->points.push_back(vOneOCloud.points[i]);
					int iPointIdx = ExtendedGM::PointoOneDIdx(vOneOCloud.points[i],m_oGMer.m_oFeatureMap);
					//to point idx
					m_vObstlPntMapIdx[iPointIdx].push_back(m_iObstacleFrames);
					//label grid
					if(!m_vConfidenceMap[iPointIdx].label)
						m_vConfidenceMap[iPointIdx].label = 1;

					m_iObstacleFrames++;
				}
			}//end i%m_iPCSmplNum
		}//end for i


		if(m_pObstacleCloud->points.size()>8000000){
			SamplingPointClouds(m_pObstacleCloud, m_vObstlPntMapIdx);
		}//end if if(m_pObstacleCloud->points.size()>X)


	}//end if (m_bGridMapReadyFlag) 

}



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

void TopologyMap::ComputeConfidence(const pcl::PointXYZ & oCurrRobotPos,
	                                const pcl::PointXYZ & oPastRobotPos) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr pNearGrndClouds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pNearBndryClouds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pNearAllClouds(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> vNearGrndGrdIdxs;

    //find the neighboring point clouds
    std::vector<MapIndex> vNearByIdxs;
	ExtendedGM::CircleNeighborhood(vNearByIdxs, 
		                           m_oGMer.m_oFeatureMap, 
		                           m_oGMer.m_vRobotSearchMask,
		                           oCurrRobotPos);

    //extract point clouds with different labels, respectively
    DevidePointClouds(*pNearGrndClouds,
    	              *pNearBndryClouds,
    	              *pNearAllClouds,
	                  vNearGrndGrdIdxs,
                      vNearByIdxs);


    //compute distance term
    m_oCnfdnSolver.DistanceTerm(m_vConfidenceMap,
    	                           oCurrRobotPos,
                                vNearGrndGrdIdxs,
	                            *pNearGrndClouds);


    //compute boundary term
    m_oCnfdnSolver.BoundTerm(m_vConfidenceMap,
                             vNearGrndGrdIdxs,
	                         pNearGrndClouds,
    	                     pNearBndryClouds);

    //in this case, robot position is based on odom frame, it need to be transfored to lidar sensor frame 
    pcl::PointXYZ oPastView;
    oPastView.x = oPastRobotPos.x;
    oPastView.y = oPastRobotPos.y;
    oPastView.z = oPastRobotPos.z + m_fViewZOffset;

    //compute visibiity
    m_oCnfdnSolver.OcclusionTerm(m_vConfidenceMap,
	                             pNearAllClouds,
	                             vNearGrndGrdIdxs,
	                             oPastView);

    //publish result
	//PublishPointCloud(*pNearGrndClouds);//for test
	//PublishPointCloud(*pNearBndryClouds);//for test
	PublishPointCloud(*pNearAllClouds);//for test
	PublishGridMap();

}
//reload without occlusion
void TopologyMap::ComputeConfidence(const pcl::PointXYZ & oCurrRobotPos) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr pNearGrndClouds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pNearBndryClouds(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> vNearGrndGrdIdxs;

    //find the neighboring point clouds
    std::vector<MapIndex> vNearByIdxs;
	ExtendedGM::CircleNeighborhood(vNearByIdxs,
	                               m_oGMer.m_oFeatureMap,
	                               m_oGMer.m_vRobotSearchMask,
	                               oCurrRobotPos);

    //extract point clouds with different labels, respectively
    DevidePointClouds(*pNearGrndClouds,
    	              *pNearBndryClouds,
	                  vNearGrndGrdIdxs,
                      vNearByIdxs);


    //compute distance term
    m_oCnfdnSolver.DistanceTerm(m_vConfidenceMap,
    	                           oCurrRobotPos,
                                vNearGrndGrdIdxs,
	                            *pNearGrndClouds);


    //compute boundary term
    if(vNearGrndGrdIdxs.size() >= 3)
    	m_oCnfdnSolver.BoundTerm(m_vConfidenceMap,
                                 vNearGrndGrdIdxs,
	                              pNearGrndClouds,
    	                         pNearBndryClouds);


    //publish result
	//PublishPointCloud(*pNearGrndClouds);//for test
	//PublishPointCloud(*pNearBndryClouds);//for test
	
	PublishGridMap();

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

    //push 
	grid_map::Matrix& gridMapData = m_oGMer.m_oFeatureMap["elevation"];

	//initial elevation map and center point clouds
	for (int i = 0; i != m_oGMer.m_oFeatureMap.getSize()(0); ++i) {//i

		for (int j = 0; j != m_oGMer.m_oFeatureMap.getSize()(1); ++j) {//j

			int iGridIdx = ExtendedGM::TwotoOneDIdx(i, j);

		//.travelTerm   //.boundTerm    //.visiTerm    //.qualTerm      //.totalValue
			gridMapData(i, j) = m_vConfidenceMap[iGridIdx].visiTerm;
			
		}//end i

	}//end j

	ros::Time oNowTime = ros::Time::now();

	m_oGMer.m_oFeatureMap.setTimestamp(oNowTime.toNSec());

	grid_map_msgs::GridMap oGridMapMessage;
	grid_map::GridMapRosConverter::toMessage(m_oGMer.m_oFeatureMap, oGridMapMessage);
	// Publish as grid map.
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

void TopologyMap::PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud){
  //publish obstacle points
  sensor_msgs::PointCloud2 vCloudData;

  pcl::toROSMsg(vCloud, vCloudData);

  vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();

  vCloudData.header.stamp = ros::Time::now();

  m_oCloudPublisher.publish(vCloudData);

}




} /* namespace */





