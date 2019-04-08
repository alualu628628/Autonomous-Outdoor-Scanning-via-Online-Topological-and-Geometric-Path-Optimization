#include "TopologyMap.h"


namespace topology_map{



TopologyMap::TopologyMap(ros::NodeHandle & node,
                         ros::NodeHandle & nodeHandle):m_oFeatureMap(grid_map::GridMap({"elevation"})),
                                                                                     m_iTrajFrameNum(0),
                                                                                     m_iOdomSampingNum(25),
                                                                                     m_iMapFreshNum(50),
                                                                                     m_bGridMapReadyFlag(false){

  //read parameters
  ReadParameters(nodeHandle);

  //subscribe topic 
  m_oOctoMapClient = nodeHandle.serviceClient<octomap_msgs::GetOctomap>(m_oOctomapServiceTopic);

  //subscribe (hear) the odometry information
  m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &TopologyMap::HandleTrajectory, this);

  //publish topic
  m_oGridMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  m_oOctomapPublisher = nodeHandle.advertise<octomap_msgs::Octomap>("octomap", 1, true);

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

TopologyMap::~TopologyMap()
{
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
                                    const float & fRobotY){

  //set feature layer
  m_oFeatureMap.add("traversability");
  m_oFeatureMap.add("boundary");
  m_oFeatureMap.add("observability");
  m_oFeatureMap.add("confidence");
  m_oFeatureMap.setBasicLayers({"elevation"});

  //corner of map's bounding box
  m_oMinCorner(0) = fRobotX - float(m_dMapMaxRange);
 
  m_oMaxCorner(0) = fRobotX + float(m_dMapMaxRange);
  
  m_oMinCorner(1) = fRobotY - float(m_dMapMaxRange);
  
  m_oMaxCorner(1) = fRobotY + float(m_dMapMaxRange);

  //build the map
  m_oFeatureMap.setGeometry(Length(2.0*m_dMapMaxRange , 2.0*m_dMapMaxRange), 
                                                              m_dResolution, 
                                                Position(fRobotX, fRobotY));

   //set the point index vector with same sizes
  std::vector<int> vOneGridIdx;
  std::vector<std::vector<int>> vColGridVec;

  for(int i = 0; i != m_oFeatureMap.getSize()(1); ++i)
    vColGridVec.push_back(vOneGridIdx);

  for(int i = 0; i != m_oFeatureMap.getSize()(0); ++i)
    m_vMapPointIndex.push_back(vColGridVec);

  //dispaly
  ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", m_oFeatureMap.getLength().x(), 
           m_oFeatureMap.getLength().y(), m_oFeatureMap.getSize()(0), m_oFeatureMap.getSize()(1));

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
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/

bool TopologyMap::ReadParameters(ros::NodeHandle & nodeHandle)
{
  //input topic
  nodeHandle.param("octomap_service_topic", m_oOctomapServiceTopic, std::string("/octomap_binary"));
  nodeHandle.param("odom_in_topic", m_sOdomTopic, std::string("/odometry/filtered"));

  //frequncy
  nodeHandle.param("odometry_rawfreq", m_dOdomRawHz, 50.0);
  nodeHandle.param("odomsampling_freq", m_dSamplingHz, 2.0);

  m_iOdomSampingNum = int(m_dOdomRawHz / m_dSamplingHz); 
  ROS_INFO("Set odometry down sampling times as [%d]",m_iOdomSampingNum);

  //
  nodeHandle.param("freshoctomap_freq", m_dOctomapFreshHz, 0.5);
  
  m_iMapFreshNum = int(m_dOdomRawHz / m_dOctomapFreshHz); 
  ROS_INFO("Set updating octomap times as [%d]",m_iMapFreshNum);
  
  //map parameters
  //map range/map size
  nodeHandle.param("gridmap_maxrange", m_dMapMaxRange, 250.0);
  if(m_dMapMaxRange <= 0)//defend a zero input
     m_dMapMaxRange = 50.0;

  //map limited height
  double dMinMapZ;
  nodeHandle.param("min_mapz", dMinMapZ, -2.0);
  m_oMinCorner(2) = dMinMapZ;

  double dMaxMapZ;
  nodeHandle.param("max_mapz", dMaxMapZ, 7.0);
  m_oMaxCorner(2) = dMaxMapZ;

  //map cell size
  nodeHandle.param("gridmap_resolution", m_dResolution, 0.1);

  //map neighboring region size
  nodeHandle.param("robot_local_r", m_dRbtLocalRadius, 5.0);

  return true;
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
void TopologyMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
  
  bool bMapUpdateFlag = false;
  bool bSamplingFlag = false;

  if(!m_bGridMapReadyFlag)
    //initial the grid map based on the odometry center
    InitializeGridMap(oTrajectory.pose.pose.position.x,
                      oTrajectory.pose.pose.position.y);


  if(!m_bGridMapReadyFlag)
    return;

  //receive the global map to obtain the newest environment information
  if(!(m_iTrajFrameNum % m_iMapFreshNum)){
    ROS_INFO("Debug 2."); 
    //std::cout<<"debug 1"<<std::endl;
    bMapUpdateFlag = true;

      ConvertAndPublishMap();

  }                                                                                     

  //record the robot position and update its neighboring map
  if(!(m_iTrajFrameNum % m_iOdomSampingNum)){
    ROS_INFO("Debug 3."); 
    bSamplingFlag = true;

    //save the odom value
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z
    m_vOdomCloud.push(oOdomPoint);

    //make the sequence size smaller than a value
    if(m_vOdomCloud.size() > 10000)
       m_vOdomCloud.pop();

    //std::cout<<"debug 2"<<std::endl;

    if(m_bGridMapReadyFlag)
    CircleNeighboringGrids(m_vOdomCloud.back());


  }

  //if the frame count touches the least common multiple of m_iOdomSampingNum and 
  if( bMapUpdateFlag && bSamplingFlag ){
    
    //ROS_INFO("loop touches [%d] and reset at 0.", m_iTrajFrameNum);
    m_iTrajFrameNum = 0;
  }

  m_iTrajFrameNum++;

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

void TopologyMap::CircleNeighboringGrids(pcl::PointXYZ & oRobotPoint)
{
  ROS_INFO("Running circle iterator demo."); 

  Position oRobotPos(oRobotPoint.x, oRobotPoint.y);
  
   
  pcl::PointCloud<pcl::PointXYZ> vCloud;

  for (grid_map::CircleIterator oIterator(m_oFeatureMap, oRobotPos, m_dRbtLocalRadius);
      !oIterator.isPastEnd(); ++oIterator) {

      //m_oFeatureMap.at("elevation", *oIterator) = 2.0;

      for(int i = 0; i != m_vMapPointIndex[(*oIterator)(0)][(*oIterator)(1)].size(); ++i){

          int iNPCIdx = m_vMapPointIndex[(*oIterator)(0)][(*oIterator)(1)][i];
          vCloud.push_back(m_vNodeCloud.points[iNPCIdx]);

      }

  }

  int test = vCloud.size();
  
  ROS_INFO("result point size is [%d].", test);

  //publish obstacle points
  sensor_msgs::PointCloud2 vCloudData;
  pcl::toROSMsg(vCloud, vCloudData);
  vCloudData.header.frame_id = "odom";
  vCloudData.header.stamp =  ros::Time::now();
  m_oCloudPublisher.publish(vCloudData);
  ROS_INFO("Debug 4."); 

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

void TopologyMap::ConvertAndPublishMap(){

  octomap_msgs::GetOctomap oOctomapServerVec;
  if (!m_oOctoMapClient.call(oOctomapServerVec)) {
    ROS_ERROR_STREAM("Failed to call service: " << m_oOctomapServiceTopic);
    return;
  }
  
  // creating octree
  octomap::OcTree* pOctomap = nullptr;
  octomap::AbstractOcTree* pOCTree = octomap_msgs::msgToMap(oOctomapServerVec.response.map);
  if (pOCTree) {
    pOctomap = dynamic_cast<octomap::OcTree*>(pOCTree);
  } else {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }

  ROS_INFO("Debug 5."); 
  //copy node points within query bounding box (oMinCorner - oMaxCorner) to grid map
  bool bConverterRes = GridOctoConverter::UpdateFromOctomap(*pOctomap, 
                                                          "elevation", 
                                                        m_oFeatureMap, 
                                                     m_vMapPointIndex,
                                                         m_vNodeCloud,
                                                        &m_oMinCorner,
                                                        &m_oMaxCorner);

  if (!bConverterRes) {
    ROS_ERROR("Failed to call convert Octomap.");
    return;
  }
  
  m_oFeatureMap.setFrameId(oOctomapServerVec.response.map.header.frame_id);

  // Publish as grid map.
  grid_map_msgs::GridMap oGridMapMessage;
  grid_map::GridMapRosConverter::toMessage(m_oFeatureMap, oGridMapMessage);
  m_oGridMapPublisher.publish(oGridMapMessage);

  // Also publish as an octomap msg for visualization
  octomap_msgs::Octomap oOctomapMessage;
  octomap_msgs::fullMapToMsg(*pOctomap, oOctomapMessage);
  oOctomapMessage.header.frame_id = m_oFeatureMap.getFrameId();
  m_oOctomapPublisher.publish(oOctomapMessage);

  
}

} /* namespace */

