#include "SLAMTrans.h"


/*************************************************
Function: SLAMTrans
Description: constrcution function for SLAMTrans class
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
SLAMTrans::SLAMTrans(ros::NodeHandle & node, 
                        ros::NodeHandle & private_node)
                        :m_bFileNmFlag(false),
                         m_iFrames(-1),
                         m_iTrajPointNum(-1){

    //get the value of parameters
    //system parameters
    GetOutputPath(private_node);
    
    //point cloud related
    GetSamplingNum(private_node);

    //get point cloud input
    GetLaserTopic(private_node);
    
    //trajectory related 
    GetOdomTopic(private_node);

    //set the topics to be published
    SetOutTopics(private_node);

    //set the frame id of published data
    SetFrameIDs(private_node);

    //get offset of height value
    GetHeightOffset(private_node);

    //subscribe (hear) the point cloud topic from laser on right side 
    m_oLaserSuber = node.subscribe(m_sLaserTopic, 2, &SLAMTrans::HandlePointClouds, this);
    //subscribe (hear) the odometry information
    m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &SLAMTrans::HandleTrajectory, this);

    //publish
    m_oLaserPub = node.advertise<sensor_msgs::PointCloud2>(m_sLaserOutTopic, 2);
    //publish topic of boundary clouds 
    m_oOdomPub = node.advertise<nav_msgs::Odometry>(m_sOdomOutTopic, 2);

}

/*************************************************
Function: GetOutputPath
Description: inital function for m_sFileHead
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sFileHead
Return: none
Others: none
*************************************************/


bool SLAMTrans::GetOutputPath(ros::NodeHandle & private_node){
        
    std::string sFileHead;

    if(private_node.getParam("Output_path", sFileHead)){

      m_sFileHead = sFileHead;

      return true;
                     
      }else{

      m_sFileHead = "./";///<in the default user's path (in usual)

      return false;

    }//end if

}


/*************************************************
Function: GetLaserTopic
Description: inital function for m_sLaserTopic
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sLaserTopic
Return: none
Others: none
*************************************************/


bool SLAMTrans::GetLaserTopic(ros::NodeHandle & private_node){
        
    std::string sLaserTopic; 

    if(private_node.getParam("lidar_topic", sLaserTopic)){

        m_sLaserTopic = sLaserTopic;

        return true;
                     
       }else{

        m_sLaserTopic = "/velodyne_points";///<velodyne LiDAR due to its popular

        return false;

    }//end if

}

/*************************************************
Function: GetSamplingNum
Description: inital function for m_iSampleNum
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_iSampleNum
Return: none
Others: none
*************************************************/


bool SLAMTrans::GetSamplingNum(ros::NodeHandle & private_node){
        
    int iSampleNum; 

    if(private_node.getParam("sampling_number", iSampleNum)){

        m_iSampleNum =  iSampleNum;

        return true;
                     
       }else{

        m_iSampleNum =  2;///<down sampling under 2 frames

        return false;

    }//end if

}

/*************************************************
Function: GetOdomTopic
Description: inital function for m_sOdomTopic
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sOdomTopic
Return: none
Others: none
*************************************************/
    //trajectory related 
bool SLAMTrans::GetOdomTopic(ros::NodeHandle & private_node){

    std::string sOdomTopic; 

    if(private_node.getParam("traj_topic", sOdomTopic)){

        m_sOdomTopic = sOdomTopic;

        return true;
                     
       }else{

        m_sOdomTopic = "/odom";///<

        return false;

    }//end if

}


/*************************************************
Function: SetOutTopics
Description:  set the topics to be published
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sLaserOutTopic
        m_sOdomOutTopic
Return: none
Others: none
*************************************************/
bool SLAMTrans::SetOutTopics(ros::NodeHandle & private_node){

  bool bKnownFlag = true;

  std::string sLaserOutTopic; 

  if(private_node.getParam("lidarout_topic", sLaserOutTopic)){

      m_sLaserOutTopic = sLaserOutTopic;

                     
    }else{

      m_sLaserOutTopic = "/slam_points";///<velodyne LiDAR due to its popular
      bKnownFlag = false;

  }//end if


  std::string sOdomOutTopic; 

  if(private_node.getParam("trajout_topic", sOdomOutTopic)){

      m_sOdomOutTopic = sOdomOutTopic;

      return true;
                     
    }else{

      m_sOdomOutTopic = "/slam_odom";///<
      bKnownFlag = false;

  }//end if

  return bKnownFlag;

}


/*************************************************
Function: SetFrameIDs
Description:  set the frame name of SLAM (the frames in which make the slam)
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sSLAMParentN
        m_sSLAMChildN
Return: none
Others: none
*************************************************/
bool SLAMTrans::SetFrameIDs(ros::NodeHandle & private_node){

  bool bKnownFlag = true;

  std::string sSLAMParentN; 

  if(private_node.getParam("slam_parentframe", sSLAMParentN)){

      m_sSLAMParentN = sSLAMParentN;

                     
    }else{

      m_sSLAMParentN = "map";///<the reference frame
      bKnownFlag = false;

  }//end if


  std::string sSLAMChildN; 

  if(private_node.getParam("slam_childframe", sSLAMChildN)){

      m_sSLAMChildN = sSLAMChildN;

      return true;
                     
    }else{

      m_sSLAMChildN = "camera_init";///<the target frame indicats where the slam locates
      bKnownFlag = false;

  }//end if

  return bKnownFlag;

}


/*************************************************
Function: GetHeightOffset
Description:  get the height value of offset between computed device and the given child frame
Calls: Launch file maybe
Called By: SLAMTrans
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_fZOffset
Return: none
Others: none
*************************************************/
bool SLAMTrans::GetHeightOffset(ros::NodeHandle & private_node){

    double dZOffset;

    if(private_node.getParam("slam_zoffset", dZOffset))
        
       m_fZOffset = float(dZOffset);

    else

       m_fZOffset = 0.0;

}


/*************************************************
Function: HandleRightLaser
Description: a callback function in below: 
             node.subscribe(m_sLaserTopic, 5, &SLAMTrans::HandlePointClouds, this);
Calls: CheckTruthPoint
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a transposed point clouds which are almost the same with raw point 
Return: none
Others: none
*************************************************/
void SLAMTrans::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{
  
 // if(!m_bFileNmFlag){

    //set the current time stamp as a file name
    //full name 
  //  m_sOutPCFileName << m_sFileHead << "_PC_" << vLaserData.header.stamp << ".txt"; 

  //  m_bFileNmFlag = true;
  //}
  
  //count input frames
  m_iFrames++;
  

  if(!(m_iFrames%m_iSampleNum)){

      std::ofstream oRecordedFile;
      oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

      //fresh the LiDAR time to be consistent with the recording time IF necessary
      //ros::Time oCurrenTime = ros::Time::now();
      ros::Time oCurrenTime = vLaserData.header.stamp;

      ////a point clouds in PCL type
      pcl::PointCloud<pcl::PointXYZ> vRawCloud;
      pcl::PointCloud<pcl::PointXYZ> vTransposedCloud;
      sensor_msgs::PointCloud2 vTransposedData;

      ////message from ROS type to PCL type
      pcl::fromROSMsg(vLaserData, vRawCloud);

      //get the size of input
      vTransposedCloud.resize(vRawCloud.size());

      for(int i = 0; i != vRawCloud.size(); ++i ){

         //output in a txt file
         //the storage type of output file is x y z time frames right/left_sensor
         vTransposedCloud.points[i].x = vRawCloud.points[i].z;
         vTransposedCloud.points[i].y = vRawCloud.points[i].x;
         vTransposedCloud.points[i].z = vRawCloud.points[i].y;

      }

      pcl::toROSMsg(vTransposedCloud, vTransposedData);
      vTransposedData.header.frame_id = m_sSLAMChildN;
      vTransposedData.header.stamp = vLaserData.header.stamp;
      m_oLaserPub.publish(vTransposedData);

  }//end if

}



/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &SLAMTrans::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a transposed odometry point to suit the special coordinate system 
Return: none
Others: none
*************************************************/
void SLAMTrans::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
  
  //if(m_iTrajPointNum < 0){

    //set the current time stamp as a file name
    //full name 
   // m_sOutTrajFileName << m_sFileHead << "_Traj_" << oTrajectory.header.stamp << ".txt"; 

  //}
  

  nav_msgs::Odometry oTransposedOdom;
  //count input frames
  m_iTrajPointNum++;

  //output in a txt file
  //
  oTransposedOdom.pose.pose.position.x = oTrajectory.pose.pose.position.z; 
  oTransposedOdom.pose.pose.position.y = oTrajectory.pose.pose.position.x;
  oTransposedOdom.pose.pose.position.z = oTrajectory.pose.pose.position.y - m_fZOffset;
  oTransposedOdom.header.stamp = oTrajectory.header.stamp;
  oTransposedOdom.header.frame_id = m_sSLAMChildN;

  m_oOdomPub.publish(oTransposedOdom);

}
