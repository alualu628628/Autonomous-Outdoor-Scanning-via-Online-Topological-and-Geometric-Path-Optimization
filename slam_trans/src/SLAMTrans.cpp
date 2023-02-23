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
                        :m_pTFListener(new tf::TransformListener),
                         m_bFileNmFlag(false),
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
Output: m_sOdomTargetFrame
        m_sOdomRawFrame
        m_sPointTargetFrame
        m_sPointRawFrame
Return: none
Others: none
*************************************************/
bool SLAMTrans::SetFrameIDs(ros::NodeHandle & private_node){

  
  bool bKnownFlag = true;

  //set the related frame id of odometry topic
  std::string sOdomTargetFrame; 

  if(private_node.getParam("odom_targetframe", sOdomTargetFrame)){

      m_sOdomTargetFrame = sOdomTargetFrame;

                     
    }else{

      m_sOdomTargetFrame = "odom";///<the reference frame
      bKnownFlag = false;

  }//end if


  std::string sOdomRawFrame; 

  if(private_node.getParam("odom_rawframe", sOdomRawFrame)){

      m_sOdomRawFrame = sOdomRawFrame;

                     
    }else{

      m_sOdomRawFrame = "camera_init";///<the target frame indicats where the slam locates
      bKnownFlag = false;

  }//end if


  //set the related frame id of laser point cloud topic
  std::string sPointTargetFrame; 

  if(private_node.getParam("clouds_targetframe", sPointTargetFrame)){

      m_sPointTargetFrame = sPointTargetFrame;

                     
    }else{

      m_sPointTargetFrame = "odom";///<the target frame indicats where the slam locates
      bKnownFlag = false;

  }//end if

  std::string sPointRawFrame; 

  if(private_node.getParam("clouds_rawframe", sPointRawFrame)){

      m_sPointRawFrame = sPointRawFrame;
                     
    }else{

      m_sPointRawFrame = "camera_init";///<the target frame indicats where the slam locates
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
Output: m_fOdomZOffset - odometry height value offset
        m_fPointZOffset - slam obtaned point height value offset
Return: none
Others: none
*************************************************/
bool SLAMTrans::GetHeightOffset(ros::NodeHandle & private_node){

    double dOdomZOffset;

    if(private_node.getParam("odom_zoffset", dOdomZOffset))
        
       m_fOdomZOffset = float(dOdomZOffset);

    else

       m_fOdomZOffset = 0.0;

    double dPointZOffset;

    if(private_node.getParam("point_zoffset", dPointZOffset))
        
       m_fPointZOffset = float(dPointZOffset);

    else

       m_fPointZOffset = 0.0;

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
/***************************************************************
void SLAMTrans::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{
  

  
  //count input frames
  m_iFrames++;

  //ROS_ERROR_STREAM("m_fPointZOffset: " << m_fPointZOffset);


  if(!(m_iFrames%m_iSampleNum)){

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
         vTransposedCloud.points[i].z = vRawCloud.points[i].y + m_fPointZOffset;

      }

      pcl::toROSMsg(vTransposedCloud, vTransposedData);
      vTransposedData.header.frame_id = m_sOdomRawFrame;
      vTransposedData.header.stamp = vLaserData.header.stamp;
      m_oLaserPub.publish(vTransposedData);

  }//end if


}
**********************************************************************/


void SLAMTrans::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{
  
  //count input frames
  m_iFrames++;

  //ROS_ERROR_STREAM("m_fPointZOffset: " << m_fPointZOffset);
  if(!(m_iFrames%m_iSampleNum)){

    // wait for transform from source framework to target framework
    m_pTFListener->waitForTransform(m_sPointTargetFrame, vLaserData.header.frame_id, ros::Time::now(), ros::Duration(5.0));  
  
    // Create a container for the data.  
    sensor_msgs::PointCloud2 vTransposedData(vLaserData);

    //converted point clouds, where PC means the point clouds
    pcl_ros::transformPointCloud(m_sPointTargetFrame, vLaserData, vTransposedData, *m_pTFListener);  

    //set frame and time
    vTransposedData.header.frame_id = m_sPointTargetFrame;
    vTransposedData.header.stamp = vLaserData.header.stamp;//timestmap is the same with the input

    //set output
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

  //ROS_ERROR_STREAM("m_fOdomZOffset: "<< m_fOdomZOffset);

  nav_msgs::Odometry oTransposedOdom;
  //count input frames
  m_iTrajPointNum++;

  //output in a txt file
  //
  oTransposedOdom.header.stamp = oTrajectory.header.stamp;
  oTransposedOdom.header.frame_id = m_sOdomTargetFrame;
  oTransposedOdom.child_frame_id = m_sOdomRawFrame;

  oTransposedOdom.pose.pose.position.x = oTrajectory.pose.pose.position.z; 
  oTransposedOdom.pose.pose.position.y = oTrajectory.pose.pose.position.x;
  oTransposedOdom.pose.pose.position.z = oTrajectory.pose.pose.position.y - m_fOdomZOffset;
  oTransposedOdom.pose.pose.orientation.x = oTrajectory.pose.pose.orientation.z;
  oTransposedOdom.pose.pose.orientation.y = oTrajectory.pose.pose.orientation.x;
  oTransposedOdom.pose.pose.orientation.z = oTrajectory.pose.pose.orientation.y;
  oTransposedOdom.pose.pose.orientation.w = oTrajectory.pose.pose.orientation.w;

  oTransposedOdom.twist.twist.angular.x = oTrajectory.twist.twist.angular.x ;
  oTransposedOdom.twist.twist.angular.y = oTrajectory.twist.twist.angular.y ;
  oTransposedOdom.twist.twist.angular.z = oTrajectory.twist.twist.angular.z ;

  oTransposedOdom.twist.twist.linear.x = oTrajectory.twist.twist.angular.x ;
  oTransposedOdom.twist.twist.linear.y = oTrajectory.twist.twist.angular.y ;
  oTransposedOdom.twist.twist.linear.z = oTrajectory.twist.twist.angular.z ;

  m_oOdomPub.publish(oTransposedOdom);

}
