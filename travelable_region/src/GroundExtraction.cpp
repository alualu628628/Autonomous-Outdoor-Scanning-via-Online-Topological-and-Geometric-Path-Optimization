#include "GroundExtraction.h"


/*************************************************
Function: GroundExtraction
Description: constrcution function for GroundExtraction class
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
GroundExtraction::GroundExtraction(ros::NodeHandle & node, 
                        ros::NodeHandle & private_node)
                        :m_bFileNmFlag(false),
                         m_iFrames(-1),
                         m_iTrajPointNum(-1){

    //get the value of parameters
    //system parameters
    GetOutputPath(private_node);
    
    //check output file or not
    GetTxtOutputFlag(private_node);

    //point cloud related
    GetSamplingNum(private_node);
    
    //get laser topic name
    GetLaserTopic(private_node);
    
    //get trajectory topic name
    GetOdomTopic(private_node);
    
    //get gp-insac related thresholds
    GetGPINSACThrs(private_node);

    //subscribe (hear) the point cloud topic from laser on right side 
    m_oLaserSuber = node.subscribe(m_sLaserTopic, 2, &GroundExtraction::HandlePointClouds, this);
    //subscribe (hear) the odometry information
    m_oOdomSuber = node.subscribe(m_sOdomTopic, 2, &GroundExtraction::HandleTrajectory, this);
    //publish topic of point clouds 
    m_oGroundPub = node.advertise<sensor_msgs::PointCloud2>("/ground_points", 2);
    //publish topic of boundary clouds 
    m_oBoundPub = node.advertise<sensor_msgs::PointCloud2>("/boundary_points", 2);
    //publish topic of point clouds 
    m_oObstaclePub  = node.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 2);
    //publish odometry with high value
    m_oHighOdomPub = node.advertise<nav_msgs::Odometry>("odom_lidar", 2);
}

/*************************************************
Function: GetOutputPath
Description: inital function for m_sFileHead
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sFileHead
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetOutputPath(ros::NodeHandle & private_node){
        
    std::string sFileHead;

    if(private_node.getParam("output_path", sFileHead)){

      m_sFileHead = sFileHead;

      return true;
                     
      }else{

      m_sFileHead = "./";///<in the default user's path (in usual)

      return false;

    }//end if

}

/*************************************************
Function: GetTxtOutputFlag
Description: inital function for m_sTxtOutFlag
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sTxtOutFlag
Return: none
Others: none
*************************************************/

bool GroundExtraction::GetTxtOutputFlag(ros::NodeHandle & private_node){
        
   bool bTxtOutFlag;

    if(private_node.getParam("txtoutput_flag", bTxtOutFlag)){

      m_bTxtOutFlag = bTxtOutFlag;

      return true;
                     
      }else{

      m_bTxtOutFlag = false;///<don't output result in txt

      return false;

    }//end if

}

/*************************************************
Function: GetLaserTopic
Description: inital function for m_sLaserTopic
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sLaserTopic
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetLaserTopic(ros::NodeHandle & private_node){
        
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
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_iSampleNum
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetSamplingNum(ros::NodeHandle & private_node){
        
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
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sOdomTopic
Return: none
Others: none
*************************************************/
    //trajectory related 
bool GroundExtraction::GetOdomTopic(ros::NodeHandle & private_node){

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
Function: GetGPINSACThrs
Description: inital function for GP-INSAC related thresholds
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: thresholds
Return: none
Others: none
*************************************************/
void GroundExtraction::GetGPINSACThrs(ros::NodeHandle & private_node){

     int iSector_num;
     if(private_node.getParam("sector_num", iSector_num))
        m_oGPThrs.iSector_num = iSector_num;

     //seed selection
     double fDisThr;
     if(private_node.getParam("seed_radius", fDisThr))
        m_oGPThrs.fDisThr = float(fDisThr);
    
    double fZLower;
    if(private_node.getParam("seed_lower", fZLower))
        m_oGPThrs.fZLower = float(fZLower);

    double fZUpper;
    if(private_node.getParam("seed_upper", fZUpper))
        m_oGPThrs.fZUpper = float(fZUpper);

    //GP model thresholds
    double dLScale;
    if(private_node.getParam("gp_lscale", dLScale))
        m_oGPThrs.dLScale = dLScale;

    double dSigmaF;
    if(private_node.getParam("gp_sigmaF", dSigmaF))
        m_oGPThrs.dSigmaF = dSigmaF;

    double dSigmaN;
    if(private_node.getParam("gp_sigmaN", dSigmaN))
        m_oGPThrs.dSigmaN = dSigmaN;

    //insac thresholds
    double fModelThr;
    if(private_node.getParam("insac_model", fModelThr))
        m_oGPThrs.fModelThr = float(fModelThr);

    double fDataThr;
    if(private_node.getParam("insac_data", fDataThr))
        m_oGPThrs.fDataThr = float(fDataThr);

}

/*************************************************
Function: OutputGroundPoints
Description: output the result of point clouds in a txt file
Calls: none
Called By: HandlePointClouds
Table Accessed: none
Table Updated: none
Input: vCloud the final result cloud
       oStamp the time stamp of this input point clouds
Output: a point cloud txt file which record ground points only
Return: none
Others: none
*************************************************/
void GroundExtraction::OutputGroundPoints(pcl::PointCloud<pcl::PointXYZ> & vCloud, const ros::Time & oStamp){
  

  if(m_bTxtOutFlag){
    
    //if this is the first time of calling this function
    if(!m_bFileNmFlag){

    //set the current time stamp as a file name
    //full name 
    m_sOutPCFileName << m_sFileHead << "_PC_" << oStamp << ".txt"; 

    m_bFileNmFlag = true;

    }


    //output
    std::ofstream oRecordedFile;
    oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

    for(int i = 0; i != vCloud.size(); ++i ){

      //output in a txt file
      //the storage type of output file is x y z time frames right/left_sensor
      oRecordedFile << vCloud.points[i].x << " "
                    << vCloud.points[i].y << " "
                    << vCloud.points[i].z << " " 
                    << oStamp << " "
                    << std::endl;
    }//end for         

     oRecordedFile.close();

  }//end if(m_bTxtOutFlag)

}


/*************************************************
Function: OutputAllPoints
Description: output all of point clouds with class label in a txt file
Calls: none
Called By: HandlePointClouds
Table Accessed: none
Table Updated: none
Input: pCloud one frame of point clouds in normal coordinate system (z is toward up)
       vRes the label result of each point
       oStamp the time stamp of this input point clouds
Output: a point cloud txt file which record all points with class label
Return: none
Others: none
*************************************************/
void GroundExtraction::OutputAllPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, 
                                          const std::vector<int> & vRes,
                                          const ros::Time & oStamp){
  
  if(m_bTxtOutFlag){

     //if this is the first time of calling this function
     if(!m_bFileNmFlag){

       //set the current time stamp as a file name
       //full name 
       m_sOutPCFileName << m_sFileHead << "_PC_" << oStamp << ".txt"; 

       m_bFileNmFlag = true;
     }

     //output
     std::ofstream oRecordedFile;
     oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

     for(int i = 0; i != pCloud->points.size(); ++i ){

         //output in a txt file
         //the storage type of output file is x y z time frames right/left_sensor
         oRecordedFile << pCloud->points[i].x << " "
                       << pCloud->points[i].y << " "
                       << pCloud->points[i].z << " " 
                       << vRes[i] << " "
                       << oStamp << " "
                       << std::endl;
      }//end for         

      oRecordedFile.close();

  }//end if m_bTxtOutFlag

}
/*************************************************
Function: HandleRightLaser
Description: a callback function in below: 
             node.subscribe(m_sLaserTopic, 5, &GroundExtraction::HandlePointClouds, this);
Calls: CheckTruthPoint
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void GroundExtraction::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{
  
  //count input frames
  m_iFrames++;
  //ROS_INFO("frame number: %d", m_iFrames);

  if(!(m_iFrames%m_iSampleNum)){

      ////a point clouds in PCL type
      pcl::PointCloud<pcl::PointXYZ> vInputCloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
      ////message from ROS type to PCL type
      pcl::fromROSMsg(vLaserData, vInputCloud);
      
      //get right point clouds from LOAM output
      for(int i = 0; i != vInputCloud.size(); ++i ){
         
         pcl::PointXYZ oArgPoint;
         oArgPoint.x = vInputCloud.points[i].x;
         oArgPoint.y = vInputCloud.points[i].y;
         oArgPoint.z = vInputCloud.points[i].z;
         vOneCloud->points.push_back(oArgPoint);

      }  

      //******************deviding section******************
      //a class to divide point clouds into the given number of sectors
      DivideSector oSectorDivider(m_oGPThrs.iSector_num);
      //compute the corresponding trajectory point
      pcl::PointXYZ oCurrentTrajP;
      
      //if have corresponding trajectory point (viewpoint)
      if( vTrajHistory.size() ){
        //
        oCurrentTrajP = ComputeQueryTraj(vLaserData.header.stamp);
        //
        oSectorDivider.SetOriginPoint(oCurrentTrajP);
      }
         
      //ROS_INFO("Querytime: %f, have trajpoint: %d", vLaserData.header.stamp.toSec(), bCurTrajPFlag);
      
      //preparation
      std::vector<std::vector<int> > oPointSecIdxs;///<point index reorganization according to sectors
      std::vector<std::vector<GroundFeature> > vGroundFeatures = oSectorDivider.ComputePointSectorIdxs(*vOneCloud, oPointSecIdxs);
      
      std::vector<std::vector<int> > vAllGroundRes;///<point value according to oPointSecIdxs
      for (int i = 0; i != vGroundFeatures.size(); ++i) {
          std::vector<int> vTmpRes(vGroundFeatures[i].size(),0);
          vAllGroundRes.push_back(vTmpRes);
      }

      //******************deviding section******************
      //***********and adopt GP-INSAC algorithm*************
      //to a sector
      for (int is = 0; is != oPointSecIdxs.size(); ++is) {

          //***********GP algorithm***********
          //new the class of Gaussian Process algorithm class
          GaussianProcessRegression<float> GPR(1, 1);
          
          //paramters
          GPR.SetHyperParams(m_oGPThrs.dLScale, m_oGPThrs.dSigmaF, m_oGPThrs.dSigmaN);

          INSAC GPINSAC(float(m_oGPThrs.dSigmaN), m_oGPThrs.fModelThr, m_oGPThrs.fDataThr);

          //select the initial training input
          GPINSAC.SetSeedThreshold(m_oGPThrs.fDisThr,m_oGPThrs.fZLower,m_oGPThrs.fZUpper);
          GPINSAC.SelectSeeds(vGroundFeatures[is]);

          //***********INSAC algorithm***********
          bool bGrowFlag = true;
          
          //looping until there is not new seed to be involved in current sector
          while(bGrowFlag){
               
               //new training vector - NEW input, NEW means the input does not include old one
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainFeaVec;
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainTruVec;
               GPINSAC.ToAddTrainSamples(vTrainFeaVec, vTrainTruVec, vGroundFeatures[is]);

               //new test vector - NEW output
               std::vector<Eigen::Matrix<float, 1, 1> > vTestFeaVec;
               std::vector<Eigen::Matrix<float, 1, 1> > vTestTruVec;
               GPINSAC.ToAddTestSamples(vTestFeaVec, vTestTruVec, vGroundFeatures[is]);
               
               //training
               GPR.AddTrainingDatas(vTrainFeaVec, vTrainTruVec);
    
               //regression (prediction)
               std::vector<int> vOneLoopLabels;
               for (size_t k = 0; k < vTestFeaVec.size(); k++) {
        
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
                    
                    //below is for parameter optimization only
                    //std::cout <<"vTestFeaVec[k] :"<< vTestFeaVec[k] << std::endl;

                    //regression of new input based on the computed training matrix
                    if (GPR.Regression(vPredValue, vPredVar, vTestFeaVec[k])){
                       
                       //belows are for parameter optimization only
                       //std::cout << "predict" << vPredValue(0) << std::endl;
                       //std::cout << "groundtruth" << vTestTruVec[k](0) << std::endl;
                       //std::cout << "var" << vPredVar(0) << std::endl;

                       //judgement the result is meet the model or not
                       int iPointLabel = GPINSAC.Eval(vTestTruVec[k](0),vPredValue(0), vPredVar(0));
                       vOneLoopLabels.push_back(iPointLabel);

                    }else

                       vOneLoopLabels.push_back(-1);//obstacle
               }
               
               //refresh the remaining unknown points
               if(GPINSAC.AssignIdx(vOneLoopLabels))
                  bGrowFlag = false;
    
          }//end while

          //assigment in current sector
          std::vector<int> vGroundLabels;
          GPINSAC.OutputRes(vGroundLabels);
      
          for (int j = 0; j != vGroundLabels.size(); ++j)
              vAllGroundRes[is][j] = vGroundLabels[j];

      }//end is
  
       //***********boundary extraction***********

      //assigment of result in whole point clouds
      std::vector<int> vCloudRes(vOneCloud->points.size(),0);
       //assignment in whole point clouds
       for (int is = 0; is != oPointSecIdxs.size(); ++is) {
            for (int j = 0; j != oPointSecIdxs[is].size(); ++j) {
               
                  int iPointIdx = oPointSecIdxs[is][j];
                  vCloudRes[iPointIdx] = vAllGroundRes[is][j];

            }//end for j
       }//end for i

       //new a boundary class
       Boundary oBounder;
       //input the segment labels
       oBounder.GetSegmentClouds(vOneCloud, vCloudRes);
       //compute boundary point
       oBounder.ComputeBoundary();
       //output the boundary cloud
       oBounder.OutputBoundClouds(vCloudRes);

       //************output value******************
       pcl::PointCloud<pcl::PointXYZ> vGroundCloud;
       sensor_msgs::PointCloud2 vGroundPub;
       pcl::PointCloud<pcl::PointXYZ> vObstacleCloud;
       sensor_msgs::PointCloud2 vObstaclePub;
       pcl::PointCloud<pcl::PointXYZ> vBoundCloud;
       sensor_msgs::PointCloud2 vBoundPub;
      
       for (int i = 0; i != vCloudRes.size(); ++i) {
                         
                  //if point is a ground point
                  if( vCloudRes[i] == 1){
                       //take data
                       vGroundCloud.push_back(vInputCloud.points[i]);
                  //if point is an obstacle point
                  }else if(vCloudRes[i] == -1){
                      //take data
                      vObstacleCloud.push_back(vInputCloud.points[i]);
                  //if point is an boundary point
                  }else if(  vCloudRes[i] == 2)
                      //take data
                      vBoundCloud.push_back(vInputCloud.points[i]);

       }//end for i

       //publish ground points
       pcl::toROSMsg(vGroundCloud, vGroundPub);
       vGroundPub.header.frame_id = "odom";
       vGroundPub.header.stamp = vLaserData.header.stamp;
       m_oGroundPub.publish(vGroundPub);

       //publish ground points
       pcl::toROSMsg(vBoundCloud, vBoundPub);
       vBoundPub.header.frame_id = "odom";
       vBoundPub.header.stamp = vLaserData.header.stamp;
       m_oBoundPub.publish(vBoundPub);

       //publish obstacle points
       pcl::toROSMsg(vObstacleCloud, vObstaclePub);
       vObstaclePub.header.frame_id = "odom";
       vObstaclePub.header.stamp =vLaserData.header.stamp;
       m_oObstaclePub.publish(vObstaclePub);

       //OutputGroundPoints(vGroundCloud, vLaserData.header.stamp);
       OutputAllPoints(vOneCloud, vCloudRes, vLaserData.header.stamp);

  }//end if m_iFrames%m_iSampleNum (down sampling)

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
void GroundExtraction::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
  
  
  //std::cout << "m_bTxtOutFlag value: " << m_bTxtOutFlag << std::endl;

  if(m_iTrajPointNum < 0 && m_bTxtOutFlag){

    //set the current time stamp as a file name
    //full name 
    m_sOutTrajFileName << m_sFileHead << "_Traj_" << oTrajectory.header.stamp << ".txt"; 

  }
  
  //count input frames
  m_iTrajPointNum++;
  
  //save the into the memory
  //save the position of trajectory
  TrajectoryPoint oTrajPoint;
  oTrajPoint.position.x = oTrajectory.pose.pose.position.x;//z in loam is x
  oTrajPoint.position.y = oTrajectory.pose.pose.position.y;//x in loam is y
  oTrajPoint.position.z = oTrajectory.pose.pose.position.z;//y in loam is z
  //save record time
  oTrajPoint.oTimeStamp =  oTrajectory.header.stamp;

  vTrajHistory.push(oTrajPoint);

  nav_msgs::Odometry oCurrOdom = oTrajectory;
  //oCurrOdom.header.stamp = ros::Time::now();
  //oCurrOdom.header.frame_id = "odom";

  //set the position
  //oCurrOdom.pose.pose.position.x = oTrajPoint.position.x;
  //oCurrOdom.pose.pose.position.y = oTrajPoint.position.y;
  oCurrOdom.pose.pose.position.z = oCurrOdom.pose.pose.position.z + 0.583;
  m_oHighOdomPub.publish(oCurrOdom);

  if(m_bTxtOutFlag){

     //output
     std::ofstream oTrajFile;
     oTrajFile.open(m_sOutTrajFileName.str(), std::ios::out | std::ios::app);

     //output in a txt file
     oTrajFile << oTrajPoint.position.x << " "
              << oTrajPoint.position.y << " "
              << oTrajPoint.position.z << " " 
              << oTrajPoint.oTimeStamp << " "
              << m_iTrajPointNum << " "
              << std::endl;
     oTrajFile.close();

  }//if m_bTxtOutFlag

}

/*************************************************
Function: InterpolateTraj
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void GroundExtraction::InterpolateTraj(const TrajectoryPoint & oCurrent,
                                                                      const TrajectoryPoint & oPast,
                                                                      const float& ratio,
                                                                      pcl::PointXYZ & oResTraj){
    //The ratio is from the interpolated value to oCurrent value 
    float invRatio = 1 - ratio;
    //p+(c-p)(1-r)
    oResTraj.x = oCurrent.position.x * invRatio + oPast.position.x * ratio;
    oResTraj.y = oCurrent.position.y * invRatio + oPast.position.y * ratio;
    oResTraj.z = oCurrent.position.z * invRatio + oPast.position.z * ratio;

}

/*************************************************
Function: ComputeQueryTraj
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
pcl::PointXYZ GroundExtraction::ComputeQueryTraj(const ros::Time & oQueryTime){

    pcl::PointXYZ oResTraj;
    //clear the output
    oResTraj.x = 0.0;
    oResTraj.y = 0.0;
    oResTraj.z = 0.0;
    //index
    int iTrajIdx = 0;
    //time different
    double timeDiff = (oQueryTime - vTrajHistory[iTrajIdx].oTimeStamp).toSec();
    //search the most recent time
    while (iTrajIdx < vTrajHistory.size() - 1 && timeDiff > 0) {
        //increase index
        iTrajIdx++;
        //time different
        timeDiff = (oQueryTime - vTrajHistory[iTrajIdx].oTimeStamp).toSec();
    }

    //if the querytime is out of the stored time section 
    if (iTrajIdx == 0 || timeDiff > 0) {
       //turn back zero
       oResTraj.x = vTrajHistory[iTrajIdx].position.x;
       oResTraj.y = vTrajHistory[iTrajIdx].position.y;
       oResTraj.z = vTrajHistory[iTrajIdx].position.z;

    } else {//if it is between two stored times
       //get the ratio
        //ROS_INFO("Trajtime between: %f and %f", vTrajHistory[iTrajIdx].oTimeStamp.toSec(), vTrajHistory[iTrajIdx - 1].oTimeStamp.toSec());

        float ratio = - timeDiff / (vTrajHistory[iTrajIdx].oTimeStamp - vTrajHistory[iTrajIdx - 1].oTimeStamp).toSec();
        //interpolate an accuracy value
        InterpolateTraj(vTrajHistory[iTrajIdx], vTrajHistory[iTrajIdx - 1], ratio, oResTraj);
    }

    return oResTraj;

}