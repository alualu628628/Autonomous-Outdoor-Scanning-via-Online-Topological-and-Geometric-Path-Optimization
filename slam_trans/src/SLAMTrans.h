#ifndef SLAMTRANS_H
#define SLAMTRANS_H

#include <iostream>
#include <sstream>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/transforms.h"  

class SLAMTrans{

public:
    
    SLAMTrans(ros::NodeHandle & node, 
                ros::NodeHandle & private_node);                      


    ~SLAMTrans(){

        delete m_pTFListener;

    }

    ////**performance function**////
    //set path of output file where you want to put on
    bool GetOutputPath(ros::NodeHandle & private_node);

    //set sampling value
    bool GetSamplingNum(ros::NodeHandle & private_node);
    
    //set the query topic of point clouds 
    bool GetLaserTopic(ros::NodeHandle & private_node);

    //set the query topiuc of published odometry
    bool GetOdomTopic(ros::NodeHandle & private_node);

    //set the topics to be published
    bool SetOutTopics(ros::NodeHandle & private_node);

    //set the frame id of published data
    bool SetFrameIDs(ros::NodeHandle & private_node);

    //get offset of height value
    bool GetHeightOffset(ros::NodeHandle & private_node);

    ////**performance function**////
    //process point cloud 
    void HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData);

    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

private:

    //the subscirber hearing the structure information and current transformation of TF (given by ros system itself)
    tf::TransformListener * m_pTFListener;

    //the subscirber below is to hear (record) point clouds produced by the Hesai devices
    ros::Subscriber m_oLaserSuber;//
    ros::Subscriber m_oOdomSuber;

    //publish related topic
    ros::Publisher m_oLaserPub;
    ros::Publisher m_oOdomPub;

    //topic name of point clouds published by sensor
    std::string m_sLaserTopic;///<laser topic name
    std::string m_sOdomTopic;///<trajectory topic name

    std::string m_sOdomOutTopic;///<transposed odom output topic name
    std::string m_sLaserOutTopic;///<transposed point cloud topic name

    std::string m_sOdomTargetFrame;///<slam child frame name 
    std::string m_sOdomRawFrame;///<slam parent frame name
    std::string m_sPointTargetFrame;
    std::string m_sPointRawFrame;

    //output file name
    std::string m_sFileHead;
    std::stringstream m_sOutPCFileName; ///<full name of output txt that records the point clouds//defind it in the function
    std::stringstream m_sOutTrajFileName;///<full name of output txt that records the trajectory point 
  
     //whether the outfile got a full name or not 
    bool m_bFileNmFlag;
    //the frame number of input
    int m_iFrames;
    int m_iTrajPointNum;

    //the sample interval of frames (e.g., 2 means jumping 1 frame, 3 jumps 2 frame)
    int m_iSampleNum;
    
    //a offset perhaps is useful in computing the actual pose of robot  
    float m_fOdomZOffset;
    float m_fPointZOffset;

};

#endif




