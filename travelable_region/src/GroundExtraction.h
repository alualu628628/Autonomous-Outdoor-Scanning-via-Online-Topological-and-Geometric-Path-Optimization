#ifndef GROUNDEXTRACTION_H
#define GROUNDEXTRACTION_H

#include "INSAC.h"
#include "CircularVector.h"
#include "Boundary.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>




  // Trajectory state data. 
typedef struct TrajectoryPoint{

    // The time of the measurement leading to this state (in seconds). //
    ros::Time oTimeStamp;

	//********angles***********
    //float roll;    ///< The current roll angle. 
    //float pitch;    ///< The current pitch angle.
    //float yaw;   ///< The current yaw angle. 

	//********coordinate value****************
    // The global trajectory position in 3D space. 
    pcl::PointXYZ position;

} TrajectoryPoint;


class GroundExtraction{

public:
    
    GroundExtraction(ros::NodeHandle & node, 
                ros::NodeHandle & private_node);                      

    ////**performance function**////
    //set path of output file where you want to put on
    bool GetOutputPath(ros::NodeHandle & private_node);

    //set whether output txt file
    bool GetTxtOutputFlag(ros::NodeHandle & private_node);

    //set sampling value
    bool GetSamplingNum(ros::NodeHandle & private_node);
    
    //set the query topic of point clouds 
    bool GetLaserTopic(ros::NodeHandle & private_node);

    //set the query topiuc of published odometry
    bool GetOdomTopic(ros::NodeHandle & private_node);
    
    //set the thresholds of GP-INSAC method
    void GetGPINSACThrs(ros::NodeHandle & private_node);

    ////**performance function**////
    //process point cloud 
    void HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData);

    //process trajectory points
    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);
    
    //inquire corresponding trajectoy points of a query lidar frame
    pcl::PointXYZ ComputeQueryTraj(const ros::Time & oQueryTime);

    //compute the corresponding trajectoy points of a query lidar frame
    inline void InterpolateTraj(const TrajectoryPoint & oCurrent, const TrajectoryPoint & oPast,
                                                                          const float& ratio, pcl::PointXYZ & oResTraj);
    //output function
    //output ground points
    void OutputGroundPoints(pcl::PointCloud<pcl::PointXYZ> & vCloud, const ros::Time & oStamp);
    //output all points
    void OutputAllPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, 
                                         const std::vector<int> & vRes, const ros::Time & oStamp);
private:

    //GP-INSAC algorithm related thresholds
    GPINSACThrs m_oGPThrs;

    //the subscirbers below are to hear (record) point clouds produced by the Hesai devices
    ros::Subscriber m_oLaserSuber;//
    ros::Subscriber m_oOdomSuber;

    //publish related topic
    ros::Publisher m_oGroundPub;
    ros::Publisher m_oObstaclePub;
    ros::Publisher m_oBoundPub;
    ros::Publisher m_oHighOdomPub;

    //topic name of point clouds published by sensor
    std::string m_sLaserTopic;///<laser topic name
    std::string m_sOdomTopic;///<trajectory topic name

    //output file name
    std::string m_sFileHead;
    std::stringstream m_sOutPCFileName; ///<full name of output txt that records the point clouds//defind it in the function
    std::stringstream m_sOutTrajFileName;///<full name of output txt that records the trajectory point 
  
     //whether the outfile got a full name or not 
    bool m_bFileNmFlag;
    bool m_bTxtOutFlag;

    //the frame number of input
    int m_iFrames;
    int m_iTrajPointNum;

    //the sample interval of frames (e.g., 2 means jumping 1 frame, 3 jumps 2 frame)
    int m_iSampleNum;
    //sampling

    //the history data among a given secs of trajectory points
    CircularVector<TrajectoryPoint> vTrajHistory;


};

#endif




