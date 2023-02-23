#ifndef SENDHUSKYGOAL_H
#define SENDHUSKYGOAL_H
#include <iostream>
#include <vector>
#include <cmath>

//ros based
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class SendHuskyGoal{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

public:

    //construction
    SendHuskyGoal(ros::NodeHandle & node,
               ros::NodeHandle & nodeHandle);

    //destruction
    ~SendHuskyGoal();

    //handle the odometry from husky robot
    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

    //handle the goal from node generation topic 
    void HandleGoalOdom(const nav_msgs::Odometry & oTrajectory);

    //compute the distance
    float TwoDDistance(const float & fOnex, const float & fOney,
		               const float & fTwox, const float & fTwoy);

    MoveBaseClient m_oAC;//client

private:

	//ros::Subscriber m_oTrajSuber;//suber
	std::string m_sTrajTopic;//topic name

	ros::Subscriber m_oGoalSuber;//suber
	std::string m_sGoalTopic;//topic name

    move_base_msgs::MoveBaseGoal oGoal;//goal information
    //robot basic action client

};

#endif