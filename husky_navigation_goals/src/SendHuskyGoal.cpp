#include "SendHuskyGoal.h"

/*************************************************
Function: ~OP
Description: destruction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
SendHuskyGoal::SendHuskyGoal(ros::NodeHandle & node,
                             ros::NodeHandle & nodeHandle):m_oAC("move_base", true){

    //parameters
	//nodeHandle.param("robotodom_topic", m_sTrajTopic, std::string("/odometry/filtered"));

	nodeHandle.param("goalodom_topic", m_sGoalTopic, std::string("/goal_odom"));

    //wait for the action server to come up
	while(!m_oAC.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    //initial goal
	oGoal.target_pose.pose.position.x = 0.0;
    oGoal.target_pose.pose.position.y = 0.0;
    oGoal.target_pose.pose.position.z = 0.0;
    oGoal.target_pose.header.frame_id = "odom";

    //subscribe trajectory topic
    //m_oTrajSuber = nodeHandle.subscribe(m_sTrajTopic, 1, &SendHuskyGoal::HandleTrajectory, this);
    //subscribe action goal topic
    m_oGoalSuber = nodeHandle.subscribe(m_sGoalTopic, 1, &SendHuskyGoal::HandleGoalOdom, this);

}


/*************************************************
Function: ~OP
Description: destruction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
SendHuskyGoal::~SendHuskyGoal(){


}




/*************************************************
Function: ~OP
Description: destruction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
//handle the odometry from husky robot
void SendHuskyGoal::HandleTrajectory(const nav_msgs::Odometry & oTrajectory){

    //using the current odom direction as the goal direction
    oGoal.target_pose.pose.orientation = oTrajectory.pose.pose.orientation;

}



/*************************************************
Function: ~OP
Description: destruction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
//handle the goal from node generation topic 
void SendHuskyGoal::HandleGoalOdom(const nav_msgs::Odometry & oInputGoalOdom){

 	float fDiff = TwoDDistance(oInputGoalOdom.pose.pose.position.x, oInputGoalOdom.pose.pose.position.y,
		                       oGoal.target_pose.pose.position.x, oGoal.target_pose.pose.position.y);

    //find a new goal
    //if current goal is different with the new input goal
 	if(fDiff > 0.2){

        //send a new goal
        //timestamp
        oGoal.target_pose.header.stamp = ros::Time::now();
        //new position
        oGoal.target_pose.pose.position.x = oInputGoalOdom.pose.pose.position.x;
        oGoal.target_pose.pose.position.y = oInputGoalOdom.pose.pose.position.y;
        oGoal.target_pose.pose.orientation.w = 1.0; 

        ROS_INFO("Sending goal at x=[%f], y=[%f].", oGoal.target_pose.pose.position.x, 
        	                                        oGoal.target_pose.pose.position.y);

        m_oAC.sendGoal(oGoal);

        //m_oAC.waitForResult();
   
        //if(m_oAC.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        //    ROS_INFO("Hooray, the base moved 1 meter forward");
        //else
        //    ROS_INFO("The base failed to move forward 1 meter for some reason");

    }//end if

}




/*************************************************
Function: ~OP
Description: destruction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
//compute the distance
float SendHuskyGoal::TwoDDistance(const float & fOneX, const float & fOneY,
		                           const float & fTwoX, const float & fTwoY){

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(fOneX - fTwoX, 2.0f)
		      + pow(fOneY - fTwoY, 2.0f));

}