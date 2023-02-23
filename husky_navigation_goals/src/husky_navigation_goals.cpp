#include "SendHuskyGoal.h"


int main(int argc, char** argv){

	ros::init(argc, argv, "husky_navigation_goals");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
  
    SendHuskyGoal HuskyGoalSender(node,privateNode);

    ros::spin();

    return 0;

}
