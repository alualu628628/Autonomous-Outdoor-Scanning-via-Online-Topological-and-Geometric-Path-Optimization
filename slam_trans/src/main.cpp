#include "SLAMTrans.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "slam_transfor");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  SLAMTrans SLAMTransfor(node,privateNode);

  ros::spin();

  return 0;
}

