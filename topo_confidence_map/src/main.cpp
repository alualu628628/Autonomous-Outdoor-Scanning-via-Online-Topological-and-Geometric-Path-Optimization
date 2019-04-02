#include "TopologyMap.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "topology_map");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  topology_map::TopologyMap TopologyMapping(node,privateNode);

  ros::Duration(2.0).sleep();

  ros::Rate r(0.1); // 1 hz
  while (ros::ok())
  {
    TopologyMapping.convertAndPublishMap();
    ros::spinOnce();
    r.sleep();
  }
  //ros::spin();

  return 0;
}

