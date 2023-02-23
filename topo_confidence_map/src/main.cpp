#include "TopologyMap.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "topology_map");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  
  topology_map::TopologyMap TopologyMapping(node,privateNode);

  ros::spin();

  return 0;
}

