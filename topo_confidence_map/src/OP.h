#ifndef OP_H
#define OP_H
#include "ConfidenceMap.h"


namespace topology_map {
//define a node type
//node type presents the position and its relative grid id 
struct Node{
	//the position
	pcl::PointXYZ point;
	//the corresponding grid idx of node
    int gridIdx;
    //the parent id
    int parentIdx;
    //visited or not
    bool visitedFlag;

};

//OP class
//OP is the orienteering problem
//
class OP{


public:
	//constructor
	OP();
	//destructor
	~OP();

    //initial the orginal robot location as the first node
	void Initial(const pcl::PointXYZ & oOriginPoint,
	             const grid_map::GridMap & oFeatureMap);

	//judge whether the robot is close/near the goal node (movement target)
	//choose the next best view as soon as the robot is inside the target region  
	bool NearGoal(const pcl::PointXYZ & oOdomPoint,
                             const int & iProcessFrame,
	                           float fDisDiffThr = 0.5,
	                            int iFirstTripThr = 10);

	//get the current node index
	void GetNewNode(const int & iNewNodeIdx,
			        const pcl::PointXYZ & oNodePoints);
	void GetNewNode(const std::vector<int> & vNewNodeIdxs,
			        const std::vector<pcl::PointXYZ> & vNewNodeClouds);

	//get the newly generated nodes
	void GetNewNodeSuppression(const int & iNewNodeIdx,
			                   const pcl::PointXYZ & oNodePoint,
					           float fSuppressionR = 5.0);
	void GetNewNodeSuppression(const std::vector<int> & vNewNodeIdxs,
			                   const std::vector<pcl::PointXYZ> & vNewNodeClouds,
					           float fSuppressionR = 5.0);

	//compute the Euclidean distance
	float TwoDDistance(const pcl::PointXYZ & oQueryPoint,
		               const pcl::PointXYZ & oTargetPoint);


	//the cost function of a pairs of nodes
    float CostFunction(const Node & oQueryNode,
	                   const Node & oTargetNode);

    //greed method
    bool GTR(const pcl::PointXYZ & oCurOdom,
	         const std::vector<ConfidenceValue> & vConfidenceMap);

	//Outout the history of traveling nodes
    void OutputPastNodes(std::vector<pcl::PointXYZ> & vOutputNodes);

    void OutputPlanNodes(std::vector<pcl::PointXYZ> & vOutputNodes);

    void OutputUnvisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes);
    //get goal position
    void OutputGoalPos(pcl::PointXYZ & oGoalPoint);
    //get goal grid idx
    void OutputGoalPos(int & iGoalGridIdx);

private:

	//the grid index of which current robot is 
	int m_iCurrNodeIdx;//this value will be only changed in plan function
	//unvisited nodes or nodes to be visited (id)
    std::vector<int> m_vPlanNodeIdx;
    //past nodes or nodes has been visited (id)
    std::vector<int> m_vPastNodeIdx;
    //all generated nodes
	std::vector<Node> m_vAllNodes;

};

}/*namespace*/

#endif
