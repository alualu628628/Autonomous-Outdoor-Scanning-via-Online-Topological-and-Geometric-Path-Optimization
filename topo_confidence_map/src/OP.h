#ifndef OP_H
#define OP_H
#include <queue>
#include "BranchBound.h"
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
    //label wide grid 
    bool wideFlag;

    Node(){

        visitedFlag = false;
    	wideFlag = false;
    };

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
	bool NearGoal(const std::queue<pcl::PointXYZ> & vOdoms,
	                              const int & iShockNumThr,
                                 const int & iProcessFrame,
	                               float fDisDiffThr = 0.5,
	                                int iFirstTripThr = 10);

	bool NearGoal(const std::queue<pcl::PointXYZ> & vOdoms,
	                              const int & iShockNumThr,
                                 const int & iProcessFrame,
                               const pcl::PointXYZ & oGoal,
	                               float fDisDiffThr = 0.5,
	                                int iFirstTripThr = 10);

	//check the nodetime can be increased
	bool CheckNodeTimes();

    //check whether the grid is wide
	bool IsWideGrid(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                              const int & iQueryIdx);

	//get the current node index
	void GetNewNode(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                            const int & iNewNodeIdx,
			                          const pcl::PointXYZ & oNodePoints);
	void GetNewNode(const std::vector<ConfidenceValue> & vConfidenceMap,
	                              const std::vector<int> & vNewNodeIdxs,
			          const std::vector<pcl::PointXYZ> & vNewNodeClouds);

	//get the newly generated nodes
	void GetNewNodeSuppression(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                       const int & iNewNodeIdx,
			                                      const pcl::PointXYZ & oNodePoint,
					                                     float fSuppressionR = 1.0);

	void GetNewNodeSuppression(const std::vector<ConfidenceValue> & vConfidenceMap,
		                                     const std::vector<int> & vNewNodeIdxs,
			                     const std::vector<pcl::PointXYZ> & vNewNodeClouds,
					                                     float fSuppressionR = 1.0);

    //update the node value
	bool UpdateNodes(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                float fWideThr = 0.7,
	                                             float fNonWideThr = 0.8);

	//compute the Euclidean distance
	float TwoDDistance(const pcl::PointXYZ & oQueryPoint,
		               const pcl::PointXYZ & oTargetPoint);


	//the measured function of a pairs of nodes
    float ObjectiveFunction(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                    const Node & oQueryNode,
	                                                   const Node & oTargetNode);

    //greed method
    bool GTR(const pcl::PointXYZ & oCurOdom,
	         const std::vector<ConfidenceValue> & vConfidenceMap);

    //branch and bound method to solve op problem
    bool BranchBoundMethod(const pcl::PointXYZ & oCurOdom,
	                       const std::vector<ConfidenceValue> & vConfidenceMap);

    //local path
    bool LocalPathOptimization(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                       const std::vector<float> & vQualityFeature,
		                       const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud);

	//Outout the history of traveling nodes
    void OutputPastNodes(std::vector<pcl::PointXYZ> & vOutputNodes);

    void OutputPlanNodes(std::vector<pcl::PointXYZ> & vOutputNodes);

    void OutputUnvisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes);
    //get goal position
    void OutputGoalPos(pcl::PointXYZ & oGoalPoint);
    //get goal grid idx
    void OutputGoalPos(int & iGoalGridIdx);

    //some functions for test
    void PrintPlanNodes(const int & iQueryIdx,
	                    const std::vector<ConfidenceValue> & vConfidenceMap);  

    //all generated nodes
	std::vector<Node> m_vAllNodes;

private:

	//the grid index of which current robot is 
	//it also will became the target idx after using plan function
	//this value will be the core variable in path plan 
	int m_iCurrNodeIdx;//note that index is in m_vAllNodes

	//unvisited nodes or nodes to be visited (id)
    std::vector<int> m_vPlanNodeIdxs;//note that index is in m_vAllNodes

    //past nodes or nodes has been visited (id)
    std::vector<int> m_vPastNodeIdxs;//note that index is in m_vAllNodes

	//branch and bound based method's object
	BranchBound BBSolver;

};

}/*namespace*/

#endif









