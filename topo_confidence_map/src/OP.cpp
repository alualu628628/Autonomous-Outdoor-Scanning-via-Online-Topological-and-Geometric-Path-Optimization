#include "OP.h"

namespace topology_map {
/*************************************************
Function: OP
Description: construction of OP class
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
OP::OP():m_iCurrNodeIdx(0){



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
OP::~OP(){


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
void OP::Initial(const pcl::PointXYZ & oOriginPoint,
	             const grid_map::GridMap & oFeatureMap){

	m_vAllNodes.clear();
	//a new node
	Node oNewNode;

	oNewNode.point.x = oOriginPoint.x;
	oNewNode.point.y = oOriginPoint.y;
	//oNewNode.point.z = vNodePoints.point.z;
    oNewNode.point.z = 0.0;//

    int iFirstNodeIdx;
	//the corresponding grid idx of node
    oNewNode.gridIdx = 	ExtendedGM::PointoOneDIdx(oOriginPoint,
	                                              oFeatureMap);

    //the parent id
    oNewNode.parentIdx = 0;//root is itself
    //visited or not
    oNewNode.visitedFlag = false;

    //get the new node
	m_vAllNodes.push_back(oNewNode);

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
bool OP::NearGoal(const pcl::PointXYZ & oOdomPoint,
                         const int & iProcessFrame,
	                             float fDisDiffThr,
	                             int iFirstTripThr){

	pcl::PointXYZ oTwoDPoint;
	oTwoDPoint.x = oOdomPoint.x;
	oTwoDPoint.y = oOdomPoint.y;
	oTwoDPoint.z = 0.0;
    
    //if it is at the orginal point
	if(m_vAllNodes.size() == 1){
        //if the confidence map at original location has been computed enough
        int iRemain = iFirstTripThr - iProcessFrame;
        ROS_INFO("Initial the first goal: remain confidence computed time is: [%d].", iRemain);
        //"<=" rather than "=" is to defend sampling error
		if(iRemain <= 0)
			return true;

	}
    
    //normal situation
	if(m_vAllNodes.size() > 1){
		//compute the distance difference between current odom and current target node
		float fDisDiff = TwoDDistance(oTwoDPoint, m_vAllNodes[m_iCurrNodeIdx].point);

		if(fDisDiff <= fDisDiffThr)
			return true;

	}

	return false;

}


/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
void OP::GetNewNode(const int & iNewNodeIdx,
			        const pcl::PointXYZ & oNodePoints){
	
	//a new node
	Node oNewNode;

	oNewNode.point.x = oNodePoints.x;
	oNewNode.point.y = oNodePoints.y;
	//oNewNode.point.z = vNodePoints.point.z;
    oNewNode.point.z = 0.0;//

	//the corresponding grid idx of node
    oNewNode.gridIdx = iNewNodeIdx;
    //the parent id

    oNewNode.parentIdx = m_iCurrNodeIdx;
    //visited or not
    oNewNode.visitedFlag = false;

    //get the new node
	m_vAllNodes.push_back(oNewNode);

}
//reload with multiple inputs
void OP::GetNewNode(const std::vector<int> & vNewNodeIdxs,
			        const std::vector<pcl::PointXYZ> & vNewNodeClouds){

	if(vNewNodeIdxs.size() != vNewNodeClouds.size()){
		ROS_INFO("Error: node index size and node point size are not the same.");
		return ;
	}

    //to each new node
	for (int i = 0; i != vNewNodeIdxs.size(); ++i) {

	    //a new node
	    Node oNewNode;

	    oNewNode.point.x = vNewNodeClouds[i].x;
	    oNewNode.point.y = vNewNodeClouds[i].y;
	    //oNewNode.point.z = vNewNodeClouds[i].z;
	    oNewNode.point.z = 0.0;

	    //the corresponding grid idx of node
	    oNewNode.gridIdx = vNewNodeIdxs[i];
	    //the parent id
	    oNewNode.parentIdx = m_iCurrNodeIdx;
	    //visited or not
	    oNewNode.visitedFlag = false;
		
		//get the grid idx of new node
		m_vAllNodes.push_back(oNewNode);

	}//end for int i = 0; i != vNewNodeIdxs.size(); ++i)

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
void OP::GetNewNodeSuppression(const int & iNewNodeIdx,
			                   const pcl::PointXYZ & oNodePoint,
					           float fSuppressionR){

    //define a rule
	float fMinDis = FLT_MAX;

	//find wether the input new node is near a unvisited node
	for(int i = 0; i != m_vAllNodes.size(); ++i){

		if(!m_vAllNodes[i].visitedFlag){
		//compute the distance
	    float fDis = TwoDDistance(oNodePoint, m_vAllNodes[i].point);

		//record the shortest distance
		if(fDis < fMinDis)
		   fMinDis = fDis;
        }//end if

	}//end for

	//if this grid is too close to a generated node
	if(fMinDis > fSuppressionR){
       
       	//a new node
	    Node oNewNode;

	    oNewNode.point.x = oNodePoint.x;
	    oNewNode.point.y = oNodePoint.y;
	    //oNewNode.point.z = vNodePoints.point.z;
        oNewNode.point.z = 0.0;//

	    //the corresponding grid idx of node
        oNewNode.gridIdx = iNewNodeIdx;
        //the parent id
        oNewNode.parentIdx = m_iCurrNodeIdx;
        //visited or not
        oNewNode.visitedFlag = false;

        //get the new node
	    m_vAllNodes.push_back(oNewNode);

	}

}
//reload with multiple inputs
void OP::GetNewNodeSuppression(const std::vector<int> & vNewNodeIdxs,
			                   const std::vector<pcl::PointXYZ> & vNewNodeClouds,
					           float fSuppressionR){

	//new a point cloud to save thr unvisited point
    pcl::PointCloud<pcl::PointXYZ>::Ptr pUnVisitCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i != m_vAllNodes.size(); ++i){
	    if(!m_vAllNodes[i].visitedFlag)
	    	pUnVisitCloud->points.push_back(m_vAllNodes[i].point);
	}


    //if there is the unvisited node
    if(pUnVisitCloud->points.size()){

	    //new a kdtree for unvisited point clouds
	    pcl::KdTreeFLANN<pcl::PointXYZ> oUnVisitKDTree;
	    oUnVisitKDTree.setInputCloud(pUnVisitCloud);

	    //to each new node
	    for (int i = 0; i != vNewNodeClouds.size(); ++i) {
			
		    //set the searched vector
		    std::vector<int> vSearchedIdx;
		    std::vector<float> vSearchedDis;
		    //find the nearest unvisited node
		    oUnVisitKDTree.nearestKSearch(vNewNodeClouds[i], 1, vSearchedIdx, vSearchedDis);

		    float fRealDis = sqrt(vSearchedDis[0]);

			//if they are far away (the distance farther than threshold)
			if(fRealDis > fSuppressionR){

				//a new node
	            Node oNewNode;

	            oNewNode.point.x = vNewNodeClouds[i].x;
	            oNewNode.point.y = vNewNodeClouds[i].y;
	            //oNewNode.point.z = vNodePoints.point.z;
                oNewNode.point.z = 0.0;//

	            //the corresponding grid idx of node
                oNewNode.gridIdx = vNewNodeIdxs[i];
                //the parent id
                oNewNode.parentIdx = m_iCurrNodeIdx;
                //visited or not
                oNewNode.visitedFlag = false;
			   
	            //get the grid idx of new node
			    m_vAllNodes.push_back(oNewNode);
			    //m_vPlanNodeIdx.push_back(m_vAllNodes.size()-1);

			}//end if

		}//end for int i = 0; i != vNewNodeIdxs.size(); ++i)

	}else{//there are not unvisited nodes

	    //directly save input nodes
        GetNewNode(vNewNodeIdxs,vNewNodeClouds);

	}//end else

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
float OP::TwoDDistance(const pcl::PointXYZ & oQueryPoint,
	                   const pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		      + pow(oQueryPoint.y - oTargetPoint.y, 2.0f));

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
inline float OP::CostFunction(const Node & oQueryNode,
	                          const Node & oTargetNode){

	//define 
	float fCostValue = FLT_MAX;

	//compute the final cost based on the cost and reward
	fCostValue = TwoDDistance(oQueryNode.point, oTargetNode.point);

	//
	return fCostValue;

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
bool OP::GTR(const pcl::PointXYZ & oCurOdom,
	         const std::vector<ConfidenceValue> & vConfidenceMap) {

	//whatever the robot successfully reached target node 
	m_vAllNodes[m_iCurrNodeIdx].point.x = oCurOdom.x;
	m_vAllNodes[m_iCurrNodeIdx].point.y = oCurOdom.y;
	//m_vAllNodes[m_iCurrNodeIdx].point.z = 0.0;
	m_vAllNodes[m_iCurrNodeIdx].visitedFlag = true;
	//record at node trajectory
    m_vPastNodeIdx.push_back(m_iCurrNodeIdx);

	//check whether all are completed
	std::vector<int> vPlanNodeIdxs;
	for (int i = 0; i != m_vAllNodes.size();++i) {
		if (!m_vAllNodes[i].visitedFlag)
			vPlanNodeIdxs.push_back(i);
    }

	//define output
	//if there are not any new nodes shoule be invoked
	if (!vPlanNodeIdxs.size()){
		return true;
    }


    int iMinIdx;
    float fMinCost = FLT_MAX;
	//to each unselected point
	for (int i = 0; i != vPlanNodeIdxs.size(); ++i) {

		int iQueryIdx = vPlanNodeIdxs[i];

		//cost function which considers the reward and cost of node
		float fPairCost = CostFunction(m_vAllNodes[m_iCurrNodeIdx], m_vAllNodes[iQueryIdx]);
			   
		//find the shorest route
		if (fPairCost < fMinCost) {
			fMinCost = fPairCost;
			iMinIdx = iQueryIdx;
		}

	}//end if(vUnVisitedStatus[i])

	m_iCurrNodeIdx = iMinIdx;

	return false;

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
void OP::OutputPastNodes(std::vector<pcl::PointXYZ> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.reserve(m_vPastNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vPastNodeIdx.size(); ++i){
		vOutputNodes.push_back(m_vAllNodes[m_vPastNodeIdx[i]].point);
	}

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
void OP::OutputPlanNodes(std::vector<pcl::PointXYZ> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.reserve(m_vPlanNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vPlanNodeIdx.size(); ++i){
		vOutputNodes.push_back(m_vAllNodes[m_vPlanNodeIdx[i]].point);
	}

}

/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
void OP::OutputUnvisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes){

	vOutputNodes.clear();

	//assignment
	for (int i = 0; i != m_vAllNodes.size(); ++i){
		if(!m_vAllNodes[i].visitedFlag)
			vOutputNodes.push_back(m_vAllNodes[i].point);
	}

}
/*************************************************
Function: TwoDDistance
Description: compute distance
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: oQueryPoint - one 3d point
       oTargetPoint - another 3d point
Output: a Euclidean distance
Return: float distance
Others: none
*************************************************/
//output the goal point
void OP::OutputGoalPos(pcl::PointXYZ & oGoalPoint){

	oGoalPoint.x = m_vAllNodes[m_iCurrNodeIdx].point.x;
	oGoalPoint.y = m_vAllNodes[m_iCurrNodeIdx].point.y;
	oGoalPoint.z = m_vAllNodes[m_iCurrNodeIdx].point.z;

}
//reload - return the grid id of target grid
void OP::OutputGoalPos(int & iGoalGridIdx){

	iGoalGridIdx = m_iCurrNodeIdx;

}


}/*namespace*/