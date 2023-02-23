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
OP::OP():m_iCurrNodeIdx(0),
               BBSolver(5){

    //5 in BBSolver(5) is a placeholder
    //it will be changed when using ObjectiveMatrix

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
bool OP::NearGoal(const std::queue<pcl::PointXYZ> & vOdoms,
	                              const int & iShockNumThr,
                                 const int & iProcessFrame,
	                                     float fDisDiffThr,
	                                     int iFirstTripThr){

	pcl::PointXYZ oCurrOdomTwoD;
	oCurrOdomTwoD.x = vOdoms.back().x;
	oCurrOdomTwoD.y = vOdoms.back().y;
	oCurrOdomTwoD.z = 0.0;

	pcl::PointXYZ oPastOdomTwoD;
	oPastOdomTwoD.x = vOdoms.front().x;
	oPastOdomTwoD.y = vOdoms.front().y;
	oPastOdomTwoD.z = 0.0;

    
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
		float fDisDiff = TwoDDistance(oCurrOdomTwoD, m_vAllNodes[m_iCurrNodeIdx].point);

		if(fDisDiff <= fDisDiffThr)
			return true;
        //check shock (shock is a situation in navigation pakcage)
        //shock may happen when robot is too close to the obstacle 
		float fDisShock = TwoDDistance(oCurrOdomTwoD, oPastOdomTwoD);

		if(vOdoms.size() >= iShockNumThr && fDisShock <= 0.5){
		    ROS_INFO("robot is standing at same place up to a given time, thereby a new goal is generated for it");	
			return true;
        }
	}

	return false;

}

//reload as a general input
bool OP::NearGoal(const std::queue<pcl::PointXYZ> & vOdoms,
	                              const int & iShockNumThr,
                                 const int & iProcessFrame,
                               const pcl::PointXYZ & oGoal,
	                                     float fDisDiffThr,
	                                     int iFirstTripThr){

	pcl::PointXYZ oCurrOdomTwoD;
	oCurrOdomTwoD.x = vOdoms.back().x;
	oCurrOdomTwoD.y = vOdoms.back().y;
	oCurrOdomTwoD.z = 0.0;

	pcl::PointXYZ oPastOdomTwoD;
	oPastOdomTwoD.x = vOdoms.front().x;
	oPastOdomTwoD.y = vOdoms.front().y;
	oPastOdomTwoD.z = 0.0;

    
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
		float fDisDiff = TwoDDistance(oCurrOdomTwoD, oGoal);

		if(fDisDiff <= fDisDiffThr)
			return true;
        //check shock (shock is a situation in navigation pakcage)
        //shock may happen when robot is too close to the obstacle 
		float fDisShock = TwoDDistance(oCurrOdomTwoD, oPastOdomTwoD);

		if(vOdoms.size() >= iShockNumThr && fDisShock <= 0.5){
		    ROS_INFO("robot is standing at same place up to a given time, thereby a new goal is generated for it");	
			return true;
        }
	}

	return false;

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
bool OP::CheckNodeTimes(){

    //if only the original node
    //thereby stay at the original place for a more completed scanning
	if(m_vAllNodes.size() > 1)
		return true;//increase
	else
		return false;//dont increase

}


/*************************************************
Function: IsWideGrid
Description: check whether grid is a wide grid or not
Calls: none
Called By: main function
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the feature map
       iQueryIdx - the query grid index
Output: a flag indicats it is a wide grid (true) or not (false)
Return: a bool value
Others: none
*************************************************/
bool OP::IsWideGrid(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                              const int & iQueryIdx) {

    //if this grid is not close to boundary and also visiable to current robot
	if (vConfidenceMap[iQueryIdx].boundTerm < 0.1 ||
		vConfidenceMap[iQueryIdx].visiTerm.value == 1.0)
		return true;//true indicates it is a wide grid

    //only the node which boundTerm >= 0.3 and visiTerm.value < 0.9 will be recorded as non wide node
	return false;//false indicates it is NOT a wide grid

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
void OP::GetNewNode(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                            const int & iNewNodeIdx,
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

    //wide or not
    if(oNewNode.parentIdx != 0)
    	oNewNode.wideFlag = IsWideGrid(vConfidenceMap, iNewNodeIdx);
    else
    	oNewNode.wideFlag = false;

    //get the new node
	m_vAllNodes.push_back(oNewNode);
    
    //need update path strategy since the new input comes
	m_vPlanNodeIdxs.clear();

}
//reload with multiple inputs
void OP::GetNewNode(const std::vector<ConfidenceValue> & vConfidenceMap,
	                              const std::vector<int> & vNewNodeIdxs,
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
		//wide or not
		if(oNewNode.parentIdx != 0)
			oNewNode.wideFlag = IsWideGrid(vConfidenceMap, vNewNodeIdxs[i]);
		else
			oNewNode.wideFlag = false;
		//get the grid idx of new node
		m_vAllNodes.push_back(oNewNode);

	}//end for int i = 0; i != vNewNodeIdxs.size(); ++i)

    //need update path strategy since the new input comes
	m_vPlanNodeIdxs.clear();

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
void OP::GetNewNodeSuppression(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                       const int & iNewNodeIdx,
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
        //wide or not
        if(oNewNode.parentIdx != 0)
        	oNewNode.wideFlag = IsWideGrid(vConfidenceMap, iNewNodeIdx);
        else
        	oNewNode.wideFlag = false;
        //get the new node
	    m_vAllNodes.push_back(oNewNode);

	    //need update path strategy since the new input comes
	    m_vPlanNodeIdxs.clear();

	}

}
//reload with multiple inputs
void OP::GetNewNodeSuppression(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                         const std::vector<int> & vNewNodeIdxs,
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
			    //wide or not
			    if(oNewNode.parentIdx != 0)
			    	oNewNode.wideFlag = IsWideGrid(vConfidenceMap, vNewNodeIdxs[i]);
			    else
			    	oNewNode.wideFlag = false;
	            //get the grid idx of new node
			    m_vAllNodes.push_back(oNewNode);
			    
			    //need update path strategy since the new input comes
	            m_vPlanNodeIdxs.clear();

			}//end if

		}//end for int i = 0; i != vNewNodeIdxs.size(); ++i)

	}else{//there are not unvisited nodes

	    //directly save input nodes
        GetNewNode(vConfidenceMap, vNewNodeIdxs, vNewNodeClouds);

        //need update path strategy since the new input comes
	    m_vPlanNodeIdxs.clear();

	}//end else

}

/*************************************************
Function: UpdateNodes
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
bool OP::UpdateNodes(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                      float fWideThr,
	                                                   float fNonWideThr){

	//check the input is correct
	if(fWideThr > fNonWideThr)
	   fWideThr = fNonWideThr;//wide threshold has better larger than the one

    //count how many nodes need to be removed from plan
    int iRemoveNum = 0;
    	//wide node number
	int iWideNodeNum = 0;

    std::cout<<"updating node totalvalue: "<<std::endl;
	//a status indicating whether the node is removed or not
	for (int i = 0; i != m_vAllNodes.size(); ++i) {
		//search all unvisited wide node
		if(!m_vAllNodes[i].visitedFlag){
            //if it not need to go
            
            PrintPlanNodes(m_vAllNodes[i].gridIdx, vConfidenceMap);
            std::cout<<std::endl;

            float fWithDrawThr;
            //if it is wide node
            if(m_vAllNodes[i].wideFlag){
            	fWithDrawThr = fWideThr;//use the wide threshold
            }else{
            	fWithDrawThr = fNonWideThr;//use the non wide threshold
            }

            //give a large threshold to original node
            if(m_vAllNodes[i].parentIdx == 0)
            	fWithDrawThr = 0.9;//because it will generated close to original robot at initialization

            //if it is up to a value and also not a near node from original node (original node can only generate near node)
			if(vConfidenceMap[m_vAllNodes[i].gridIdx].travelTerm > fWithDrawThr){
			   m_vAllNodes[i].visitedFlag = true;
			   iRemoveNum++;
			   continue;
			}//end if >

            //original nodes are structure nodes forever
            if(m_vAllNodes[i].parentIdx == 0)
            	continue;

			//also update the wide characteristic of nodes
			m_vAllNodes[i].wideFlag = IsWideGrid(vConfidenceMap, m_vAllNodes[i].gridIdx);
			//count if it is a wide node
			if(m_vAllNodes[i].wideFlag)
				iWideNodeNum++;

		}//end if visitedFlag

	}//end for

    //need update path strategy since some nodes removed
    if(iRemoveNum)
    	m_vPlanNodeIdxs.clear();

	//if there are still some wide areas
	std::cout << "****remove node number: " << iRemoveNum << std::endl;
	std::cout << "****remain wide node number: " << iWideNodeNum << std::endl;
	if (iWideNodeNum)
		return true;
	else
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
inline float OP::ObjectiveFunction(const std::vector<ConfidenceValue> & vConfidenceMap,
	                                                           const Node & oQueryNode,
	                                                          const Node & oTargetNode){

	//using measured method

	//compute the final cost value
	float fCost = TwoDDistance(oQueryNode.point, oTargetNode.point);

	//compute the reward value
	float fReward = 1.0 - vConfidenceMap[oTargetNode.gridIdx].totalValue;
	
	//initial the objective value as inf limit
	float fObjectValue = FLT_MAX;

	//Constrained magnification
	//because some grids are too late to calculate feature value before generating node
	//also prevent division by zero
	if (fReward < 0.5)
		fReward = 0.5;

    //if reward is in normal
	//objective = cost / reward, thereby to find the minimum cost
	fObjectValue = fCost / fReward;

	//check the target grid is a wide grid
	//if it is a wide grid, then targetedly decreases the cost 
	if (IsWideGrid(vConfidenceMap, oTargetNode.gridIdx))
		fObjectValue = fObjectValue;

	//return result
	return fObjectValue;

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
    m_vPastNodeIdxs.push_back(m_iCurrNodeIdx);

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
    float fMinMeasure = FLT_MAX;
	//to each unselected point
	for (int i = 0; i != vPlanNodeIdxs.size(); ++i) {

		int iTargetIdx = vPlanNodeIdxs[i];

		//cost function which considers the reward and cost of node
		float fPairMeasure = ObjectiveFunction(vConfidenceMap,
			                                   m_vAllNodes[m_iCurrNodeIdx], 
			                                   m_vAllNodes[iTargetIdx]);
			   
		//find the shorest route
		if (fPairMeasure < fMinMeasure) {
			fMinMeasure = fPairMeasure;
			iMinIdx = iTargetIdx;
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
bool OP::BranchBoundMethod(const pcl::PointXYZ & oCurOdom,
	                       const std::vector<ConfidenceValue> & vConfidenceMap) {

    //whatever the robot successfully reached target node 
	m_vAllNodes[m_iCurrNodeIdx].point.x = oCurOdom.x;
	m_vAllNodes[m_iCurrNodeIdx].point.y = oCurOdom.y;
	//m_vAllNodes[m_iCurrNodeIdx].point.z = 0.0;
	m_vAllNodes[m_iCurrNodeIdx].visitedFlag = true;
	//record at node trajectory
    m_vPastNodeIdxs.push_back(m_iCurrNodeIdx);

	//check whether a plan should be build up
	//size = 0 means need new plan
	//size = 1 means the travel is over
	//size > 1 means continue raw plan

	if(m_vPlanNodeIdxs.size()){

	    //implement the raw plan
		if(m_vPlanNodeIdxs.size() > 1){
			std::cout<<"use raw plan: "<<std::endl;
			//NEW plan
			std::vector<int> vNewPlanIdxs;
			for(int i = 1; i < m_vPlanNodeIdxs.size(); ++i)//start from 1
				vNewPlanIdxs.push_back(m_vPlanNodeIdxs[i]);
            //reset raw plan
            m_vPlanNodeIdxs.clear();
            for(int i = 0; i!= vNewPlanIdxs.size(); ++i){//start from 0
				m_vPlanNodeIdxs.push_back(vNewPlanIdxs[i]);
				PrintPlanNodes(m_vAllNodes[vNewPlanIdxs[i]].gridIdx, vConfidenceMap);
				std::cout<<" -> ";
			}
            //get current target grid index
		    m_iCurrNodeIdx = m_vPlanNodeIdxs[0];
            //ouput messages that some nodes are still not visited
		    return false;
	    }//end if m_vPlanNodeIdxs.size() > 1

		//the current grid index stand alone
		if(m_vPlanNodeIdxs.size() == 1){
			//remove current
			m_vPlanNodeIdxs.pop_back();
			std::cout<<"over from raw plan of bb"<<std::endl;
		    //all nodes are visited
		    return true; 
		}

	}

   	//check which points are need to be planed
	std::vector<int> vPlanNodeIdxs;
	vPlanNodeIdxs.push_back(m_iCurrNodeIdx);
	for (int i = 0; i != m_vAllNodes.size();++i) {
		if (!m_vAllNodes[i].visitedFlag)
			vPlanNodeIdxs.push_back(i);
    }

	//still check again if some nodes has been removed in updating map
	if (vPlanNodeIdxs.size() < 2){
		return true;
    }

    //construct a directed graph for remained nodes
	//effective matrix
	std::vector<std::vector<float>> vEffectMatrix;
	std::vector<float> vArray(vPlanNodeIdxs.size(), 0.0);
	for (int i = 0; i != vPlanNodeIdxs.size(); ++i) {
		vEffectMatrix.push_back(vArray);
	}
	
	//construct a measured matrix among unvisited nodes
	for (int i = 0; i != vPlanNodeIdxs.size(); ++i) {

		int iSourceIdx = vPlanNodeIdxs[i];

		for(int j = 0; j != vPlanNodeIdxs.size(); ++j) {
			
			//compute the real cost using the given cost function
			int iTargetIdx =vPlanNodeIdxs[j];

			//cost function which considers the reward and cost of node
			vEffectMatrix[i][j] = ObjectiveFunction(vConfidenceMap,
			                                        m_vAllNodes[iSourceIdx], 
			                                        m_vAllNodes[iTargetIdx]);

		}//end if(vUnVisitedStatus[i])
	}

    //input the matrix to bb object
	BBSolver.ObjectiveMatrix(vEffectMatrix);
	//get the plan result
	std::vector<int> vResTour;
	//output without the frist element, the frist one of output is the goal(next best node)
	//use the non-closed type
	float fBestEffective = BBSolver.SolveOP(vResTour);  
	std::cout << "new plan! and the best effective is " << fBestEffective << std::endl;
	
	for (int i = 1; i != vResTour.size(); ++i) {//start from 1
	
		int iBBNodeidx = vResTour[i];//note that bb node is beginning from 1 
		//find the real index in unvisited nodes
		int iNodeIdx = vPlanNodeIdxs[iBBNodeidx];
		//fresh the plan
	    m_vPlanNodeIdxs.push_back(iNodeIdx);
        //print the plan
		PrintPlanNodes(m_vAllNodes[iNodeIdx].gridIdx, vConfidenceMap);
		std::cout<<" -> ";
	}
	std::cout << std::endl;

    //get the next best viewpoint
	m_iCurrNodeIdx = m_vPlanNodeIdxs[0];

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
bool OP::LocalPathOptimization(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                       const std::vector<float> & vQualityFeature,
		                       const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud){



	
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
	vOutputNodes.reserve(m_vPastNodeIdxs.size());

	//assignment
	for (int i = 0; i != m_vPastNodeIdxs.size(); ++i){
		vOutputNodes.push_back(m_vAllNodes[m_vPastNodeIdxs[i]].point);
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
	vOutputNodes.reserve(m_vPlanNodeIdxs.size());

	//assignment
	for (int i = 0; i != m_vPlanNodeIdxs.size(); ++i){
		vOutputNodes.push_back(m_vAllNodes[m_vPlanNodeIdxs[i]].point);
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
void OP::PrintPlanNodes(const int & iQueryIdx,
	                    const std::vector<ConfidenceValue> & vConfidenceMap){


	std::cout << " node id is " << iQueryIdx
	          << " travel " << vConfidenceMap[iQueryIdx].travelTerm
	          << " bound " << vConfidenceMap[iQueryIdx].boundTerm 
	          << " visible " << vConfidenceMap[iQueryIdx].visiTerm.value 
	          << " totalvalue " << vConfidenceMap[iQueryIdx].totalValue;

}




}/*namespace*/














