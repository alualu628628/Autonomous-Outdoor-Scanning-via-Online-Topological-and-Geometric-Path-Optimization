#include "BranchBound.h"

namespace topology_map {



BranchBound::BranchBound(const int & f_iNodeNum){

	//int 
	iNodeNum = f_iNodeNum;
	int iArraySize = iNodeNum + 1;
	//clear old data
	vEffective.clear();
	//new the vEffective
	std::vector<float> vArray(iArraySize,-1.0);
	for (int i = 0; i != iArraySize; ++i) {
		vEffective.push_back(vArray);
	}

}



BranchBound::~BranchBound(){
}





//compute distance in two dimension
float BranchBound::TwoDDistance(const pcl::PointXYZ & oPointOne,
	const pcl::PointXYZ & oPointTwo) {
	//compute 2d distance
	float fDis = sqrt(pow(oPointOne.x - oPointTwo.x, 2.0) +
		pow(oPointOne.y - oPointTwo.y, 2.0));
	//ouput
	return fDis;
}




//initial the objective value between each node pairs
void BranchBound::SimpleInitial(const pcl::PointCloud<pcl::PointXYZ> & vNodeCloud){

	//the first element is empty
	//beginning node is at the second element of vResTour
	for (int i = 0; i != vNodeCloud.size(); ++i) {
		for (int j = 0; j != vNodeCloud.size(); ++j) {
			if (i != j)
				vEffective[i + 1][j + 1] = TwoDDistance(vNodeCloud[i], vNodeCloud[j]);
			else
				vEffective[i + 1][j + 1] = -1.0;
		}//end i
	}//end j

}


//get objective value between each two nodes 
void BranchBound::ObjectiveMatrix(const std::vector<std::vector<float>> & vObjectMatrix){

	vEffective.clear();
	iNodeNum = vObjectMatrix.size();
	//refresh the vEffective based on the input matrix size
	int iArraySize = iNodeNum + 1;
	std::vector<float> vArray(iArraySize, -1.0);
	for (int i = 0; i != iArraySize; ++i) {
		vEffective.push_back(vArray);
	}

	//the first element is empty
	//beginning node is at the second element of vResTour
	for (int i = 0; i != vObjectMatrix.size(); ++i) {
		for (int j = 0; j != vObjectMatrix[i].size(); ++j) {
			if (i != j)
				vEffective[i + 1][j + 1] = vObjectMatrix[i][j];
		}//end i
	}//end j

}


//the first element is empty
//beginning node is at the second element of vResTour
float BranchBound::SolveHamiltonianOP(std::vector<int> & vResTour){

	vResTour.clear();

 //bestTour[1:n]´

	int *bestTour = new int[iNodeNum + 1];
	
	minHeap<heapNode> liveNodeMinHeap;

	//cost
	float *minCostOutEdge = new float[iNodeNum + 1];
	
	float sumOfMinCostOutEdges = 0;
	for (int i = 1; i <= iNodeNum; ++i)
	{
		float minCost = -1.0;
		for (int j = 1; j <= iNodeNum; ++j)
			if (vEffective[i][j] != -1 && (minCost == -1.0 || minCost > vEffective[i][j]))
				minCost = vEffective[i][j];
		if (minCost == -1.0)
			return -1.0;
		minCostOutEdge[i] = minCost;
		sumOfMinCostOutEdges += minCost;
	}

	//
	heapNode eNode;
	eNode.lowerCost = sumOfMinCostOutEdges;
	eNode.currentCost = 0;
	eNode.restMinCost = sumOfMinCostOutEdges;
	eNode.s = 0;
	eNode.currentTour = new int[iNodeNum];
	//[1,2,3,...,n,1]
	for (int i = 0; i < iNodeNum; ++i)
		eNode.currentTour[i] = i + 1;

	float bestCostSoFar = -1.0;  //
	int *currentTour = eNode.currentTour;

	//
	while (eNode.s < iNodeNum - 1)
	{
		currentTour = eNode.currentTour;
		if (eNode.s == iNodeNum - 2)
		{//
			if (vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]] != -1.0 &&
				vEffective[currentTour[iNodeNum - 1]][1] != -1.0 &&
				(bestCostSoFar == -1.0 || eNode.currentCost +
					vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]] +
					vEffective[currentTour[iNodeNum - 1]][1] < bestCostSoFar))
			{//
				bestCostSoFar = eNode.currentCost +
					vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]] +
					vEffective[currentTour[iNodeNum - 1]][1];
				eNode.currentCost = bestCostSoFar;
				eNode.lowerCost = bestCostSoFar;
				eNode.s++;
				liveNodeMinHeap.push(eNode);
			}
			else
			{
				delete[] eNode.currentTour;  //
				eNode.currentTour = nullptr;
			}
		}

		else
		{//
			for (int i = eNode.s + 1; i < iNodeNum; ++i)
				if (vEffective[currentTour[eNode.s]][currentTour[i]] != -1.0)
				{//
					float currentCost = eNode.currentCost +
						vEffective[currentTour[eNode.s]][currentTour[i]];
					float restMinCost = eNode.restMinCost -
						minCostOutEdge[currentTour[eNode.s]];
					float leastCostPossible = currentCost + restMinCost;

					if (bestCostSoFar == -1.0 ||
						leastCostPossible < bestCostSoFar)
					{//
						heapNode hNode;
						hNode.lowerCost = leastCostPossible;
						hNode.currentCost = currentCost;
						hNode.restMinCost = restMinCost;
						hNode.s = eNode.s + 1;
						hNode.currentTour = new int[iNodeNum];
						std::copy(currentTour, currentTour + iNodeNum, hNode.currentTour);

						std::swap(hNode.currentTour[hNode.s], hNode.currentTour[i]);
						liveNodeMinHeap.push(hNode);
					}
				}

			//
			delete[] eNode.currentTour;
			eNode.currentTour = nullptr;
		}

		if (liveNodeMinHeap.empty())
			break;

		//
		eNode = liveNodeMinHeap.top();
		liveNodeMinHeap.pop();
	}

	if (bestCostSoFar == -1.0)
		return -1.0;

	//
	std::copy(eNode.currentTour, eNode.currentTour + iNodeNum, bestTour + 1);

	//
	//
	while (true)
	{
		delete[] eNode.currentTour;
		if (liveNodeMinHeap.empty())
			break;
		//È¡ÏÂÒ»¸öE-½Úµã
		eNode = liveNodeMinHeap.top();
		liveNodeMinHeap.pop();
	}

	//set as a vector starting from 0
	//each element need to mins 1
	//output without the frist element
	for (int i = 1; i < iNodeNum + 1; ++i)
		vResTour.push_back(bestTour[i] - 1);

	delete[] bestTour;
	delete[] minCostOutEdge;

	return bestCostSoFar;
}


//in this function below:
//the problem need not to sontruct a Hamiltonian graph
//this means that the travel not need to go back to the orginal node
//beginning node is at the second element of vResTour
float BranchBound::SolveOP(std::vector<int> & vResTour){

	vResTour.clear();

 //bestTour[1:n]´

	int *bestTour = new int[iNodeNum + 1];
	
	minHeap<heapNode> liveNodeMinHeap;

	//cost
	float *minCostOutEdge = new float[iNodeNum + 1];
	
	float sumOfMinCostOutEdges = 0;
	for (int i = 1; i <= iNodeNum; ++i)
	{
		float minCost = -1.0;
		for (int j = 1; j <= iNodeNum; ++j)
			if (vEffective[i][j] != -1 && (minCost == -1.0 || minCost > vEffective[i][j]))
				minCost = vEffective[i][j];
		if (minCost == -1.0)
			return -1.0;
		minCostOutEdge[i] = minCost;
		sumOfMinCostOutEdges += minCost;
	}

	//
	heapNode eNode;
	eNode.lowerCost = sumOfMinCostOutEdges;
	eNode.currentCost = 0;
	eNode.restMinCost = sumOfMinCostOutEdges;
	eNode.s = 0;
	eNode.currentTour = new int[iNodeNum];
	//[1,2,3,...,n,1]
	for (int i = 0; i < iNodeNum; ++i)
		eNode.currentTour[i] = i + 1;

	float bestCostSoFar = -1.0;  //
	int *currentTour = eNode.currentTour;

	//
	while (eNode.s < iNodeNum - 1)
	{

		currentTour = eNode.currentTour;

		if (eNode.s == iNodeNum - 2){
		//check
			if (vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]] != -1.0 &&
				vEffective[currentTour[iNodeNum - 1]][1] != -1.0 &&
				(bestCostSoFar == -1.0 || eNode.currentCost + vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]] < bestCostSoFar)){
				//
				bestCostSoFar = eNode.currentCost + vEffective[currentTour[iNodeNum - 2]][currentTour[iNodeNum - 1]];

				eNode.currentCost = bestCostSoFar;
				eNode.lowerCost = bestCostSoFar;
				eNode.s++;
				liveNodeMinHeap.push(eNode);
			}
			else
			{
				delete[] eNode.currentTour;  //
				eNode.currentTour = nullptr;
			}
		}

		else
		{//
			for (int i = eNode.s + 1; i < iNodeNum; ++i)
				if (vEffective[currentTour[eNode.s]][currentTour[i]] != -1.0)
				{//
					float currentCost = eNode.currentCost +
						vEffective[currentTour[eNode.s]][currentTour[i]];
					float restMinCost = eNode.restMinCost -
						minCostOutEdge[currentTour[eNode.s]];
					float leastCostPossible = currentCost + restMinCost;

					if (bestCostSoFar == -1.0 ||
						leastCostPossible < bestCostSoFar)
					{//
						heapNode hNode;
						hNode.lowerCost = leastCostPossible;
						hNode.currentCost = currentCost;
						hNode.restMinCost = restMinCost;
						hNode.s = eNode.s + 1;
						hNode.currentTour = new int[iNodeNum];
						std::copy(currentTour, currentTour + iNodeNum, hNode.currentTour);

						std::swap(hNode.currentTour[hNode.s], hNode.currentTour[i]);
						liveNodeMinHeap.push(hNode);
					}
				}

			//
			delete[] eNode.currentTour;
			eNode.currentTour = nullptr;
		}

		if (liveNodeMinHeap.empty())
			break;

		//
		eNode = liveNodeMinHeap.top();
		liveNodeMinHeap.pop();
	}

	if (bestCostSoFar == -1.0)
		return -1.0;

	//
	std::copy(eNode.currentTour, eNode.currentTour + iNodeNum, bestTour + 1);

	//
	//
	while (true)
	{
		delete[] eNode.currentTour;
		if (liveNodeMinHeap.empty())
			break;
		//È¡ÏÂÒ»¸öE-½Úµã
		eNode = liveNodeMinHeap.top();
		liveNodeMinHeap.pop();
	}

	//set as a vector starting from 0
	//each element need to mins 1
	//output without the frist element
	for (int i = 1; i < iNodeNum + 1; ++i)
		vResTour.push_back(bestTour[i] - 1);

	delete[] bestTour;
	delete[] minCostOutEdge;

	return bestCostSoFar;
}

}/*namespace*/

