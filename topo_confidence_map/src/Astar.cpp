#include "Astar.h"  

namespace topology_map {

void Astar::InitAstarTravelMap(const grid_map::GridMap & oExtendGridMap){

	//clear raw data
	maze.clear(); 

	//resize the maze
    maze.resize(oExtendGridMap.getSize()(0));

	//note that 1 value is a obstacle grid label in astar algorithm
	for (int i = 0; i != oExtendGridMap.getSize()(0); ++i) {

		for(int j = 0; j != oExtendGridMap.getSize()(1); ++j)
		    maze[i].push_back(1);//all obstacles at the beginning

	}

}




//update map which record the travelable grids
void Astar::UpdateTravelMap(const grid_map::GridMap & oExtendGridMap,
	                        const std::vector<ConfidenceValue> & vConfidenceMap){

    //i
    for (int i = 0; i != oExtendGridMap.getSize()(0); ++i) {//i
        //j
		for (int j = 0; j != oExtendGridMap.getSize()(1); ++j) {//j
            //get index
			int iGridIdx = ExtendedGM::TwotoOneDIdx(i, j);

            //record the travelable region
            if(vConfidenceMap[iGridIdx].label == 1 ||vConfidenceMap[iGridIdx].label == 3 )
                maze[i][j] = 1;//note that 1 value is a obstacle grid label in astar algorithm
			else
				maze[i][j] = 0;//note that 0 value is a travelable grid label in astar algorithm
		}//end j

    }//end i

}




int Astar::calcG(AstarPoint *temp_start, AstarPoint *point)
{
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	//If it is the initial node, its parent node is empty  
	int parentG = point->parent == NULL ? 0 : point->parent->G; 

	return parentG + extraG;
}



int Astar::calcH(AstarPoint *point, AstarPoint *end)
{
	//Calculating H with a simple Euclidean distance
	//Euclidean distance can be instead of other measured algorithms  
	return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;
}



int Astar::calcF(AstarPoint *point)
{
	return point->G + point->H;
}




AstarPoint *Astar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
		if (point->F<resPoint->F)
			resPoint = point;
		return resPoint;
	}
	return NULL;
}





AstarPoint *Astar::findPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner)
{
	//Put in the starting point, copy and open a node, inside and outside isolation
	openList.push_back(new AstarPoint(startPoint.x, startPoint.y));   

	while (!openList.empty())
	{
		//Find the point with the lowest F value
		auto curPoint = getLeastFpoint();  
		//Remove from open list
		openList.remove(curPoint); 
        //put in the close list
		closeList.push_back(curPoint);   
		//1. find the grid that can pass in the current eight squares 
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2. For a grid, if it is not in the open list, add to the open list, set the current grid to its parent, and calculate F G H 
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3. for a grid, it is in the open list, calculate the G value
			//if it is larger than the original, do nothing, otherwise set its parent node as the current point, and update G and F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG<target->G)
				{
					target->parent = curPoint;

					target->G = tempG;
					target->F = calcF(target);
				}
			}
			AstarPoint *resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //Return the node pointer in the list, do not use the original incoming pointer, because a deep copy occurred  
		}
	}

	return NULL;
}





std::list<AstarPoint *> Astar::GetPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner)
{
	AstarPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);

	std::list<AstarPoint *> path;

	//Return the path, if no path is found, return the empty list 
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}

	openList.clear();

	closeList.clear();

	return path;

}
//reload as a pcl::PointCLoud output
bool Astar::GetPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                         const grid_map::GridMap & oExtendGridMap,
	                                 const pcl::PointXYZ & oHeadPoint, 
	                                 const pcl::PointXYZ & oTailPoint, 
	                                              bool isIgnoreCorner)
{

    //clear output
    pAstarCloud->clear();

    //set input
    MapIndex oHeadIdx = ExtendedGM::PointoAllTypeIdx(oHeadPoint, oExtendGridMap);
    AstarPoint startPoint(oHeadIdx.oTwoIndex(0),oHeadIdx.oTwoIndex(1));
    MapIndex oTailIdx = ExtendedGM::PointoAllTypeIdx(oTailPoint, oExtendGridMap);
    AstarPoint endPoint(oTailIdx.oTwoIndex(0),oTailIdx.oTwoIndex(1));

    //compute the path
	AstarPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);

	std::list<AstarPoint *> path;

	//Return the path, if no path is found, return the empty list 
	while (result)
	{

		path.push_front(result);
        //get path member 2D index
		grid_map::Index oPathGridIdx;
		oPathGridIdx(0) = result->x;
		oPathGridIdx(1) = result->y;

        pcl::PointXYZ oPathGridPoint;
        //get path point
        ExtendedGM::TwoDIdxtoPoint(oPathGridPoint,
    	                             oPathGridIdx,
		                           oExtendGridMap);
        //construct path
		pAstarCloud->points.push_back(oPathGridPoint);
		result = result->parent;

	}

	openList.clear();

	closeList.clear();

    //output whether path is generated
	if(pAstarCloud->size())
		return true;
	else
		return false;

}

//reload as a one dimension grid output
bool Astar::GetPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                             std::vector<float> & vQualityFeature,
		                pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                                    const ExtendedGM & oExtendGridMap,
	                  const std::vector<ConfidenceValue> & vConfidenceMap,
	                                     const pcl::PointXYZ & oHeadPoint, 
	                                     const pcl::PointXYZ & oTailPoint, 
	                                                  bool isIgnoreCorner)
{

    //clear output
    pAstarCloud->clear();
    pAttractorCloud->clear();
    vQualityFeature.clear();

    //define a status vector presenting the point clouds of data
    std::vector<bool> vRegionStatus(vConfidenceMap.size(),false);

    //set input
    MapIndex oHeadIdx = ExtendedGM::PointoAllTypeIdx(oHeadPoint, oExtendGridMap.m_oFeatureMap);
    AstarPoint startPoint(oHeadIdx.oTwoIndex(0),oHeadIdx.oTwoIndex(1));
    MapIndex oTailIdx = ExtendedGM::PointoAllTypeIdx(oTailPoint, oExtendGridMap.m_oFeatureMap);
    AstarPoint endPoint(oTailIdx.oTwoIndex(0),oTailIdx.oTwoIndex(1));

    //compute the path
	AstarPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);

	std::list<AstarPoint *> path;

	//Return the path, if no path is found, return the empty list 
	while (result)
	{

		path.push_front(result);
        //get path member 2D index
		grid_map::Index oPathGridIdx;
		oPathGridIdx(0) = result->x;
		oPathGridIdx(1) = result->y;

        pcl::PointXYZ oPathGridPoint;
        //get path point
        ExtendedGM::TwoDIdxtoPoint(oPathGridPoint,
    	                           oPathGridIdx,
		                           oExtendGridMap.m_oFeatureMap);

        //construct path
        oPathGridPoint.z = 0.0;
		pAstarCloud->points.push_back(oPathGridPoint);

        //computed the neighboring grid of one astar path grid
        std::vector<MapIndex> vOneAstarNearIdxs;
        ExtendedGM::CircleNeighborhood(vOneAstarNearIdxs,
	                                   oExtendGridMap.m_oFeatureMap,
	                                   oExtendGridMap.m_vAstarPathMask,
	                                   oPathGridPoint);

        //record in total map status
        for(int i = 0; i!= vOneAstarNearIdxs.size(); ++i)
        	vRegionStatus[vOneAstarNearIdxs[i].iOneIdx] = true;


		result = result->parent;

	}

	openList.clear();

	closeList.clear();

	if(pAstarCloud->size() == 0)
		return false;

	//save the attractor
    for(int i = 0; i != vRegionStatus.size(); ++i){
    	//if it is neiboring region of astar point
        if(vRegionStatus[i]){
            //if it is a selected grid
        	if(vConfidenceMap[i].qualTerm.seletedflag){
                //record point instead of grid
        	   pcl::PointXYZ oAttractorPoint;
               ExtendedGM::OneDIdxtoPoint(oAttractorPoint, i, oExtendGridMap.m_oFeatureMap);
               oAttractorPoint.z = 0.0;
        	   //add it at attractor point clouds
               pAttractorCloud->push_back(oAttractorPoint);
               //record the corresponding quality value
               vQualityFeature.push_back(vConfidenceMap[i].qualTerm.means);
            }
        }

    }

    //output whether path is generated
    return true;

}





AstarPoint *Astar::isInList(const std::list<AstarPoint *> &list, const AstarPoint *point) const
{
	//To determine whether a node is in the list, the pointer cannot be compared here, because each time the join list is a newly opened node, only the coordinates can be compared.
	for (auto p : list)
	if (p->x == point->x&&p->y == point->y)
		return p;
	return NULL;
}




bool Astar::isCanreach(const AstarPoint *point, const AstarPoint *target, bool isIgnoreCorner) const
{
	if (target->x<0 || target->x>maze.size() - 1
		|| target->y<0 || target->y>maze[0].size() - 1
		|| maze[target->x][target->y] == 1
		|| target->x == point->x&&target->y == point->y
		|| isInList(closeList, target)) //Returns false if the point coincides with the current node is out of the map, is an obstacle, or is in the close list
		return false;
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //non-oblique angle pass 
			return true;
		else
		{
			//judge whether is holding in diagonal
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				return true;
			else
				return isIgnoreCorner;
		}
	}
}




std::vector<AstarPoint *> Astar::getSurroundPoints(const AstarPoint *point, bool isIgnoreCorner) const
{
	std::vector<AstarPoint *> surroundPoints;

	for (int x = point->x - 1; x <= point->x + 1; x++)
	for (int y = point->y - 1; y <= point->y + 1; y++)
	if (isCanreach(point, new AstarPoint(x, y), isIgnoreCorner))
		surroundPoints.push_back(new AstarPoint(x, y));

	return surroundPoints;
}




}/*namespace*/