#ifndef ASTAR_H
#define ASTAR_H

#include <vector>  
#include <list>  
#include <iostream>
#include <math.h>  

#include "OP.h"


namespace topology_map {
///************************************************************************///
// a class to implement the A* (called A star) algorithm
// A* - A star algorithm, the raw code is from
// https://github.com/lnp1993/Astar.git
// modified by Huang Pengdi, 2019.01.03

//Version 1.0 
// - raw version by the original author
//Version 2.0 2019.01.03
// - modified the class to adapt to husky system
//Version 3.0 2019.05.07
// - modified to a online version

///************************************************************************///


struct AstarPoint
{
	//Point coordinates, here for the convenience of C + + array calculation, x for horizontal row, y for vertical column
	int x, y; 
	//F = G + H  
	int F, G, H; 
	//The coordinates of the parent, there is no pointer here, which simplifies the code  
	AstarPoint *parent; 
	//Variable initialization  
	AstarPoint(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL){
		//none
	}

};

//************************************
//Class: A* method
// 
//************************************
class Astar
{
public:

    Astar():kCost1(10), kCost2(10){

    }

    void InitAstarTravelMap(const grid_map::GridMap & oExtendGridMap);

    //update map which record the travelable grids
    void UpdateTravelMap(const grid_map::GridMap & oExtendGridMap,
	                     const std::vector<ConfidenceValue> & vConfidenceMap);

	std::list<AstarPoint *> GetPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner);
    //roload for a point cloud path output
	bool GetPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                      const grid_map::GridMap & oExtendGridMap,
	                              const pcl::PointXYZ & oHeadPoint, 
	                              const pcl::PointXYZ & oTailPoint, 
	                                           bool isIgnoreCorner);

	//roload for a path region map index output
	bool GetPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                          std::vector<float> & vQualityFeature,
		             pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                                 const ExtendedGM & oExtendGridMap,
	               const std::vector<ConfidenceValue> & vConfidenceMap,
	                                  const pcl::PointXYZ & oHeadPoint, 
	                                  const pcl::PointXYZ & oTailPoint, 
	                                               bool isIgnoreCorner);

	//the obstacle map
	std::vector<std::vector<int>> maze;

private:

	AstarPoint *findPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner);

	std::vector<AstarPoint *> getSurroundPoints(const AstarPoint *point, bool isIgnoreCorner) const;
	//judge whether a point can be used for the next step  
	bool isCanreach(const AstarPoint *point, const AstarPoint *target, bool isIgnoreCorner) const; 
	//judge whether a point is included in the on/off list
	AstarPoint *isInList(const std::list<AstarPoint *> &list, const AstarPoint *point) const; 
	//Returns the node with the lowest F value from the open list
	AstarPoint *getLeastFpoint(); 

	//compute FGH value  
	int calcG(AstarPoint *temp_start, AstarPoint *point);

	int calcH(AstarPoint *point, AstarPoint *end);

	int calcF(AstarPoint *point);

    //data set
    //open the list
	std::list<AstarPoint *> openList;    
    //close the list
	std::list<AstarPoint *> closeList;   

	const int kCost1; //Direct movement cost  
    const int kCost2; //Diagonal movement cost

};


}/*namespace*/

#endif // !ASTAR_H