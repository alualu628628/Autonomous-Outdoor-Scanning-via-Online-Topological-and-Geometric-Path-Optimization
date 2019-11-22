#ifndef LOCALPATHOPTIMIZATION_H
#define LOCALPATHOPTIMIZATION_H

#include "OP.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace topology_map {

struct QualityPair{

	int idx;
	float quality;

	QualityPair(){
		idx = 0;
		quality = 0.0;
	};

};


class PathOptimization{

public:
	
	//
	PathOptimization(){
	
	};

	//sort the controls from max to min
	void SortFromBigtoSmall(pcl::PointCloud<pcl::PointXY>::Ptr & pAttractorSeq,
	                        const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                    const std::vector<float> & vQualityFeature,
		                    unsigned int iMaxNum = 20);

	////compute the moving distance
	static pcl::PointXY ShiftPosition(const pcl::PointXY & oLinePoint,
		                              const pcl::PointXY & oObsPoint,
		                              const float & fMovingDis,
		                              bool bTransposeFlag = false);

	////generate new local path
	bool NewLocalPath(pcl::PointCloud<pcl::PointXYZ> & vNewAncherClouds,
	                  const pcl::PointCloud<pcl::PointXY>::Ptr & pAttractorSeq, 
		              const std::vector<float> & vQualityFeature,
		              const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                  const ExtendedGM & oExtendGridMap,
	                  const std::vector<ConfidenceValue> & vConfidenceMap,
		              float fMoveDis = 1.5,
		              int iAnchorNum = 1);

	//construct a 2D point clouds

private:


};



}/*namespace*/


#endif // !LocalPathOptimization_h

