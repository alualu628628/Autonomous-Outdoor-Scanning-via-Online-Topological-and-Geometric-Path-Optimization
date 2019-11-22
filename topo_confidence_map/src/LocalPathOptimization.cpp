#include "LocalPathOptimization.h"

namespace topology_map {


//sort
void PathOptimization::SortFromBigtoSmall(pcl::PointCloud<pcl::PointXY>::Ptr & pAttractorSeq,
	                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAttractorCloud, 
		                                  const std::vector<float> & vQualityFeature,
		                                  unsigned int iMaxNum){

    pAttractorSeq->points.clear();

    //get input
	std::vector<QualityPair> vUnsortAttract;
	vUnsortAttract.reserve(vQualityFeature.size());

	for(int i = 0; i!= pAttractorCloud->points.size();++i){

		QualityPair oPair;
		oPair.idx = i;
		oPair.quality = vQualityFeature[i];
		vUnsortAttract.push_back(oPair);

	}

    //sort from large to small
	for (int i = 0; i < vUnsortAttract.size(); i++)
	{
		QualityPair oTemp;
		for (int j = i + 1; j < vUnsortAttract.size(); j++){

			if (vUnsortAttract[i].quality < vUnsortAttract[j].quality){
				//exchange
				oTemp.quality = vUnsortAttract[i].quality;
				oTemp.idx = vUnsortAttract[i].idx;
				//
				vUnsortAttract[i].idx = vUnsortAttract[j].idx;
				vUnsortAttract[i].quality = vUnsortAttract[j].quality;
				//
				vUnsortAttract[j].idx = oTemp.idx;
				vUnsortAttract[j].quality = oTemp.quality;
			}//end if
		}//end j
	}//end i

    //output
    if(iMaxNum > vUnsortAttract.size())
       iMaxNum = vUnsortAttract.size();
   
    //output the first iMaxNum number members
    for(int i = 0; i < iMaxNum; ++i){

    	int iPointIdx = vUnsortAttract[i].idx;

    	pcl::PointXY oPoint;
    	oPoint.x = pAttractorCloud->points[iPointIdx].x;
    	oPoint.y = pAttractorCloud->points[iPointIdx].y;

    	pAttractorSeq->points.push_back(oPoint);
    }

}










bool PathOptimization::NewLocalPath(pcl::PointCloud<pcl::PointXYZ> & vNewAncherClouds,
	                                const pcl::PointCloud<pcl::PointXY>::Ptr & pAttractorSeq, 
		                            const std::vector<float> & vQualityFeature,
		                            const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAstarCloud, 
	                                const ExtendedGM & oExtendGridMap,
	                                const std::vector<ConfidenceValue> & vConfidenceMap,
		                            float fMoveDis,
		                            int iAnchorNum){

	vNewAncherClouds.clear();

	//if can not constructed a tree
	if (!pAttractorSeq->points.size())
		return false;//return if nothing input
	
	//if can not
	if (!pAstarCloud->points.size())
		return false;//return if nothing input

	//if control points is less
	if (pAttractorSeq->points.size() < iAnchorNum)
		iAnchorNum = pAttractorSeq->points.size();

	pcl::PointCloud<pcl::PointXY>::Ptr pLineCloud(new pcl::PointCloud<pcl::PointXY>);
	std::vector<int> vAstarPointStatus(pAstarCloud->points.size(), -1);
    
	//construct a xy point clouds for line input
	for (int i = 0; i != pAstarCloud->points.size(); ++i) {
		pcl::PointXY oXYPoint;
		oXYPoint.x = pAstarCloud->points[i].x;
		oXYPoint.y = pAstarCloud->points[i].y;
		pLineCloud->points.push_back(oXYPoint);
	}

	//construct a kdtree
	pcl::KdTreeFLANN<pcl::PointXY> oLineTree;
	oLineTree.setInputCloud(pLineCloud);


	int iCnddtCount = 0;//count how many candidates have been selected to compute anchor (not every candidate point can be the anchor) 
	int iAnchorSucess = 0; //The successed seed

	//define a output vector
	std::vector<pcl::PointXYZ> vNewAnchorPoints; 
  
	//compute the index
	while (iCnddtCount < pAttractorSeq->points.size() && iAnchorSucess < iAnchorNum) {

		std::vector<int> vSearchIdx;
		std::vector<float> vSearchDis;
		//search distance of center of sign based on max distance to center

		oLineTree.nearestKSearch(pAttractorSeq->points[iCnddtCount], 1, vSearchIdx, vSearchDis);
        
		//if the searched point is at the head or tail of path, it means the searched point is out of the line region
		if (vSearchIdx[0] > 7 && vSearchIdx[0] < pLineCloud->points.size() - 7) {

			//if this point has not been computed yet
			if (vAstarPointStatus[vSearchIdx[0]]) {
               
				float fAtTravelFlag = false;
                //two possibility
				std::vector<bool> vMvDisFunctionPara;
				vMvDisFunctionPara.push_back(false);
				vMvDisFunctionPara.push_back(true);

                int iMDCount = 0; //move distance function computed times
  
                //add new data set
				pcl::PointXYZ oNewAncherXYZ;
                
                //compute the move action twice if necessary 
                while(!fAtTravelFlag && iMDCount < 2){
                    
                    //compute the shift location
				    pcl::PointXY oNewAncher = ShiftPosition(pLineCloud->points[vSearchIdx[0]],
						                                    pAttractorSeq->points[iCnddtCount], 
						                                    fMoveDis,vMvDisFunctionPara[iMDCount]);

				    oNewAncherXYZ.x = oNewAncher.x;
				    oNewAncherXYZ.y = oNewAncher.y;
				    oNewAncherXYZ.z = 0.0;
                
                    //compute the index
                    int iPointGridIdx = ExtendedGM::PointoOneDIdx(oNewAncherXYZ, oExtendGridMap.m_oFeatureMap);

                    //check the shift location is at the travelable region
                    if(vConfidenceMap[iPointGridIdx].travelable == 1)
                    	fAtTravelFlag = true;

                    iMDCount++;

                }//end while
                
                //if the anchor point is at the travelable region
                if(fAtTravelFlag){
                    
                    //set the point as 
                	std::vector<int> vRadiuSIdx;
				    std::vector<float> vRadiuSDis;

				    //kdtree search
				    oLineTree.radiusSearch(pLineCloud->points[vSearchIdx[0]], fMoveDis, vRadiuSIdx, vRadiuSDis);

                    //label the point status as being selected
				    for (int i = 0; i != vRadiuSIdx.size(); ++i) 
				    	vAstarPointStatus[vRadiuSIdx[i]] = iAnchorSucess;

				    vNewAnchorPoints.push_back(oNewAncherXYZ);

                    //count sucess anchor
					iAnchorSucess++;

				}//end if vConfidenceMap[iPointGridIdx].travelable == 1
				    
			}//end if (vAstarPointStatus[vSearchIdx[0]])

		}//end if vSearchIdx[0] < 5 && vSearchIdx[0] > pLineCloud->points.size() - 5

		iCnddtCount++;

	}//while

    //if not any anchor be generated
    if(!iAnchorSucess)
    	return false;
    
	//get a sequance from astar path with covering labels
	std::vector<int> vSequanceIdx;
	//assigment value
	int iLabel = -1;
	for (int i = 0; i != vAstarPointStatus.size(); ++i) {
		//-1 -1 -1 2 2 2 -1 -1 -1 0 0 0 1 1 1 -1 -1
		//the result is:
		// 2 0 1
		if (vAstarPointStatus[i] != -1 &&
			vAstarPointStatus[i] != iLabel) {
            //get covering label
			vSequanceIdx.push_back(vAstarPointStatus[i]);
			iLabel = vAstarPointStatus[i];
		}

	}

	//exchange
	for(int i = 0; i != vSequanceIdx.size(); ++i)
		vNewAncherClouds.push_back(vNewAnchorPoints[vSequanceIdx[i]]);

	return true;

}






pcl::PointXY PathOptimization::ShiftPosition(const pcl::PointXY & oLinePoint,
	                                         const pcl::PointXY & oObsPoint,
	                                         const float & fMovingDis,
	                                         bool bTransposeFlag){
 
    //           -fKTime                      fKTime
    //   * ----------------------- *----------------------- *------------------*
    //oMovedPoint(transposed)  oLinePoint           oMovedPoint(trans)     oObsPoint

	float fTranParam = 1.0;
    //using transposed model (compute forward)
	if(bTransposeFlag)
		fTranParam = -1.0;

	//normalizetion
	//multiplied by the ratio
	float fKTime = fMovingDis / sqrt(pow(oLinePoint.x - oObsPoint.x, 2.0) +
		                             pow(oLinePoint.y - oObsPoint.y, 2.0));

    pcl::PointXY oMovedPoint;
	oMovedPoint.x = oLinePoint.x + fTranParam * fKTime * (oObsPoint.x - oLinePoint.x);
	oMovedPoint.y = oLinePoint.y + fTranParam * fKTime * (oObsPoint.y - oLinePoint.y);

	return oMovedPoint;

}



}/*namespace*/