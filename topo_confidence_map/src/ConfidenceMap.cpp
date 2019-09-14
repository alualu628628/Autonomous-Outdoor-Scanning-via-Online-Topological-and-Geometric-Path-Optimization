 #include "ConfidenceMap.h"

namespace topology_map {

/*************************************************
Function: Confidence
Description: constrcution function for Confidence class
Calls: SetSigmaValue()
       SetVisTermThr()
       SetNodeGenParas()
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: f_fSigma - the computed radius of robot
	   f_fGHPRParam - the parameter of GHPR algorithm
	   f_fVisTermThr - the threshold of visbility term - useless
	   f_fMinNodeThr - minimum value to generate node
Output: parameter initialization
Return: none
Others: m_fTraversWeight - traverel weight
        m_fExploreWeight - visibility weight
        m_fDisWeight - distance term weight
        m_fBoundWeight - bound term weight
*************************************************/
Confidence::Confidence(float f_fSigma,
	                   float f_fGHPRParam,
	                  float f_fVisTermThr,
	                  float f_fMinNodeThr):
	                 m_fTraversWeight(0.9),
                     m_fExploreWeight(0.1),
                         m_fDisWeight(0.6),
                       m_fBoundWeight(0.4){

    //set sigma value
	SetSigmaValue(f_fSigma);

    //set visibility parameters
	SetVisTermThr(f_fVisTermThr);

    //set node generation parameter
	SetNodeGenParas(f_fMinNodeThr);

	//srand((unsigned)time(NULL));

}


/*************************************************
Function: ~Confidence
Description: destrcution function for Confidence class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
Confidence::~Confidence(){


}

/*************************************************
Function: SetSigmaValue
Description: set value to the private data member m_fSigma
Calls: none
Called By: Confidence()
Table Accessed: none
Table Updated: none
Input: f_fSigma - a given sigma value depends on the scanning region
Output: none
Return: none
Others: none
*************************************************/
void Confidence::SetSigmaValue(const float & f_fSigma) {

	m_fSigma = f_fSigma;

}


/*************************************************
Function: SetVisTermThr
Description: set value to the private data member m_fVisTermThr
Calls: none
Called By: Confidence()
Table Accessed: none
Table Updated: none
Input: f_fVisTermThr - a visibility term threshold
Output: none
Return: none
Others: none
*************************************************/
void Confidence::SetVisTermThr(const float & f_fVisTermThr) {

	m_fVisTermThr = f_fVisTermThr;

}

/*************************************************
Function: SetTermWeight
Description: set value to the parameters related to visibility term
Calls: none
Called By: main function or external call
Table Accessed: none
Table Updated: none
Input: f_fTraversWeight - traverel term weight
	   f_fExploreWeight - explore term weighr
	   f_fDisWeight - distance term weight
	   f_fBoundWeight - boundary term weight
Output: m_fTraversWeight
	    m_fExploreWeight
        m_fDisWeight
	    m_fBoundWeight
Return: none
Others: Theoretically,  f_fTraversWeight + f_fExploreWeight = 1, f_fDisWeight + f_fBoundWeight = 1
        However, in fact, each weight can be assigned an appropriate value like f_fTraversWeight + f_fExploreWeight > 1
*************************************************/
void Confidence::SetTermWeight(const float & f_fTraversWeight,
	                           const float & f_fExploreWeight,
	                           const float & f_fDisWeight,
	                           const float & f_fBoundWeight){

    //get input
	m_fTraversWeight = f_fTraversWeight;
	m_fExploreWeight = f_fExploreWeight;
    m_fDisWeight = f_fDisWeight;
	m_fBoundWeight = f_fBoundWeight;

}
//reload with normalization input
//in this reload function
//the weights must follow :
//f_fTraversWeight + f_fExploreWeight = 1
//f_fDisWeight + f_fBoundWeight = 1
void Confidence::SetTermWeight(const float & f_fTraversWeight,
	                           const float & f_fDisWeight){

    //get input
	m_fTraversWeight = f_fTraversWeight;
	m_fExploreWeight = 1.0 - f_fTraversWeight;
    m_fDisWeight = f_fDisWeight;
	m_fBoundWeight = 1.0 - f_fDisWeight;

}


/*************************************************
Function: SetNodeGenParas
Description: set value to the parameters of node generation threshold
Calls: none
Called By: Confidence
Table Accessed: none
Table Updated: none
Input: f_fMinNodeThr - a threshold of node generation based on total confidence value
Output: m_fMinNodeThr
Return: none
Others: none
*************************************************/
void Confidence::SetNodeGenParas(const float & f_fMinNodeThr){

	m_fMinNodeThr = f_fMinNodeThr;

}


/*************************************************
Function: OutNodeGenParas
Description: output the node generation threshold
Calls: none
Called By: main function or external call
Table Accessed: none
Table Updated: none
Input: none
Output: fMinNodeThr
Return: fMinNodeThr
Others: none
*************************************************/
float Confidence::OutNodeGenParas(){

	float fMinNodeThr = m_fMinNodeThr;
	return fMinNodeThr;

}


/*************************************************
Function: VectorInnerProduct
Description: This is an inner product operation of two vectors
Calls: none
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: oAVec - vector A
       oBVec - vector B
Output: the inner product value of vector A and vector B
Return: an inner product value
Others: none
*************************************************/
float Confidence::VectorInnerProduct(const pcl::PointXYZ & oAVec,
	                                 const pcl::PointXYZ & oBVec){

	//a*b = xa*xb + ya*yb + za*zb
	return oAVec.x * oBVec.x + oAVec.y * oBVec.y + oAVec.z * oBVec.z;

}

/*************************************************
Function: GaussianKernel
Description: This is a Gaussian Kernel Function to smooth the distance value between two points
Calls: Compute2Norm
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
	   sigma - the parameter that controls the affect radius of Gaussian Function, therefore the value
	   of sigma is normally set as half of searched radius 
Output: none
Return: none
Others: none
*************************************************/
inline float Confidence::GaussianKernel(const pcl::PointXYZ & oQueryPo,
	                                    const pcl::PointXYZ & oTargerPo, 
	                                                      float & sigma){

	// k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (2 * sigma^2) }
	//or k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (sigma^2) }

	//distance between two input vector
	float fNormSquare = Compute2Norm(oQueryPo, oTargerPo);
	
	      fNormSquare = pow(fNormSquare, 2.0f);
	//ouput equation result
	return exp(-1.0f * fNormSquare / pow(sigma,2.0f));
	
}

/*************************************************
Function: LinearKernel
Description: This is a Linear Kernel Function to compute the visibility value 
Calls: none
Called By: 
Table Accessed: none
Table Updated: none
Input: fTargetVal - input of linear function
	   fThrVal - the threshold of linear function,
	   the value will be 1 if input is larger than this one 
Output: the respond of linear function
Return: float computed value
Others: none
*************************************************/
inline float Confidence::LinearKernel(const float & fTargetVal,
	                         const float & fThrVal){

	//if the input value is smaller than the given threshold 
	if (fTargetVal < fThrVal)
		return fTargetVal / fThrVal;
	else
		return 1.0;

}

/*************************************************
Function: StandardDeviation
Description: This is a Linear Kernel Function to compute the visibility value
Calls: none
Called By:
Table Accessed: none
Table Updated: none
Input: fTargetVal - input of linear function
          fThrVal - the threshold of linear function,
          the value will be 1 if input is larger than this one
Output: the respond of linear function
Return: float computed value
Others: none
*************************************************/
float Confidence::StandardDeviation(const PCLCloudXYZ & vCloud){

	//define output
	float fSDeviation = 0.0;

	//compute the mean value of point set
	pcl::PointXYZ oMeanPoint = ComputeCenter(vCloud);
	for (int i = 0; i != vCloud.points.size(); ++i) {
	    //accumulation
		fSDeviation += ComputeSquareNorm(oMeanPoint, vCloud.points[i]);

	}

	//compute the standard deviation
	fSDeviation = sqrt(fSDeviation/ float(vCloud.points.size()));

	return fSDeviation;

}

/*************************************************
Function: ComputeDensity
Description: This is a Linear Kernel Function to compute the visibility value
Calls: none
Called By:
Table Accessed: none
Table Updated: none
Input: fTargetVal - input of linear function
fThrVal - the threshold of linear function,
the value will be 1 if input is larger than this one
Output: the respond of linear function
Return: float computed value
Others: none
*************************************************/
inline float Confidence::ComputeDensity(const PCLCloudXYZ & vCloud,
								                   int iSampleTimes,
								                       bool bKDFlag){

	//use the kdtree structureto compute density if bKDFlag is true
	if(bKDFlag){

		//if point number is very small
		if(vCloud.points.size() < iSampleTimes)
			return 1.0;//it is neighborhood is onlt itself
		
		PCLCloudXYZPtr pGridCloud(new PCLCloudXYZ);
		//construct a point clouds
	    pGridCloud->width = vCloud.points.size();
	    pGridCloud->height = 1;
	    pGridCloud->is_dense = false;
	    pGridCloud->points.resize(pGridCloud->width * pGridCloud->height);

		for(int i = 0;i != vCloud.size(); ++i){

			pGridCloud->points[i].x = vCloud.points[i].x;
			pGridCloud->points[i].y = vCloud.points[i].y;
			pGridCloud->points[i].z = vCloud.points[i].z;
		}

		//construct a kdtree
	    pcl::KdTreeFLANN<pcl::PointXYZ> oGridCloudTree;
	    oGridCloudTree.setInputCloud(pGridCloud);
		
		//get the random query index of point
		std::vector<int> vQueryIndices = GetRandom(pGridCloud->points.size(), iSampleTimes);

		//total point number
		int iNeighPNums = 0;

		//find indices using kdtree
	    for (size_t i = 0; i != vQueryIndices.size(); ++i) {
			
			//define temps
			std::vector<int> vNearestIdx;
			std::vector<float> vNearestDis;
			
			//search the nearest raw point of query constructed convex hull surface point 
			oGridCloudTree.radiusSearch(pGridCloud->points[vQueryIndices[i]], 0.3, vNearestIdx, vNearestDis);
			
			//accumulation
			iNeighPNums += vNearestIdx.size();
		
		}//end for i != pHullCloud->points.size()

		return float(iNeighPNums)/float(iSampleTimes);

	}else{//ouput the point number directly in false model
	
		return float(vCloud.points.size());
	}

}



/*************************************************
Function: Compute2Norm
Description: compute the Euclidean distance between two points
Calls: none
Called By: GaussianKernel
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
Output: none
Return: none
Others: This function is the same with ComputeEuclideanDis, but it is an online one
*************************************************/
inline float Confidence::Compute2Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPo.x - oTargerPo.x, 2.0f)
		      + pow(oQueryPo.y - oTargerPo.y, 2.0f)
		      + pow(oQueryPo.z - oTargerPo.z, 2.0f));

}


/*************************************************
Function: ComputeSquareNorm
Description: compute the square of norm
Calls: none
Called By: GaussianKernel
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
Output: the Euclidean distance value between two points
Return: a distance value 
Others: This function is the same with ComputeEuclideanDis, but it is an online one
*************************************************/
inline float Confidence::ComputeSquareNorm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	return pow(oQueryPo.x - oTargerPo.x, 2.0f)
		     + pow(oQueryPo.y - oTargerPo.y, 2.0f)
	         + pow(oQueryPo.z - oTargerPo.z, 2.0f);

}


/*************************************************
Function: ComputeCenter
Description: compute the center point of a given point set with quired index
Calls: none
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: vCloud - a 3D point clouds
       vPointIdx - the point index vector indicates which point need to be computed 
Output: centerpoint center point(position) of the queried point set
Return: centerpoint
Others: none
*************************************************/
pcl::PointXYZ Confidence::ComputeCenter(const PCLCloudXYZ & vCloud,
	                            const std::vector<int> & vPointIdx){

	//define output
	pcl::PointXYZ oCenter;
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	//check whether the input has point 
	if(!vPointIdx.size())
		return oCenter;

	//calculate the average
	for (int i = 0; i != vPointIdx.size(); ++i) {

		oCenter.x = oCenter.x + vCloud.points[vPointIdx[i]].x;
		oCenter.y = oCenter.y + vCloud.points[vPointIdx[i]].y;
		oCenter.z = oCenter.z + vCloud.points[vPointIdx[i]].z;

	}

	oCenter.x = oCenter.x / float(vPointIdx.size());
	oCenter.y = oCenter.y / float(vPointIdx.size());
	oCenter.z = oCenter.z / float(vPointIdx.size());

	//output
	return oCenter;

}

/*************************************************
Function: ComputeCenter - Reload
Description:  compute the center point of a given point set 
Input: vCloud - a 3D point clouds
*************************************************/
pcl::PointXYZ  Confidence::ComputeCenter(const PCLCloudXYZ & vCloud){

	//define output
	pcl::PointXYZ oCenter;
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	//check whether the input has point 
	if (!vCloud.points.size())
		return oCenter;

	//calculate the average
	for (int i = 0; i != vCloud.points.size(); ++i) {
	
		oCenter.x = oCenter.x + vCloud.points[i].x;
		oCenter.y = oCenter.y + vCloud.points[i].y;
		oCenter.z = oCenter.z + vCloud.points[i].z;
	
	}

	oCenter.x = oCenter.x / float(vCloud.points.size());
	oCenter.y = oCenter.y / float(vCloud.points.size());
	oCenter.z = oCenter.z / float(vCloud.points.size());

	//output
	return oCenter;

}


/*************************************************
Function: GetRandom
Description: compute the Euclidean distance between two points
Calls: none
Called By: main function of project or other classes
Table Accessed: none
Table Updated: none
Input: oQueryPoint - the query point (based point)
oTargetPoint - the target point
Output: the distance value
Return: a distance value
Others: This function is the same with Compute1Norm, but it is a static one
*************************************************/
std::vector<int> Confidence::GetRandom(const unsigned int iSize,
		                               const int iSampleNums){

	//define output vector
	std::vector<int> vAllValueVec(iSize, 0);

	//get all of value
	for (int i = 0; i != iSize; ++i)
		vAllValueVec[i] = i;

	//if the sampling number is larger than the size of total number
	if (iSize <= iSampleNums) {

		return vAllValueVec;

	}

	//the last number
	int iLastIdx = iSize - 1;
	int iCurSampNum = 0;

	//get the random value
	while (iSampleNums - iCurSampNum) {

		//defend repeat selection
		int iRandomRes = (rand() % (iLastIdx - iCurSampNum + 1));
			
		//exchange the last one of unselected value and current randon selected value
		int iTempValue = vAllValueVec[iRandomRes];
		vAllValueVec[iRandomRes] = vAllValueVec[iLastIdx - iCurSampNum];
		vAllValueVec[iLastIdx - iCurSampNum] = iTempValue;
		//count
		iCurSampNum++;

	}//while

	//get the last iCurSampNum value of total value
	std::vector<int> iRandomVec;
	iRandomVec.reserve(iCurSampNum);
	for (int i = iLastIdx; i != (iLastIdx - iCurSampNum); --i) {

		iRandomVec.push_back(vAllValueVec[i]);

	}

	//over
	return iRandomVec;

}
//reload
/*//=================================================================================
std::vector<int> Confidence::GetRandom(const PCLCloudXYZPtr & pAllTravelCloud,
	                                                         GridMap & oMaper,
	                                                    const int iSampleNums){

	std::vector<int> vRandomVec;
	int iSize = pAllTravelCloud->points.size();
	//define output vector
	std::vector<int> vAllValueVec(pAllTravelCloud->points.size(), 0);

	//get all of value
	for (int i = 0; i != iSize; ++i)
		vAllValueVec[i] = i;

	//if the sampling number is larger than the size of total number
	if (iSize <= iSampleNums) {

		return vAllValueVec;

	}

	//the last number
	int iLastIdx = iSize - 1;
	int iCurSampNum = 0;

	//get the random value
	while (iSampleNums - iCurSampNum) {

		//defend repeat selection
		int iRandomRes = (rand() % (iLastIdx - iCurSampNum + 1));

		int iRandomGrid = oMaper.AssignPointToMap(pAllTravelCloud->points[iRandomRes]);

		if (oMaper.vConfidenceMap[iRandomGrid].label == 2) {
			//exchange the last one of unselected value and current randon selected value
			int iTempValue = vAllValueVec[iRandomRes];
			vAllValueVec[iRandomRes] = vAllValueVec[iLastIdx - iCurSampNum];
			vAllValueVec[iLastIdx - iCurSampNum] = iTempValue;
			//save the result
			vRandomVec.push_back(iRandomRes);
		//count
		iCurSampNum++;
        }

	}//while

	//over
	return vRandomVec;

}
=================================================================================8/


/*************************************************
Function: ComputeEuclideanDis
Description: compute the Euclidean distance between two points
Calls: none
Called By: main function of project or other classes
Table Accessed: none
Table Updated: none
Input: oQueryPoint - the query point (based point)
       oTargetPoint - the target point 
Output: the distance value
Return: a distance value
Others: This function is the same with Compute1Norm, but it is a static one
*************************************************/
float Confidence::ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		      + pow(oQueryPoint.y - oTargetPoint.y, 2.0f)
		      + pow(oQueryPoint.z - oTargetPoint.z, 2.0f));

}


/*************************************************
Function: DistanceTerm
Description: the function is to compute the distance feature to the confidence value
Calls: GaussianKernel()
       ComputeTotalCoffidence()
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   oRobotPoint - the location of the robot  
	   vNearGroundIdxs - the neighboring grids based on the input robot location
	   vGroundCloud - the NEARBY travelable point clouds (the NEARBY ground point clouds) 
Output: update the total confidence (actually the travelTerm) value of the given nearby grid
Return: none
Others: none
*************************************************/
void Confidence::DistanceTerm(std::vector<ConfidenceValue> & vConfidenceMap,
	                                      const pcl::PointXYZ & oRobotPoint,
                                   const std::vector<int> & vNearGroundIdxs,
	                                       const PCLCloudXYZ & vGroundCloud){

	//**compute the distance part** 
	for (int i = 0; i != vNearGroundIdxs.size(); ++i) {

		int iNearGridId = vNearGroundIdxs[i];
	
		//non-empty ground grid
		//if (vConfidenceMap[iNearGridId].label == 2) {

			//compute smooth distance using Gaussin Kernel based on the center point
			//the empty grid has zero value in this term
			float fGridTravelRes = GaussianKernel(oRobotPoint, 
				                                  vGroundCloud.points[i], 
				                                  m_fSigma);

			//**********Incremental item************
	        //fd(p) = max(fd(pi))  
		    if(vConfidenceMap[iNearGridId].travelTerm < fGridTravelRes)
		    	vConfidenceMap[iNearGridId].travelTerm = fGridTravelRes;

            //update the total coffidence value for new distance value
		    ComputeTotalCoffidence(vConfidenceMap,iNearGridId);

		//}//end if 

	}//end i
	
}


/*************************************************
Function: BoundTerm
Description: the function is to compute the boundary feature for the confidence value
Calls: ComputeTotalCoffidence()
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   vNearGroundIdxs - the nearby ground grid idx 
	   pGroundCloud - the NEARBY travelable point clouds (the NEARBY ground point clouds)
	   pBoundCloud - the NEARBY boundary point clouds
Output: update the total confidence value (actually the boundTerm) of the given nearby grid
Return: none
Others: the calculation of bound term is nothing to do with the robot position
*************************************************/
void Confidence::BoundTerm(std::vector<ConfidenceValue> & vConfidenceMap,
                               const std::vector<int> & vNearGroundIdxs,
	                                const PCLCloudXYZPtr & pGroundCloud,
	                                 const PCLCloudXYZPtr & pBoundCloud){

	 //**compute the center offset part**
	 //maybe there is not any boundary in an open area
	if (pBoundCloud->points.size()) {

        //define a threshold indicating where is dangerous for robot to close  
		//float fNoTouchThr = (m_fSigma - 0.5) / m_fSigma;

        //set a kdtree for boundary point clouds
		pcl::KdTreeFLANN<pcl::PointXYZ> oBoundTree;
		oBoundTree.setInputCloud(pBoundCloud);

		//for each non-empty neighboring grid
		for (int i = 0; i != vNearGroundIdxs.size(); ++i) {

			//compute the distance between boundary and travelable region
			std::vector<int> vSearchIdx;
			std::vector<float> vSearchDis;

			oBoundTree.nearestKSearch(pGroundCloud->points[i], 1, vSearchIdx, vSearchDis);

			//compute the boundary distance 
			float fBoundvalue = (m_fSigma - sqrt(vSearchDis[0])) / m_fSigma;
			if (fBoundvalue < 0)
				fBoundvalue = 0.0;
            //record the maximum boundary responde indicating the closest distance
			if (vConfidenceMap[vNearGroundIdxs[i]].boundTerm < fBoundvalue)
				vConfidenceMap[vNearGroundIdxs[i]].boundTerm = fBoundvalue;

            //also affect the travelable when the robot is too close to wall
			//if (vConfidenceMap[vNearGroundIdxs[i]].boundTerm > fNoTouchThr)
            //    vConfidenceMap[vNearGroundIdxs[i]].travelable = 4;

            //update the total value
            //thereby the boundterm will be calculated at the end of the other two terms 
            ComputeTotalCoffidence(vConfidenceMap,vNearGroundIdxs[i]);

		}//end i 

	}//end if

}


/*************************************************
Function: OcclusionTerm
Description: the function is to compute the visibility feature to the confidence value
Calls: GHPR class
       ComputeTotalCoffidence()
Called By: main function of project or other classes
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
       pNearAllCloud - the NEARBY point clouds (the NEARBY ground, obstacle and boundary points)
	   vNearGroundIdxs - the NEARBY ground grid index
	   oPastViewPoint - the past viewpoint (past robot position, also the )
	   iNodeTimes - number of node generations
Output: update the total confidence value (actually the visiTerm) of the given nearby grid
Return: none
Others: none
*************************************************/
void Confidence::OcclusionTerm(std::vector<ConfidenceValue> & vConfidenceMap,
	                                          PCLCloudXYZPtr & pNearAllCloud,
	                                const std::vector<int> & vNearGroundIdxs,
	                                    const pcl::PointXYZ & oPastViewPoint,
	                                                  const int & iNodeTimes){ 
    
	//check the point cloud size (down sampling if point clouds is too large)
	unsigned int iNonGrndPSize = pNearAllCloud->points.size() - vNearGroundIdxs.size();
	unsigned int iSmplThr = 500000;

    //if need sampling
	if(iNonGrndPSize > iSmplThr){

		pcl::PointCloud<pcl::PointXYZ> vSamplingClouds;
		//retain ground points
		for(int i = 0; i != vNearGroundIdxs.size(); ++i)
			vSamplingClouds.push_back(pNearAllCloud->points[i]);

		//down sampling
		int iSmplNum = int(iNonGrndPSize / iSmplThr);

		//only down sample the non-ground points 
		for(int i = vNearGroundIdxs.size(); i != pNearAllCloud->points.size(); i = i + iSmplNum){
			//sampling
			vSamplingClouds.push_back(pNearAllCloud->points[i]);

		}

        pNearAllCloud->clear();

		//restore the point clouds
		for(int i = 0; i != vSamplingClouds.points.size(); ++i)
			pNearAllCloud->points.push_back(vSamplingClouds.points[i]);//get back

	}//end if iNonGrndPSize > iSmplThr
   
	//using the GHPR algorithm 
	GHPR oGHPRer(3.7);
    
	//**********Measurement item************
	//compute the visibility based on the history of view points
	std::vector<bool> vVisableRes = oGHPRer.ComputeVisibility(*pNearAllCloud, oPastViewPoint);
	
	//**********Incremental item************
	//fv(p) = fv(n)  
	for (int i = 0; i != vNearGroundIdxs.size(); ++i){
		//count in each view
		vConfidenceMap[vNearGroundIdxs[i]].visiTerm.totaltimes++;
		//if it is a clear view
		if(vVisableRes[i])//get the occlusion result
			vConfidenceMap[vNearGroundIdxs[i]].visiTerm.visibletimes += 1.0;

		vConfidenceMap[vNearGroundIdxs[i]].visiTerm.value = vConfidenceMap[vNearGroundIdxs[i]].visiTerm.visibletimes /
		                                                    vConfidenceMap[vNearGroundIdxs[i]].visiTerm.totaltimes;

        //update the confidence map
		ComputeTotalCoffidence(vConfidenceMap,vNearGroundIdxs[i]);                           

	}
    
	//output the occlusion result of point clouds - for test only
	//OutputOcclusionClouds(*pNearAllCloud, vVisableRes, oPastViewPoint);
 
}


/*************************************************
Function: QualityTermUsingDensity
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
GaussianKernel
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
oRobotPoint - the location of the robot
vNearbyGridIdxs - the neighboring grids based on the input robot location
vTravelCloud - the travelable point clouds (the ground point clouds)
vGridTravelPsIdx - the index of point within each grid to total travelable point clouds
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
//void Confidence::QualityTermUsingDensity(std::vector<ConfidenceValue> & vConfidenceMap,
//                               const std::vector<int> & vNearbyGridIdxs,
//	                                  const PCLCloudXYZ & vTravelCloud,
//	            const std::vector<std::vector<int>> & vGridTravelPsIdx,
//	                                const PCLCloudXYZ & vAllBoundCloud,
//	             const std::vector<std::vector<int>> & vGridBoundPsIdx,
//	                                const PCLCloudXYZ & vObstacleCloud,
//	               const std::vector<std::vector<int>> & vGridObsPsIdx) {
//
//	//save the point that is in an unreachable grid
//	for (int i = 0; i != vNearbyGridIdxs.size(); ++i) {
//
//		//point clouds to be seen
//		PCLCloudXYZPtr pMeasuredCloud(new PCLCloudXYZ);
//
//		int iOneGridIdx = vNearbyGridIdxs[i];
//
//		//record the ground point
//		for (int j = 0; j != vGridTravelPsIdx[iOneGridIdx].size(); ++j)
//			pMeasuredCloud->points.push_back(vTravelCloud.points[vGridTravelPsIdx[iOneGridIdx][j]]);
//
//		//record the boundary point
//		for (int j = 0; j != vGridBoundPsIdx[iOneGridIdx].size(); ++j)
//			pMeasuredCloud->points.push_back(vAllBoundCloud.points[vGridBoundPsIdx[iOneGridIdx][j]]);
//
//		//record the obstacle point
//		for (int j = 0; j != vGridObsPsIdx[iOneGridIdx].size(); ++j)
//			pMeasuredCloud->points.push_back(vObstacleCloud.points[vGridObsPsIdx[iOneGridIdx][j]]);
//		
//		//estimate the quality of feature
//		//compute the quality using the density feature
//		//vConfidenceMap[iOneGridIdx].quality = ComputeDensity(*pMeasuredCloud,5);
//		//compute the quality using the standard deviation feature
//		vConfidenceMap[iOneGridIdx].quality = StandardDeviation(*pMeasuredCloud);
//
//	}//end for i
//
//}

/*************************************************
Function: QualityTerm
Description: the function is to measuer the scanning quality of point clouds
Calls: HausdorffDimension class
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   pObstacleCloud - the given obstacle point clouds to be measured
       vObstNodeTimes - the iNodeTime value corresponding to pObstacleCloud
       vObstlPntMapIdx - the grid index of each obstacle point
	   oExtendGridMap - grid map
	   vNearByIdxs - nearby grid index of robot
       iNodeTime - the current times of node generations
       iSmplNum - number of seeds, the seed is randonly selected 
Output: update the qualTerm value of the given nearby grid
Return: none
Others: none
*************************************************/
void Confidence::QualityTerm(std::vector<ConfidenceValue> & vConfidenceMap,
	                                 const PCLCloudXYZPtr & pObstacleCloud,
                                   const std::vector<int> & vObstNodeTimes,
                    const std::vector<std::vector<int> > & vObstlPntMapIdx,
		                                 const ExtendedGM & oExtendGridMap,
		                         const std::vector<MapIndex> & vNearByIdxs,
                                                     const int & iNodeTime,
                                                              int iSmplNum){

	//define non ground grid indexes
    std::vector<int> vNonGrndGrids;
    
    //record 
	for(int i = 0; i != vNearByIdxs.size(); ++i){

		int iOneNearIdx = vNearByIdxs[i].iOneIdx;
						//if it is a obstacle grid
		if(vConfidenceMap[iOneNearIdx].label == 1 || vConfidenceMap[iOneNearIdx].label == 3){
			//if it has been scanned
			vNonGrndGrids.push_back(iOneNearIdx);

		}//end if
    }//end for

    //check distribution
    if(!vNonGrndGrids.size())
    	return;

    //construct a candidates
    //choose a grid (randomly), which is equal to down sampling
    std::vector<int> vSelectedGrids = GetRandom(vNonGrndGrids.size(), iSmplNum);

    //compute the dimension feature of each selected obstacle grid
    for(int is = 0; is != vSelectedGrids.size(); ++is){

    	int iOneSlctIdx = vNonGrndGrids[vSelectedGrids[is]];
        
        //point clouds to be measured
	    PCLCloudXYZPtr pMeasuredCloud(new PCLCloudXYZ);

	    //compute the local region based on the selected grid
        std::vector<int> vMeasuredGridIdx; 
		ExtendedGM::CircleNeighborhood(vMeasuredGridIdx,
						               oExtendGridMap.m_oFeatureMap, 
									   oExtendGridMap.m_vLocalQualityMask,
		                               iOneSlctIdx);

	    //save the point that is in an unreachable grid
	    for (int i = 0; i != vMeasuredGridIdx.size(); ++i) {

		    int iOneGridIdx = vMeasuredGridIdx[i];
		    //if this grid is a obstacle grid or boundary grid
		    if(vConfidenceMap[iOneGridIdx].label == 1 || vConfidenceMap[iOneGridIdx].label == 3){

			    for (int j = 0; j != vObstlPntMapIdx[iOneGridIdx].size(); ++j){
        	        if(vObstNodeTimes[vObstlPntMapIdx[iOneGridIdx][j]] == iNodeTime)//if it is recorded at current node time
        	    	    pMeasuredCloud->points.push_back(pObstacleCloud->points[vObstlPntMapIdx[iOneGridIdx][j]]);

        	    }//end for j
	
		    }//end if

	    }//end for i

        //if not points input
        if(pMeasuredCloud->size() < 10)
        	continue;
	    //using Hausdorff Dimension to measure point clouds
	    HausdorffDimension oHDor(5, 1);
	    
	    //set the 
	    oHDor.SetMinDis(0.1);
	    
	    //set the dimension type
	    oHDor.SetParaQ(0);
	   
	    //compute the Hausdorff result
	    float fHausRes = oHDor.BoxCounting(*pMeasuredCloud);
        //give large weight for less scan
	    fHausRes = fHausRes - 2.0f;
	    if(fHausRes < 0.0)
	       fHausRes = -1.5f*fHausRes;

	    //record each measured point clouds for test only
        //OutputQualityClouds(*pMeasuredCloud, fHausRes);
	    //assig at the selected grid because it is grid
        //quality term
        for(int i = 0; i!= vMeasuredGridIdx.size(); ++i){
            
            int iOneGridIdx = vMeasuredGridIdx[i];
		    //if this grid is a obstacle grid or boundary grid
		    if(vConfidenceMap[iOneGridIdx].label == 1 || vConfidenceMap[iOneGridIdx].label == 3){
               vConfidenceMap[iOneGridIdx].qualTerm.total += fHausRes;
	           vConfidenceMap[iOneGridIdx].qualTerm.num += 1.0; 
	           vConfidenceMap[iOneGridIdx].qualTerm.means = vConfidenceMap[iOneGridIdx].qualTerm.total / 
	                                                        vConfidenceMap[iOneGridIdx].qualTerm.num;
	        }//end if  vConfidenceMap[iOneGridIdx].label == 1 || vConfidenceMap[iOneGridIdx].label == 3
	    }//end for i = 0; i!= vMeasuredGridIdx.size(); ++i

	    vConfidenceMap[iOneSlctIdx].qualTerm.seletedflag = true;
        //vConfidenceMap[iOneSlctIdx].selectedFlag = true;

    }//end for is

}


/*************************************************
Function: BoundaryTerm
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
       GaussianKernel
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   oRobotPoint - the location of the robot  
	   vNearbyGridIdxs - the neighboring grids based on the input robot location
	   vTravelCloud - the travelable point clouds (the ground point clouds)
	   vGridTravelPsIdx - the index of point within each grid to total travelable point clouds  
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
//std::vector<float> Confidence::BoundaryTerm(PCLCloudXYZ & vTravelCloud, PCLCloudXYZ & vBoundCloud, pcl::PointXYZ & oRobotPoint){
//	
//	//new output
//	std::vector<float> vBoundRes;
//	if (!vBoundCloud.points.size()) {
//		vBoundRes.resize(vTravelCloud.points.size(), ROBOT_AFFECTDIS);
//		return vBoundRes;
//	}
//	std::cout << "bound points: " << vBoundCloud.points.size() << std::endl;
//	//preparation
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pBoundKDCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i != vBoundCloud.points.size(); ++i)
//		pBoundKDCloud->points.push_back(vBoundCloud.points[i]);
//
//	//new a kdtree for query point clouds
//	pcl::KdTreeFLANN<pcl::PointXYZ> oBoundKDTree;
//	oBoundKDTree.setInputCloud(pBoundKDCloud);
//
//	//
//	for (int i = 0; i < vTravelCloud.points.size(); ++i) {
//
//		//
//		std::vector<int> vSearchedIdx;
//		std::vector<float> vSearchedDis;
//		//
//		oBoundKDTree.nearestKSearch(vTravelCloud.points[i], 1, vSearchedIdx, vSearchedDis);
//		//
//		//if (vSearchedDis[0] > 5.0)
//		//	vSearchedDis[0] = 5.0;
//
//		vBoundRes.push_back(vSearchedDis[0]);
//
//	}//end i
//
//	return vBoundRes;
//
//}


/*************************************************
Function: FrontierTerm
Description: the function is to compute the frontier feature, which is popular in roboticmethod
Calls: none
       none
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   iQueryGrid - the grid at which the robot right now is
	   vNearbyGridIdxs - the neighboring grids index based on the robot grid
Output: change the confidence value about frontier part
Return: none
Others: none
*************************************************/
//void Confidence::FrontierTerm(std::vector<ConfidenceValue> & vConfidenceMap, 
//	                                            const int & iQueryGrid,
//	                           const std::vector<int> & vNearbyGridIdxs){
//
//	//variables
//	float fBoundaryRes = 0.0;
//	int iUnkownCount = 0;
//
//	//if it is a ground region
//	if (vConfidenceMap[iQueryGrid].label == 2) {
//
//		for (int k = 0; k != vNearbyGridIdxs.size(); ++k) {
//			//count its neighboring unknown grids
//			if (!vConfidenceMap[vNearbyGridIdxs[k]].bKnownFlag) {
//				iUnkownCount++;
//			}
//
//		}//end for k
//
//	}//end if vConfidenceMap
//
//	//it has a high value if it is far away from the boundary 
//	if (!iUnkownCount)
//		fBoundaryRes = 1.0;
//
//	vConfidenceMap[iQueryGrid].boundary = fBoundaryRes;
//
//}



/*************************************************
Function: ComputeTotalCoffidence
Description: the function is to compute the total confidence value (totalValue in ConfidenceValue)
Calls: none
Called By: OcclusionTerm()
           BoundTerm()
           DistanceTerm()
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - confindence variance 
	   iQueryIdx - one query grid
Output: u
Return: update the total confidence value
Others: none
*************************************************/
void Confidence::ComputeTotalCoffidence(std::vector<ConfidenceValue> & vConfidenceMap, 
	                                                           const int & iQueryIdx){

    //get maximum value of distance term
    //float fTotalVal =  m_fDisWeight * vDisPartValue[i] 
    //                + m_fBoundWeight * vBoundPartValue[i] 
    //                + m_fExploreWeight * LinearKernel(vConfidenceMap[iQueryIdx].visibility, m_fVisTermThr);

    float fTotalVal =  m_fTraversWeight * m_fDisWeight * vConfidenceMap[iQueryIdx].travelTerm 
    	             + m_fTraversWeight * m_fBoundWeight * vConfidenceMap[iQueryIdx].boundTerm
                     + m_fExploreWeight * vConfidenceMap[iQueryIdx].visiTerm.value;

    //total value
    vConfidenceMap[iQueryIdx].totalValue = fTotalVal;

}


/*************************************************
Function: RegionGrow
Description: this function is to find the reachable grid based on current robot location
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vConfidenceMap - the confidence map (grid map)
	   vNearbyGridIdxs - the nearby grids 
	   oExtendGridMap - grid map for searching
	   iNodeTimes -  the current times of node generations
Output: label the reachable grid if possible
Return: none
Others:  //**status of grid in region grow**
	     //-1 indicates it is an unknown grid
	     //0 indicates this grid is ground but not reachable now
         //1 indicates this grid is a travelable region grid
	     //2 is the new scanned ground grids (input) without growing calculation (in this time)
	     //3 indicates the grid has been computed
	     //4 indicates this grid is a off groud grid (not reachable forever)
*************************************************/
void Confidence::RegionGrow(std::vector<ConfidenceValue> & vConfidenceMap,
	                        const std::vector<MapIndex> & vNearbyGridIdxs,
								        const ExtendedGM & oExtendGridMap,
								                   const int & iNodeTimes){

    //the grid need grow in MAP index (No neighboorhood local index)
	std::vector<int> vNeedGrowGridIdx;
	
	for(int i = 0; i != vNearbyGridIdxs.size(); ++i){
        //the current grid index
		int iCurGridIdx = vNearbyGridIdxs[i].iOneIdx;

        //in fact the node count is nothing to do with region grow
        //it is here so computing can have one less cycle
		if(vConfidenceMap[iCurGridIdx].nodeCount < 0)
           vConfidenceMap[iCurGridIdx].nodeCount = iNodeTimes;

        //find region grow candidates
		//if it is a ground grid
		if(vConfidenceMap[iCurGridIdx].label == 2){
			//if this grid has not been covered by boundary
			if(vConfidenceMap[iCurGridIdx].travelable < 1){
		       vConfidenceMap[iCurGridIdx].travelable = 2;
               vNeedGrowGridIdx.push_back(iCurGridIdx);
		    }//end if
			//else
			//   vConfidenceMap[vNearbyGridIdxs[i]].travelable = 4;//off-ground
		}
	}

    //if nothing to grow
	if(!vNeedGrowGridIdx.size())
		return;

    //check each input grid
	for(int i = 0; i != vNeedGrowGridIdx.size(); ++i){

		//current seed index
	    int iCurIdx;
		//inital result as a non-touchable grid
		int iGrowRes = 0;
		
		//seeds
	    std::vector<int> vSeeds;  
		std::vector<int> vSeedHistory;
		//get one input grid as seed
		vSeeds.push_back(vNeedGrowGridIdx[i]);
        
        //if this one still has not been grown after calculation of other grids
		if(vConfidenceMap[vNeedGrowGridIdx[i]].travelable == 2){
            //growing
	        while(!vSeeds.empty()){

		        //choose a seed (last one)
		        iCurIdx = *(vSeeds.end()-1);
		        //delete the last one
		        vSeeds.pop_back();

			    //record the history of seeds
			    vSeedHistory.push_back(iCurIdx);

			    //label this seed as it has being considered
				vConfidenceMap[iCurIdx].travelable = 3;

				//find the nearboring grids
				//std::vector<int> vGrowNearGridIdx = SearchGrids(iCurIdx, 0.5);
				std::vector<int> vGrowNearGridIdx;
				ExtendedGM::CircleNeighborhood(vGrowNearGridIdx,
						                       oExtendGridMap.m_oFeatureMap, 
											   oExtendGridMap.m_vGrowSearchMask,
		                                       iCurIdx);

				//check neighboring grids
				for (int i = 0; i != vGrowNearGridIdx.size(); ++i) {
					//if the near grid is new input
					if (vConfidenceMap[vGrowNearGridIdx[i]].travelable == 2 &&
						vConfidenceMap[vGrowNearGridIdx[i]].label == 2)
						vSeeds.push_back(vGrowNearGridIdx[i]);

					//the near grid is reachable thereby the query ground grids must be reachable too
					if (vConfidenceMap[vGrowNearGridIdx[i]].travelable == 1 &&
						vConfidenceMap[vGrowNearGridIdx[i]].label == 2)
						iGrowRes = 1;
				}//end for int i = 0;i!=vGrowNearGridIdx.size();++i
				 
		    }//end while

		    //assignment as a touchable grid or ioslated grids
			for(int i = 0; i != vSeedHistory.size(); ++i)
			    vConfidenceMap[vSeedHistory[i]].travelable = iGrowRes;
			
        }//end if vConfidenceMap[vGrowNearGridIdx[i]].travelable == 2
			
	}//end for


}


/*************************************************
Function: CheckIsNewScannedGrid
Description: this function is to find the reachable grid based on current robot location
Calls: none
Called By: FindLocalMinimum()
Table Accessed: none
Table Updated: none
Input: iCurrNodeTime - the current times of node generations, which is equal to iNodeTimes in other functions 
	   vConfidenceMap - the confidence map (grid map)
	   iQueryIdx - the query gird index
Output: check the grid is a new scanning ground grid or not
Return: a binary result, true value indicates the grid is new scanning ground grid
Others: none
*************************************************/
inline bool Confidence::CheckIsNewScannedGrid(const int & iCurrNodeTime, 
	                                          const std::vector<ConfidenceValue> & vConfidenceMap,
	                                          const int & iQueryIdx){
    
    //it should be a ground grid at first
	if(vConfidenceMap[iQueryIdx].label != 2)
		return false;

    //it then should be a reachable groud grid
	if(vConfidenceMap[iQueryIdx].travelable != 1)
		return false;

	//it also should be a new scanned grid (in new scaning trip)
	if(vConfidenceMap[iQueryIdx].nodeCount != iCurrNodeTime)
		return false;

    return true;

}

/*************************************************
Function: FindLocalMinimum
Description: this function is to find the LOCAL minimum value of confidence map 
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vNodeIdxs - node grid index (each grid would most have one node)
	   vNodeClouds - the positions of nodes
	   vConfidenceMap - the confidence map to generate node
	   oExtendGridMap - grid map
	   iCurrNodeTime - the current node times
Output: nodes in local minimum value (by given a radius)
Return: none
Others: none
*************************************************/
void Confidence::FindLocalMinimum(std::vector<int> & vNodeIdxs,
	                              std::vector<pcl::PointXYZ> & vNodeClouds,
	                              const std::vector<ConfidenceValue> & vConfidenceMap,
								  const ExtendedGM & oExtendGridMap,
	                              const int & iCurrNodeTime){

	//define candidate variables
	std::vector<int> vMinCandidates;
	std::vector<bool> vMinCandidStatus(vConfidenceMap.size(),true);

	//search each grid to construct a candidate node extraction region
	for (int i = 0; i != vConfidenceMap.size(); ++i) {
			//if it is a reachable ground grid (some initial reachable grids are not the ground grids)

		if(CheckIsNewScannedGrid(iCurrNodeTime, vConfidenceMap, i)){

            //if it is small
			if (vConfidenceMap[i].totalValue < m_fMinNodeThr){
		        //get this grid into candidate vector
				vMinCandidates.push_back(i);
			}//end if vConfidenceMap[i].totalValue < m_fMinThreshold

		}//end CheckIsNewScannedGrid(iCurrNodeTime, vConfidenceMap, i)

	}//end for i

	//Traversal each candidate grid
	for (int i = 0; i != vMinCandidates.size(); ++i) {
		
		int iCurIdx = vMinCandidates[i];
		//if the candidate grid has not been removed
		if (vMinCandidStatus[iCurIdx]) {
			//find neighboring grid
			std::vector<int> vNeighborGrids;
			ExtendedGM::CircleNeighborhood(vNeighborGrids,
						                   oExtendGridMap.m_oFeatureMap, 
										   oExtendGridMap.m_vNodeMadeMask,
		                                   iCurIdx);
			
			//search each neighboring grid
			for (int j = 0; j != vNeighborGrids.size(); ++j) {
				//the contrastive grids must be synchronous
				if(CheckIsNewScannedGrid(iCurrNodeTime, vConfidenceMap, vNeighborGrids[j])){
					
				   //do not compare with itself(query grid)
				   if (iCurIdx != vNeighborGrids[j]) {
					   //compare the total value
					   if (vConfidenceMap[iCurIdx].totalValue <= vConfidenceMap[vNeighborGrids[j]].totalValue)
					       vMinCandidStatus[vNeighborGrids[j]] = false;
				       else 
					       vMinCandidStatus[iCurIdx] = false;
			       }//if != vNeighborGrids[j]

				}//end CheckIsNewScannedGrid(iCurrNodeTime, vConfidenceMap, vNeighborGrids[j])

			}//end for j

		}//end if (vMinCandidates[vMinCandidates[i]])

	}//end i != vConfidenceMap.size()
	
	//define output
	vNodeIdxs.clear();
    vNodeClouds.clear();

    //assign to each candidate grid
	for (int i = 0; i != vMinCandidates.size(); ++i){
        //it is the local minimum value
		if (vMinCandidStatus[vMinCandidates[i]]){
            //get grid index
			vNodeIdxs.push_back(vMinCandidates[i]);
		    //get corresponding ground point
			pcl::PointXYZ oGridPoint;
			ExtendedGM::OneDIdxtoPoint(oGridPoint, vMinCandidates[i], oExtendGridMap.m_oFeatureMap);

            vNodeClouds.push_back(oGridPoint);
		
        }//end if
	}//end for

}

/*************************************************
Function: Normalization
Description: normalize the input feature vector
Calls: none
Called By: major function
Table Accessed: none
Table Updated: none
Input: vFeatures - a feature (feature value of each grid or feature value of each point)
Output: change the feature value and limit it between 0 and 1
Return: none
Others: 0 <= vFeatures[i] <= 1
*************************************************/
void Confidence::Normalization(std::vector<float> & vFeatures){

	//maximam and minimam
	float fMaxValue = -FLT_MAX;
	float fMinValue = FLT_MAX;

	//find the maximun and minmum value
	for (int i = 0; i != vFeatures.size(); ++i) {
	
		if (vFeatures[i] > fMaxValue)
			fMaxValue = vFeatures[i];
		if (vFeatures[i] < fMinValue)
			fMinValue = vFeatures[i];
	}

	//Normalization
	for (int i = 0; i != vFeatures.size(); ++i) 
		vFeatures[i] = (vFeatures[i] - fMinValue) / (fMaxValue - fMinValue);
	
}



/*************************************************
Function: OutputOcclusionClouds
Description: output a point cloud for debug and test
Calls: all member functions
Called By: OcclusionTerm()
Table Accessed: none
Table Updated: none
Input: vCloud - the test point cloud to be observed
	   vVisableRes - the visibility result of each point
	   viewpoint - the viewpoint position
Output: a point cloud in txt file
Return: none
Others: none
*************************************************/

void Confidence::OutputOcclusionClouds(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
	                                            const std::vector<bool> & vVisableRes,
	                                                  const pcl::PointXYZ & viewpoint){

	std::stringstream sOutPCName;

    //set the current time stamp as a file name
    //full name 
    sOutPCName << "/home/ludy/PC_" << ros::Time::now() << ".txt"; 

    //output file
    std::ofstream oPointCloudFile;
    oPointCloudFile.open(sOutPCName.str(), std::ios::out | std::ios::app);


    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        oPointCloudFile << vCloud.points[i].x << " "
                        << vCloud.points[i].y << " "
                        << vCloud.points[i].z << " " 
                        << vVisableRes[i] << " "
                        << std::endl;
    }//end for         
    //write the viewpoint
    oPointCloudFile << viewpoint.x << " " << viewpoint.y << " " 
                    << viewpoint.z << " " << 2 << " " << std::endl;

    oPointCloudFile.close();

}



/*************************************************
Function: OutputQualityClouds
Description: this function is to check whether the quality measured result is right
Calls: all member functions
Called By: QualityTerm()
Table Accessed: none
Table Updated: none
Input: vCloud - the test point clouds present the raw measuring points
	   fHausRes - the corresponding Hausdorff measured results
Output: a point cloud in txt file
Return: none
Others: none
*************************************************/
void Confidence::OutputQualityClouds(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
	                                                       const float & fHausRes){

	std::stringstream sOutPCName;

    //set the current time stamp as a file name
    //full name 
    sOutPCName << "/home/ludy/Q" << ros::Time::now()<<"_"<< fHausRes << ".txt"; 

    //output file
    std::ofstream oPointCloudFile;
    oPointCloudFile.open(sOutPCName.str(), std::ios::out | std::ios::app);


    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        oPointCloudFile << vCloud.points[i].x << " "
                        << vCloud.points[i].y << " "
                        << vCloud.points[i].z << " " 
                        << std::endl;
    }//end for         

    oPointCloudFile.close();

}




}/*namespace*/


