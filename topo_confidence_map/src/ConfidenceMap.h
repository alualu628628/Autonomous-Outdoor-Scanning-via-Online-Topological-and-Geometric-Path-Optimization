#ifndef CONFIDENCEMAP_H 
#define CONFIDENCEMAP_H 


//#include "HausdorffMeasure.h"

#include "GHPR.h"
#include "ExtendedGridMap.h"
#include "HausdorffMeasure.h"

#include <stdlib.h>
#include <time.h> 


///************************************************************************///
// a class to implement the confidence map generation based on point clouds
// created and edited by Huang Pengdi

//Version 1.1 2018.11.25
// - add the 2D map funtion to organize point cloud storage
//Version 1.2 2018.12.13
// - change all calculation under the grid model
// - modify distance features
//Version 1.3 2018.12.14
// - add noting
// - complete distance term
//version 2.0 2019.04.12
// - real time processing version
///************************************************************************///


namespace topology_map {


//visible measured value
//a variable type of visibility
struct Visible{

	float visibletimes;//the number of times that this point is visible in observation
	float totaltimes;//total observation times
	float value;
    
    //initialization
	Visible(){
		visibletimes = 0.0;
		totaltimes = 0.0;
		value = 0.0;
	}
};

//quality measured value
//a variable type of quality
struct Quality{

    float means;//
	float total;//total measured value
	float num;//total computed time
	bool seletedflag;//whether a point is selected 

    //initialization
    Quality(){

        means = 0.0;
	    total = 0.0;
	    num = 0.0;
	    seletedflag = false;

    }

};


//grid status
//a variable type of confidence feature
struct ConfidenceValue{

	//distance based term
	float travelTerm;
	//visibility based term
	float boundTerm;
	//bound term
	Visible visiTerm;
	//quality
	Quality qualTerm;
	//quality
	float totalValue;

	//************details of grid label value in m_vMapGridLabel
    // cover means rewrite if new sematic object appears in this grid
    // 0 nothing or unknown (0 is covered by 1)
    // 1 obstacles (1 is covered by 2)
    // 2 ground points (2 is covered by 3)
    // 3 boundary 
    // for example, boundary grid can not be defined as another classification, but obstacle can be recovered by ground or obstacles 
    // the "-" negetive means the grid has been computed in node generation
	short label;

    //************details of grid status value in m_vMapGridTravel
    //status indicates whether the gird is travelable or not (can the robot reaches this grid right now)
    // -1 indicates it is an unknown grid
    // 0 indicates this grid is ground but not reachable now
    // 1 indicates this grid is a travelable region grid
    // 2 is the new scanned grids (input) without growing
    // 3 indicates the grid has been computed
    // 4 indicates this grid is a off groud grid (not reachable forever)
	short travelable;

	//quality computed flag
	bool qualFlag;
	//minimum computed flag
	short nodeCount;
	//center point of a map grid 
	pcl::PointXYZ oCenterPoint;

	//constructor
	ConfidenceValue() {

		travelTerm = 0.0;//start with 1, which means no need to go there
		boundTerm = 0.0;//start with 1, which means no need to go there
		//visiTerm = 1.0;
		//qualTerm = 0.0;
		totalValue = 0.0;//initial each grid as not need to move there
		label = 0;//start with nothing
		travelable = -1;//start with unknown
		nodeCount = -1;	//start with undone	
		qualFlag = true;//start with undone
		oCenterPoint.x = 0.0;//start from 0, which will be re-define in InitializeGridMap
		oCenterPoint.y = 0.0;//start from 0
		oCenterPoint.z = 0.0;//start from 0

	};

};


//a class computing confidence value of each map grid
//there is a lot of geometrical features in it
//it depends on pcl and grid_map lib
class Confidence {

	typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;
	typedef pcl::PointCloud<pcl::PointXYZI> PCLCloudXYZI;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloudXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLCloudXYZIPtr;

public:

	//constructor
	Confidence(float f_fSigma,
		       float f_fGHPRParam = 4.2,
		       float f_fVisTermThr = 5,
		       float f_fMinNodeThr = 0.2);
	
	//destructor
	~Confidence();

	//set the affect radius of Gaussian Kernel
	void SetSigmaValue(const float & f_fSigma);

	//set visibility term related paramters
	void SetVisTermThr(const float & f_fVisTermThr);

    //set the weight of each terms
	void SetTermWeight(const float & f_fTraversWeight,
	                   const float & f_fExploreWeight,
	                   const float & f_fDisWeight,
	                   const float & f_fBoundWeight);
    void SetTermWeight(const float & f_fTraversWeight,
	                   const float & f_fDisWeight);

    //set node minimum threshold
	void SetNodeGenParas(const float & f_fMinNodeThr);

	//output m_fMinNodeThr
	float OutNodeGenParas();

	//*******************Mathematical Formula Part********************
	
	inline float VectorInnerProduct(const pcl::PointXYZ & oAVec,
		                            const pcl::PointXYZ & oBVec);
	
	//Gaussian Kernel Function
	inline float GaussianKernel(const pcl::PointXYZ & oQueryPo,
		                       const pcl::PointXYZ & oTargerPo,
		                                         float & sigma);
	//linear Kernel Function
	inline float LinearKernel(const float & fTargetVal,
	                        	const float & fThrVal);

	//variance
	inline float StandardDeviation(const PCLCloudXYZ & vCloud);

	//density
	inline float ComputeDensity(const PCLCloudXYZ & vCloud,
								int iSampleTimes = 5,
								bool bKDFlag = true);

	//the 2 norm of a vector
	inline float Compute2Norm(const pcl::PointXYZ & oQueryPo,
		                     const pcl::PointXYZ & oTargerPo);

	//the 1 norm of a vector
	inline float ComputeSquareNorm(const pcl::PointXYZ & oQueryPo,
		                           const pcl::PointXYZ & oTargerPo);

	//compute center
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud,
		                       const std::vector<int> & vPointIdx);
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud);

	//get random value
	inline std::vector<int> GetRandom(const unsigned int iSize,
		                                 const int iSampleNums);
	//static std::vector<int> GetRandom(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllTravelCloud,
	//	                                                                 GridMap & oMaper,
	//	                                                              const int iSampleNums);

	//Compute Euclidean distance
	static float ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, 
		                            pcl::PointXYZ & oTargetPoint);


	//*******************Feature Term Part********************
	//1. the distance term of confidence map
	//generate the distance feature map of a neighboring grids
	void DistanceTerm(std::vector<ConfidenceValue> & vConfidenceMap,
		                          const pcl::PointXYZ & oRobotPoint,
                           const std::vector<int> & vNearGroundIdxs,
	                               const PCLCloudXYZ & vGroundCloud);


	//2. compute the boundary feature of neighboorhood
	void BoundTerm(std::vector<ConfidenceValue> & vConfidenceMap,
                        const std::vector<int> & vNearGroundIdxs,
	                         const PCLCloudXYZPtr & pGroundCloud,
	                          const PCLCloudXYZPtr & pBoundCloud);


	//3. Compute the occlusion
	void OcclusionTerm(std::vector<ConfidenceValue> & vConfidenceMap,
	                                  PCLCloudXYZPtr & pNearAllCloud,
	                        const std::vector<int> & vNearGroundIdxs,
	                            const pcl::PointXYZ & oPastViewPoint,
	                                          const int & iNodeTimes);
	

	//2. quality term of confidence map
	//void QualityTermUsingDensity(std::vector<ConfidenceValue> & vConfidenceVec,
	//	                          const std::vector<int> & vNearByIdxs,
	//	                                 const PCLCloudXYZ & vTravelCloud,
	//               const std::vector<std::vector<int>> & vGridTravelPsIdx,
	//	                               const PCLCloudXYZ & vAllBoundCloud,
	//	            const std::vector<std::vector<int>> & vGridBoundPsIdx,
	//	                               const PCLCloudXYZ & vObstacleCloud,
	//	              const std::vector<std::vector<int>> & vGridObsPsIdx);

	//4. quality term of confidence map
	void QualityTerm(std::vector<ConfidenceValue> & vConfidenceMap,
		                     const PCLCloudXYZPtr & pObstacleCloud,
                           const std::vector<int> & vObstNodeTimes,
            const std::vector<std::vector<int> > & vObstlPntMapIdx,
		                         const ExtendedGM & oExtendGridMap,
		                 const std::vector<MapIndex> & vNearByIdxs,
                                             const int & iNodeTime,
                                                  int iSmplNum = 5);


	//4. Compute the boundary item
	//std::vector<float> BoundaryTerm(PCLCloudXYZ & vTravelCloud, PCLCloudXYZ & vBoundCloud, pcl::PointXYZ & oRobotPoint);
	
	//5. Compute the frontier item (constract method)
	//void FrontierTerm(std::vector<ConfidenceValue> & vConfidenceVec, const int & iQueryGrid, const std::vector<int> & vNearByIdxs);

	//6. Compute the total coffidence value
	void ComputeTotalCoffidence(std::vector<ConfidenceValue> & vConfidenceMap, 
	                                                   const int & iQueryIdx);


	//region grow to obtain travelable region (travelable region is the ground which can be touch from current location)
	void RegionGrow(std::vector<ConfidenceValue> & vConfidenceMap,
	                const std::vector<MapIndex> & vNearbyGridIdxs,
								const ExtendedGM & oExtendGridMap,
								           const int & iNodeTimes);

    //check whether the grid is a newest scanned travelable ground grid
	bool CheckIsNewScannedGrid(const int & iCurrNodeTime, 
	                           const std::vector<ConfidenceValue> & vConfidenceMap,
	                           const int & iQueryIdx);

	//non-minimum suppression
    void FindLocalMinimum(std::vector<int> & vNodeIdxs,
	                      std::vector<pcl::PointXYZ> & vNodeClouds,
	                      const std::vector<ConfidenceValue> & vConfidenceMap,
						  const ExtendedGM & oExtendGridMap,
	                      const int & iCurrNodeTime);

	//normalization of features
	static void Normalization(std::vector<float> & vFeatures);

    //output occlusion point clouds for test
	void OutputOcclusionClouds(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
	                                   const std::vector<bool> & vVisableRes,
	                                         const pcl::PointXYZ & viewpoint);

	void OutputQualityClouds(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
	                                                const float & fHausRes);

	//*******************Public Data Part********************
	pcl::PointXYZ oShowCenter;
	pcl::PointXYZ oShowRobot;

private:

	//sigma parameter of Gaussian function in GaussianKernel()
	float m_fSigma;
	
	//visibility term based paramters
	float m_fVisTermThr;///<the threshold of visibility term

	//weighted of each term for total confidence value
	float m_fTraversWeight;
	float m_fExploreWeight;

	//weighted of each term for total confidence value
	float m_fDisWeight;
	float m_fBoundWeight;

    //node generation
	float m_fMinNodeThr;

};



}/*namespace*/


#endif