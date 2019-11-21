#ifndef HAUSDORFFMEASURE_H 
#define HAUSDORFFMEASURE_H

#include <cmath>

#include <cfloat>

#include <pcl/point_types.h>

#include <pcl/point_cloud.h>

///************************************************************************///
// This code is to measure Hausdorff dimension on a given points
//
// created and edited by Huang Pengdi, 2018.12.27
// Email: alualu628628@gmail.com
///************************************************************************///
namespace topology_map {

//a structe records the non-emptys voxels number and its corresponding measured scale
//scale is in meter
//this structure is mainly designed for linear fitting method
struct HitsInScale{
	//the value of current scale
	float boxScale;
	//thenon-empty boxes/voxels number 
	float boxNum;
};

///************************************************************************///
// a class to implement the box counting method, which is to compute the Hausdorff dimension
// the measuring result of a point clouds is between 0 and 3 in a 3D space.
// The implementation is mainly based on a reverse searching, which is like reverse octree
// This code can also calculate the Hausdorff dimension and related dimensions

//Version 1.0 2018.12.27
// - created
//Version 1.0 2018.12.30
// - modified functions and remove something others
//Version 1.1 2019.1.1
// - add the notes

///************************************************************************///

class HausdorffDimension{

public:

    //*************Initialization function*************
	//constructor
	HausdorffDimension(int f_iIterMax=3,
		               int f_iIterMin=1);

	//destructor
	~HausdorffDimension();
	
	//*************Parameter setting function*************
	//set the generalized dimensional parameter q, where q = 0 is a normal Hausdorff dimension 
	void SetParaQ(int f_iParaQ);//where q = 2 is a related dimension

	//initial the iteration times (upper and lower limits)
	void InitialLoopParams(const int & f_iIterMax,
	                       const int & f_iIterMin);

    //set the measuring scale manually, e.g., from 20 to 100m
	void SetGivenScales(const float & f_fLargeScale,
	                    const float & f_fSmallScale);

	//set the minimum distance
	void SetMinDis(float f_fMinDis);

    //Set a priori value of bounding box corner
	void SetMaxMinCoor(const pcl::PointXYZ & f_oMaxCoor, 
		               const pcl::PointXYZ & f_oMinCoor);

	//extract the edge length 
	bool ExtractEdgeLength(pcl::PointXYZ & oOutLength);

	//clear the member variables which are related to the input data
	void ClearLength();
	
	//clear and reset all of member variables
	void ClearAll();

	//*************performance function*************

	//round a given value
	float Round(const float & fIdx){
		
		return(fIdx > 0.0) ? floor(fIdx + 0.5) : ceil(fIdx - 0.5);
	
	};

	//Least squares linear fitting
    float LinearFitting(std::vector<HitsInScale> & vSpectrum);

	//compute the length of the bounding box
	std::vector<float> GetBoundingLength(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

	//find the maximum value among the input vector
	template <typename T>
	inline T FindMaximum(std::vector<T> vSequence){
		
		//find the maximum among the input datas
		T tMaxValue;
		
		//inital the MaxValue as the first value of input
		if(vSequence.size())	
			tMaxValue = vSequence[0];
		
		//find the maximum value
		for(int i = 0; i != vSequence.size(); ++i){
			
			if(tMaxValue < vSequence[i])
			tMaxValue = vSequence[i];
		}//end for i
		
		return tMaxValue;
	}
	
	//the major function
	//this function is to implement the Hausdorff measure by using the box counting based method
	//the box counting is an approximate solution of Hausdorff measure
	float BoxCounting(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

private:
	
	//The first and last iteration times 
	int m_iIterMax;//
	int m_iIterMin;//

	//a flag denotes whether the corners is known before processing 
	bool m_bMaxMinCoFlag;
	
	//this flag indicates the bounding box construction has been done or not 
	bool m_bEdgeFlag;

	//length of bounding box 
	pcl::PointXYZ m_oMinCoor;//corner
	pcl::PointXYZ m_oMaxCoor;//corner
	pcl::PointXYZ m_oEdgeLength;//length at x, y, z axis, respectively
	float m_fBoundBoxLen;//the maximum length of bounding box						
	const float m_fRedundancy;//it adds a minimum value for m_fBoundBoxLen to prevent bug

	//generalized dimension Q
	float m_fParaQ;

	////the minimum and the maximum measuring scale
	float m_fLargeScale;
	float m_fSmallScale;
	bool m_bScaleFlag;

	//the minimum measuring scale
	float m_fMinDis;
	bool m_bMinDisFlag;

};


}/*namespace*/

#endif

/******************* one example show how to use it *******************
//please insure you have a point cloud input with pcd type (PCL lib is required)
//one example: you can copy it and run it directly

HausdorffDimension oHDor(5,1);

oHDor.SetMinDis(0.1);

oHDor.SetParaQ(0);

std::cout<<"HD: "<<oHDor.BoxCounting(*cloud)<<std::endl;

==============================================*/