#ifndef Boundary_h
#define Boundary_h

#include"INSAC.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>

///************************************************************************///
// a class to obtain the boundary point after ground extraction
//
// 
//
// Generated and edited by Huang Pengdi 2018.11.02
///************************************************************************///

class Boundary{

public:

	//constructor
	Boundary();

	//decosntructor
	~Boundary();

	//*****parameters******
	//set number of sample
	void SetSampleNum(const int & f_iSmplNum);
	
	//set height value
	void SetHeigtThreshold(const float & f_fHeigtThr);

	//set kdtree searching radius
	void SetKDRadius(const float & f_fRadius);

	//set potential point number of noise 
	void SetNoiseNum(const int & f_iNoiseNum);

	//judge the point is near the boundary or not 
	inline bool NearBoundary(const float & fQuery, const float & fTarget);

	//extract boundary point between travable point and obstacle point
	void ComputeBoundary();

	//get the corresponding point clouds based on the segmentation result
	void GetSegmentClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud,
		const std::vector<int> & vResults);
	
	//output boundary point clouds
	void OutputBoundClouds(pcl::PointCloud<pcl::PointXYZ> & vBoundCloud,
		             const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud);

	//reload: output whole point clouds value with boundary value 2
	void OutputBoundClouds(std::vector<int> & vCloudLabel);

private:
	
	unsigned int m_iSmplNum;///<the number of sample 
	unsigned int m_iNoiseNum;///<the number of noise on the ground
	float m_fHeigtThr;///< height value of threshold 
	float m_fRadius;///<searching radius of a query point(ground point)

	//point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pGroundCloud;///<ground point clouds
	//DO NOT output this obstacle point clouds due to its incompleteness
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pObstacleCloud;///<obstacle point clouds
	//index
	std::vector<int> m_vGroundIdx;///<the index of ground points in total point set
	std::vector<int> m_vBoundIdx;///<the index of boundary point in TOTAL point clouds

	//std::vector<int> m_vObstacleIdx;///<the index of obstacle points in all point set

};


#endif