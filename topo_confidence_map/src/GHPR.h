#ifndef GHPR_H
#define GHPR_H
//pcl based
#include <vector>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//flann based
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

//compute the convex hull
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>//important

///************************************************************************///
// a class to implement the GHPR algorithm
// GHPR - Generalized Hidden Point Removal operator
// Katz S., Tal A., On the visibility of point clouds, ICCV, 2015,1350-1358.
// created and edited by Huang Pengdi, 2018.11.03

//Version 1.0 2018.11.03
// - add the implementation of the HPR algorithm
//Version 1.1 2018.12.18
// - add the implementation of the GHPR algorithm

///************************************************************************///

namespace topology_map {


class GHPR{

public:

	//constructor
	GHPR(float f_fParam = 3.8);

	//destructor
	~GHPR();

	//give a value to the m_fParam parameter
	void SetParamExponentInput(float f_fParam);

	//***kernel function***
	//the linear / mirror kernel function
	inline float LinearKernel(const float & fGamma, 
		                      const float & fPointNorm);

	//compute the visiable point cloud set based on the input viewpoint
	std::vector<bool> ComputeVisibility(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
		                                            const pcl::PointXYZ & oViewPoint);

	//find the indices of convex hull
	void FindVisibleIndices(std::vector<bool> & vVisibleRes,
		                    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pTransforCloud,
		                    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pHullCloud);

private:

	//m_fParam >= 1.0, in practice, it will be very large
    //param designed in HPR algorithm, which is published in below 
	//Katz S , Tal A , Basri R . Direct visibility of point sets[J]. ACM Transactions on Graphics, 2007, 26(3):24.
	float m_fParam;///<make the gamma is larger than the maximum distance of point set to viewpoint

};



}



#endif

//*********************an example display how to use this class***********************
////generate a GHPR object
//GHPR oGHPRer(3.2);
//
////compute the visibility of point clouds
//std::vector<int> vVisableIdx = oGHPRer.ComputeVisibility(*pCloud, oViewPoint);
//
////save the result
//std::vector<int> vLabel(pCloud->points.size(), 0);
//for (int i = 0; i != vVisableIdx.size(); ++i) {
//	//label visiable result
//	vLabel[vVisableIdx[i]] = 1;
//}//end for i