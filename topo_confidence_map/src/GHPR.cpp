#include "GHPR.h"


namespace topology_map {


/*************************************************
Function: GHPR
Description: constrcution function for GHPR class
Calls: SetExponentialParam
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: f_fParam - the unique parameter of GHPR algorithm
Output: none
Return: none
Others: none
*************************************************/
GHPR::GHPR(float f_fParam){

	//set the parameter to act on gamma value
	SetParamExponentInput(f_fParam);

}

/*************************************************
Function: ~GHPR
Description: destrcution function for GHPR class
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
GHPR::~GHPR() {

	//nothing

}

/*************************************************
Function: SetParamExponentInput
Description: Set Exponent Type Input to parameter 
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: f_fParam - the exponent of GHPR parameter
Output: none
Return: none
Others: none
*************************************************/
void GHPR::SetParamExponentInput(float f_fParam){

	//logarithm to the base of 10
	//param = 10^param
	m_fParam = pow(10.0f, f_fParam);
	
}

/*************************************************
Function: LinearKernel
Description: the linear Kernel function of GHPR algorithm
Calls: none
Called By: ComputeVisibility
Table Accessed: none
Table Updated: none
Input: fGamma - the gamma parameter of kernel function
       fPointNorm - the point norm (distance)
Output: the distance value indicates the transformation 
Return: a distance value
Others: gamma is larger than the maximum distance from viewpoint to point set
*************************************************/
inline float GHPR::LinearKernel(const float & fGamma, 
	                            const float & fPointNorm) {

	//the linear kernel is f(||p||) = 2 * gamma - ||p||
	return 2.0f * fGamma * m_fParam - fPointNorm;

}



/*************************************************
Function: ComputeVisibility
Description: Compute the visiable points of a point set from a viewpoint
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vCloud - an input point clouds 
       oViewPoint - a viewpoint
Output: none
Return: none
Others: none
*************************************************/
std::vector<bool> GHPR::ComputeVisibility(const pcl::PointCloud<pcl::PointXYZ> & vCloud,
                                                      const pcl::PointXYZ & oViewPoint){

    //define output
	std::vector<bool> vVisiableRes;
    //check there  is enough input
	if(vCloud.points.size() < 3){
		//all points visible if less points 
		vVisiableRes.resize(vCloud.size(),true);
		return vVisiableRes;
	}

	//F(p,C) = C + (p - C) * f(||p - C||)/||p - C||
	//a kernel function input, which must be larger that the maximum distance from the viewpoint to poins set
	float fGamma = -FLT_MAX;

	//*******Transform the point cloud which is to be observed *******

	//a point cloud to save the transfored points
	//new point clouds include raw point set and viewpoint
	std::vector<float> vPointNorms(vCloud.points.size());///<norm of each point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pConvexCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//the viewpoint will be added in the end of calculation
	pConvexCloud->width = vCloud.points.size();
	pConvexCloud->height = 1;
	pConvexCloud->is_dense = false;
	pConvexCloud->points.resize(pConvexCloud->width * pConvexCloud->height);

	//to each point
	//shift the viewpoint to the origin (0,0,0) of coordinate system 
	for(size_t i = 0; i != vCloud.points.size(); ++i){
		//build the local coordinate system at the origin
		pConvexCloud->points[i].x = vCloud.points[i].x - oViewPoint.x;
		pConvexCloud->points[i].y = vCloud.points[i].y - oViewPoint.y;
		pConvexCloud->points[i].z = vCloud.points[i].z - oViewPoint.z;
		//compute the distance from viewpoint to the searched point
		vPointNorms[i] = sqrt(pConvexCloud->points[i].x * pConvexCloud->points[i].x
			                + pConvexCloud->points[i].y * pConvexCloud->points[i].y
		                    + pConvexCloud->points[i].z * pConvexCloud->points[i].z);

		pConvexCloud->points[i].x = pConvexCloud->points[i].x / vPointNorms[i];
		pConvexCloud->points[i].y = pConvexCloud->points[i].y / vPointNorms[i];
		pConvexCloud->points[i].z = pConvexCloud->points[i].z / vPointNorms[i];
		//give the maximum norm to the gamma value 
		if (vPointNorms[i] > fGamma)
			fGamma = vPointNorms[i];

	}

	for (size_t i = 0; i != pConvexCloud->points.size(); ++i) {
	
		//compute the kernel value
		float oKernelValue = LinearKernel(fGamma, vPointNorms[i]);
		//compute the final transfored point set
		pConvexCloud->points[i].x = pConvexCloud->points[i].x * oKernelValue;
		pConvexCloud->points[i].y = pConvexCloud->points[i].y * oKernelValue;
		pConvexCloud->points[i].z = pConvexCloud->points[i].z * oKernelValue;
	
	}

	//dont forget to add the viewpoint at origin!
	pcl::PointXYZ oViewOriginPoint;
	oViewOriginPoint.x = 0.0;
	oViewOriginPoint.y = 0.0;
	oViewOriginPoint.z = 0.0;
	pConvexCloud->points.push_back(oViewOriginPoint);//in the end of point clouds

	//*******Compute the convex hull of transformed point cloud *******
	
	//a object based on pcl convex hull class 
    pcl::ConvexHull<pcl::PointXYZ> oConvexHull;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pChullResCloud(new pcl::PointCloud<pcl::PointXYZ>);

	//input transfored point clouds
	oConvexHull.setInputCloud(pConvexCloud);
    //set 3d convex hull model
	oConvexHull.setDimension(3);
	//compute the 3D convex hull
	oConvexHull.reconstruct(*pChullResCloud);

	//get the indices of convex hull point (visiable point set)
	//call this function below is risk
	//because this function could be called only when the version of pcl is up tp 1.8.0
	//pcl::PointIndices vHullPointIndices;	
	//oConvexHull.getHullPointIndices(vHullPointIndices);

	//In terms of this situation, we use the kdtree (also O(nlogn)) to find the index

	FindVisibleIndices(vVisiableRes,pConvexCloud,pChullResCloud);

	//return the reslut
	return vVisiableRes;

}



/*************************************************
Function: FindHullIndices
Description: find the point indices of constructed convex hull
Calls: none
Called By: ComputeVisibility
Table Accessed: none
Table Updated: none
Input: vVisibleRes - the visible point 
       pTransforCloud - a geometrical transfor point clouds 
       pHullCloud - a point clouds at the convex hull
Output: none
Return: none
Others: none
*************************************************/
void GHPR::FindVisibleIndices(std::vector<bool> & vVisibleRes,
	                          const pcl::PointCloud<pcl::PointXYZ>::Ptr & pTransforCloud,
	                          const pcl::PointCloud<pcl::PointXYZ>::Ptr & pHullCloud) {

	//define output
	//remove the viewpoint from ouput list
	vVisibleRes.clear();
	vVisibleRes.resize(pTransforCloud->points.size() - 1,false);

	int iViewPointIdx = pTransforCloud->points.size() - 1;

	//construct a kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> oTransforTree;
	oTransforTree.setInputCloud(pTransforCloud);


	//find indices using kdtree
	for (size_t i = 0; i != pHullCloud->points.size(); ++i) {

		//define temps
		std::vector<int> vNearestIdx;
		std::vector<float> vNearestDis;
		
		//search the nearest raw point of query constructed convex hull surface point 
		oTransforTree.nearestKSearch(pHullCloud->points[i], 1, vNearestIdx, vNearestDis);

		//if the query point is not the viewpoint
		if (vNearestIdx[0] != iViewPointIdx)
			vVisibleRes[vNearestIdx[0]] = true;

	}//end for i != pHullCloud->points.size()

}




}