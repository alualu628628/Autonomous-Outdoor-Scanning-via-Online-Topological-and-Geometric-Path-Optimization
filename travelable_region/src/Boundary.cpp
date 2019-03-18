#include "Boundary.h"

/*************************************************
Function: Boundary
Description: Constructor of Boundary Class
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: None
Output: initialize some necessary parameters
Return: none
Others: none
*************************************************/

Boundary::Boundary():m_pGroundCloud(new pcl::PointCloud<pcl::PointXYZ>),
                     m_pObstacleCloud(new pcl::PointCloud<pcl::PointXYZ>),
	                 m_iSmplNum(3),
	                 m_iNoiseNum(3),
                     m_fHeigtThr(0.2),
	                 m_fRadius(0.5){



}

/*************************************************
Function: ~Boundary
Description: destructor of INSAC Class
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
Boundary::~Boundary(){

	//Theoretically, boost pointers are automatically destroyed

}

/*************************************************
Function: SetSampleNum
Description: set the number of sample (jump the same number of point)
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: f_iSmplNum a given number of sample
Output: number of down sampling during reading ground point clouds 
Return: none
Others: none
*************************************************/

void Boundary::SetSampleNum(const int & f_iSmplNum){

	m_iSmplNum = f_iSmplNum;

}

/*************************************************
Function: SetHeigtThreshold
Description: set height different value between a query ground point and a obstacle point 
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: f_fHeigtThr a given height different value
Output: height different value in real obstacle judgement
Return: none
Others: none
*************************************************/

void Boundary::SetHeigtThreshold(const float & f_fHeigtThr){

	m_fHeigtThr = f_fHeigtThr;

}

/*************************************************
Function: SetKDRadius
Description: set the searching radius in kd tree of obstacle points
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: f_fRadius a given searching radius
       cost time of processing increases when radius increases
Output: seeds have not been selected
Return: none
Others: none
*************************************************/

void Boundary::SetKDRadius(const float & f_fRadius){

	m_fRadius = f_fRadius;

}

/*************************************************
Function: SetNoiseNum
Description: set the point that is belong to noise in a local region 
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: f_iNoiseNum a given potential noise presents as some points
Output: the number of noise point
Return: none
Others: none
*************************************************/
void Boundary::SetNoiseNum(const int & f_iNoiseNum){

	m_iNoiseNum = f_iNoiseNum;

}

/*************************************************
Function: ComputeBoundary
Description: calculate the boundary point between the obstacle and ground points
Calls: NearBoundary
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: none (but it should have the priori obstacle points and ground points)
Output: m_vBoundIdx the boundary point index of ground points
Return: none
Others: none
*************************************************/

void Boundary::ComputeBoundary(){

	//new a kdtree for point clouds which is to be queryed
	pcl::KdTreeFLANN<pcl::PointXYZ> oObstKDTree;
	oObstKDTree.setInputCloud(m_pObstacleCloud);

	//to each query point (ground point after down sampling)
	for (int i = 0; i < m_pGroundCloud->points.size(); ++i) {
	
		//temp
		std::vector<int> vSearchedIdx;
		std::vector<float> vSearchedDis;
		//searching with the given radius		
		if(oObstKDTree.radiusSearch(m_pGroundCloud->points[i], m_fRadius, vSearchedIdx, vSearchedDis)){
			//indeed nearby the obstacle
			int iRealObsCount = 0;
			for (int j = 0; j != vSearchedIdx.size(); ++j) {
				//judge weather the query point is a boundary point 
				//judge the height different in order to avoid some false obstacle (ground) extaction
				if (NearBoundary(m_pGroundCloud->points[i].z, m_pObstacleCloud->points[vSearchedIdx[j]].z)) {
					//save the boundary point
					iRealObsCount++;
					//if it is really near a obstacle rather than a noise point
					if (iRealObsCount > m_iNoiseNum){
					    m_vBoundIdx.push_back(m_vGroundIdx[i]);
					    //to the next query point
					    break;
				             }//if m_iNoiseNum
				}//end if NearBoundary	
			}//end for j
		}//end if radius 

	}//end i

}

/*************************************************
Function: NearBoundary
Description: process the high different of two given value 
Calls: None
Called By: ComputeBoundary
Table Accessed: none
Table Updated: none
Input: fQuery and fTarget,two high value
Output: indeed higher than query point or not 
Return: bool 1 is yes, 0 is not high enough
Others: none
*************************************************/
bool Boundary::NearBoundary(const float & fQuery,const float & fTarget){

             //attention the fabs is for float value
	//abs function causes a mistake since it loses data precision
	return fabs(fQuery - fTarget) > m_fHeigtThr ? 1 : 0;

}

/*************************************************
Function: GetSegmentClouds
Description: input function, to read the ground and obstacle points
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: vCloud whole point cloud (one frame of point clouds)
Output: m_pGroundCloud ground point clouds
        m_vGroundIdx corresponding index
        m_pObstacleCloud obstacle point clouds
Return: none
Others: none
*************************************************/
void Boundary::GetSegmentClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud,
	                            const std::vector<int> & vResults) {


	//clear all data
	m_pGroundCloud->clear();
	m_pObstacleCloud->clear();
	m_vGroundIdx.clear();
	
	int iGroundCount = 0;
	//to each point
	for (int i = 0; i != vResults.size(); ++i) {
	    //find either ground or obstacle point
		if (vResults[i] == 1 ) {//ground
		    iGroundCount++;
			if(!(iGroundCount%m_iSmplNum)){//sampling if necessary
			   //record the point
			   m_pGroundCloud->points.push_back(vCloud->points[i]);
			   m_vGroundIdx.push_back(i);
			}
	    // vCloud->points[i].z < 0 is want to minimize the point set
	    // the obstacle points below the laser scanner are selected  
		//}else if (vResults[i] == -1 && vCloud->points[i].z < 0) {//obstacle
		}else if (vResults[i] == -1) {//obstacle
		    //record the point
			m_pObstacleCloud->points.push_back(vCloud->points[i]);

		}//end else

	}//end for i

}

/*************************************************
Function: OutputBoundClouds
Description: output the boundary points (results)
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: vCloud whole point clouds (one frame point clouds)
Output: vBoundCloud (final boundary points)
Return: none
Others: none
*************************************************/
void Boundary::OutputBoundClouds(pcl::PointCloud<pcl::PointXYZ> & vBoundCloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud) {

	vBoundCloud.clear();
	
	//output the boundary point clouds
	for (int i = 0; i != m_vBoundIdx.size(); ++i) {
	
		vBoundCloud.points.push_back(vCloud->points[m_vBoundIdx[i]]);

	}//end for i

}

/*************************************************
Function: OutputBoundClouds
Description: reload OutputBoundClouds as ouputting indexes instead of points
Calls: None
Called By: Major function maybe
Table Accessed: none
Table Updated: none
Input: none
Output: vCloudLabel (label of each points) boundary point is labelled as 2
Return: none
Others: none
*************************************************/
void Boundary::OutputBoundClouds(std::vector<int> & vCloudLabel) {

	//output the boundary point clouds
	for (int i = 0; i != m_vBoundIdx.size(); ++i) {

		vCloudLabel[ m_vBoundIdx[i] ] = 2;

	}//end for i

}