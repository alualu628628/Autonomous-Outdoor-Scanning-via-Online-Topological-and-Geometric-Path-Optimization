#include "SectorPartition.h"

/*************************************************
Function: SetSectorNum
Description: a parameter value assigment function in below:
             DivideSector XXX;
             XXX.SetSectorNum(8);
Calls: None
Called By: DivideSector, which is the construction function
Table Accessed: none
Table Updated: none
Input: f_iSectorNum, the given number of sectors
Output: none
Return: none
Others: none
*************************************************/
void DivideSector::SetSectorNum(int f_iSectorNum) {

	m_iSectorNum = f_iSectorNum;

};


/*************************************************
Function: SetOriginPoint
Description: a parameter value assigment function in below:
             DivideSector XXX;
             XXX.SetOriginPoint(vBasePoint);
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: f_oPoint, a 3d point indicates the original point or view point of LiDAR sensor
Output: none
Return: none
Others: none
*************************************************/
void  DivideSector::SetOriginPoint(const pcl::PointXYZ & f_oPoint) {

	m_OriginPoint.x = f_oPoint.x;
	m_OriginPoint.y = f_oPoint.y;
	m_OriginPoint.z = f_oPoint.z;
	//flag up
	m_bOriPointFlag = true;

}


/*************************************************
Function: ComputePointSectorIdxs
Description: a callback function in below:
             DivideSector XXX(8);
             XXX.SetOriginPoint(vRobotCloud->points[0]);
             XXX.ComputePointSectorIdxs(*vOneCloud, oPointSecIdxs);
Calls: CheckTruthPoint
Called By: none
Table Accessed: none
Table Updated: none
Input: vCloud, one frame of point clouds through value passing
       vPointSecIdx, the output
Output: vPointSecIdx, the index of each point in sectors
Return: none
Others: none
*************************************************/
std::vector<std::vector<GroundFeature> > DivideSector::ComputePointSectorIdxs(pcl::PointCloud<pcl::PointXYZ> vCloud,
	std::vector<std::vector<int>> & vPointSecIdx) {

	std::vector<std::vector<GroundFeature> > vGroundFeatures;

	//i hope this situation would not happen
	if (!m_bOriPointFlag) {
		//compute the center point of scanning region as the base point
		for (int i = 0; i != vCloud.points.size(); ++i) {
			//
			m_OriginPoint.x += vCloud.points[i].x;
			m_OriginPoint.y += vCloud.points[i].y;
			m_OriginPoint.z += vCloud.points[i].z;
		}

		m_OriginPoint.x = m_OriginPoint.x / float(vCloud.points.size());
		m_OriginPoint.y = m_OriginPoint.y / float(vCloud.points.size());
		m_OriginPoint.z = m_OriginPoint.z / float(vCloud.points.size());
	}//end if

	 //shift coordinate based on the original point
	for (int i = 0; i != vCloud.size(); ++i) {

		vCloud.points[i].x = vCloud.points[i].x - m_OriginPoint.x;
		vCloud.points[i].y = vCloud.points[i].y - m_OriginPoint.y;
		vCloud.points[i].z = vCloud.points[i].z - m_OriginPoint.z;
	}

	vPointSecIdx.clear();
	vGroundFeatures.clear();
	//calculate the angle width of a sector
	float fSectorWidth = 2.0 * M_PI / float(m_iSectorNum);
	vPointSecIdx.resize(m_iSectorNum);
	vGroundFeatures.resize(m_iSectorNum);

	//preparation
	int EachSecPNum = ceil(float(vCloud.points.size()) / float(m_iSectorNum));
	for (int i = 0; i != vPointSecIdx.size(); ++i){
		vPointSecIdx[i].reserve(EachSecPNum);
		vGroundFeatures[i].reserve(EachSecPNum);
             }
	//***************************************//
	//traversal
	for (int i = 0; i != vCloud.points.size(); ++i) {

		//prepare
		GroundFeature oPointFeature;

		//compute feature first
		oPointFeature.fDis = Compute3dNorm(vCloud.points[i].x, vCloud.points[i].y, vCloud.points[i].z);
		oPointFeature.fElevation = vCloud.points[i].z;

		//using the angle cosine method
		//base vector is the [1,0]
		float fPointAngl = acos(vCloud.points[i].x
			/ Compute2dNorm(vCloud.points[i].x, vCloud.points[i].y));
	
		//for the points that are in the third and fourth quadrants
		if (vCloud.points[i].y < 0.0)
			fPointAngl = 2.0 * M_PI - fPointAngl;

		//assignment
		int iSectorIdx = floor(fPointAngl / fSectorWidth);
		//if in normal
		if (iSectorIdx < m_iSectorNum){
			//index
			vPointSecIdx[iSectorIdx].push_back(i);
			//corresponding value
			vGroundFeatures[iSectorIdx].push_back(oPointFeature);
		}//if
		else { //defend boundary value due to the accacury of float 
			//index 
			vPointSecIdx[m_iSectorNum - 1].push_back(i);
			//corresponding value
			vGroundFeatures[m_iSectorNum - 1].push_back(oPointFeature);
		}//else

	}//end for i

	return vGroundFeatures;

}