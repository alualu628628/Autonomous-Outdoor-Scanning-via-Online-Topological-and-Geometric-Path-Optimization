#ifndef SECTORPARTITION_H
#define SECTORPARTITION_H

///************************************************************************///
// a class denotes the dividing sector operation
//
// Generated and edited by Huang Pengdi from 2018.09.27
//
///************************************************************************///

//based lib
#include <vector>
#include <iostream>

//pcl related lib
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct GroundFeature {

	float fDis;///<distance
    //the height value of robot minus height value of query point
	float fElevation;///<relative height

};


class DivideSector {

public:

	///construct function
	DivideSector(int f_iSectorNum = 8) {

		//set how many sectors should be divded in scanning region 
		SetSectorNum(f_iSectorNum);

		//haven't set the original point of scanning region
		m_OriginPoint.x = 0.0;
		m_OriginPoint.y = 0.0;
		m_OriginPoint.z = 0.0;
		m_bOriPointFlag = false;

	};

	//deconstruction function
	~DivideSector() {
	};

	//set the number value of sector
	void SetSectorNum(int f_iSectorNum);
	//set the original point of point clouds
	void SetOriginPoint(const pcl::PointXYZ & f_oPoint);

	//compute the norm of a vector, and the ouput must be not a zero
	inline float Compute2dNorm(float f_x, float f_y) {
		//compute norm
		float fNorm = sqrt(pow(f_x, 2.0f) + pow(f_y, 2.0f));
		if (fNorm)
			return fNorm;
		else
			return FLT_MAX;//to a reasonable value

	};

	//compute the norm of a vector, and the ouput must be not a zero, reload
	inline double Compute2dNorm(double f_x, double f_y) {
		//compute norm
		double fNorm = sqrt(pow(f_x, 2.0) + pow(f_y, 2.0));
		if (fNorm)
			return fNorm;
		else
			return DBL_MAX;//to a reasonable value

	};

	//compute the norm of a 3d vector, and the ouput can be a zero
	inline float Compute3dNorm(float f_x, float f_y, float f_z) {
		//compute norm
		float fNorm = sqrt(pow(f_x, 2.0f) + pow(f_y, 2.0f) + pow(f_z, 2.0f));
		return fNorm;
		
	};

	//compute the corresponding sector section to each point 
	std::vector<std::vector<GroundFeature> > ComputePointSectorIdxs(pcl::PointCloud<pcl::PointXYZ> vCloud,
		std::vector<std::vector<int> > & vPointSecIdx);

	//obtained features
	//std::vector<std::vector<float> > m_vPointFeatures;

private:

	//number of sectors 
	int m_iSectorNum;
	//the original point
	pcl::PointXYZ m_OriginPoint;
	bool m_bOriPointFlag;

};




#endif

///******One Example indicates how to use this code*******
///***********Any Question contact Huang********************
//
//#include "HpdPointCloudDisplay.h"
//#include "LasOperator.h"
//#include "SectorPartition.h"
//#include <iostream>
//#include <cmath>
//
//
//
//int main() {
//
//	std::vector<Point3D> vOneFrame;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	HPDpointclouddataread("_PC_1frame.las", vOneCloud, vOneFrame);
//
//	std::vector<Point3D> vRobotPoint;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr vRobotCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	HPDpointclouddataread("_Traj_1frame.las", vRobotCloud, vRobotPoint);
//
//	std::vector<std::vector<int>> oPointSecIdxs;
//	DivideSector oSectorDivider(16);
//	oSectorDivider.SetOriginPoint(vRobotCloud->points[0]);
//	oSectorDivider.ComputePointSectorIdxs(*vOneCloud, oPointSecIdxs);
//	
//	vOneFrame.clear();
//	for (int i = 0; i != vOneCloud->points.size(); ++i) {
//		Point3D onePoint;
//		onePoint.x = vOneCloud->points[i].x;
//		onePoint.y = vOneCloud->points[i].y;
//		onePoint.z = vOneCloud->points[i].z;
//		vOneFrame.push_back(onePoint);
//	}
//
//
//	for (int i = 0; i != oPointSecIdxs.size(); ++i) {
//		for (int j = 0; j != oPointSecIdxs[i].size(); ++j) {
//			vOneFrame[oPointSecIdxs[i][j]].classification = i;
//		}
//	}
//	//
//	CLasOperator saver;
//	saver.saveLasFile("res.las", &vOneFrame);
//	//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	HpdDisplay hpdisplay;
//	viewer = hpdisplay.Showclassification(vOneFrame, "assign");
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//
//	return 0;
//
//}
//
//
//
