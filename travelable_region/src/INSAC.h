#ifndef INSAC_H
#define INSAC_H
#include "SectorPartition.h"
#include "GaussianProcess.h"
///************************************************************************///
// a class denotes the GP-INSAC algorithm (i.e.,INSAC part)
//
// the implementation refers to this paper as below:
// Douillard B, Underwood J, Kuntz N, et al. On the segmentation of 3D LIDAR point clouds,
// IEEE International Conference on Robotics and Automation. IEEE, 2011:2798-2805.
//
// Generated and edited by Huang Pengdi 2018.10.16
///************************************************************************///
class GPINSACThrs {

public:
	//constructor
	GPINSACThrs() : iSector_num(360),
	                           fDisThr(7.0),
		              fZLower(-2.5),
		              fZUpper(-1.0),
		             dLScale(28.01),
		             dSigmaF(1.76),
		             dSigmaN(0.12),
		             fModelThr(0.2),
		             fDataThr(1.5) {
		//none
	}

             int iSector_num;
	//seed selection thresholds
	float fDisThr;
	float fZLower;
	float fZUpper;
	//GP model thresholds
	double dLScale;
	double dSigmaF;
	double dSigmaN;
	//INSAC thresholds
	float fModelThr;
	float fDataThr;
};


class INSAC {

	//ground feature vector type
    //***feature dimension can be changed based on your own algorithm
	typedef Eigen::Matrix<float, 1, 1> GroundFeatureType;
	typedef Eigen::Matrix<float, 1, 1> GroundTruthType;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXr;

public:

	//construction function
	INSAC(float f_fSigmaN, float f_fModelThr = 0.2, float f_fDataThr = 1.5);
	
	//
	~INSAC();

	//Set the parameter of Eval function
	void SetEvalThreshold(const float & f_fSigmaN,
		const float & f_fModelThr,const float & f_fDataThr) {
	    //assigment
        m_fSigmaNP = f_fSigmaN * f_fSigmaN;///<SigmaN^2
		m_fModelThr = f_fModelThr;///<Tmodel
		m_fDataThr = f_fDataThr;///<Tdata
	};

	//Set the parameter of seed selection
	void SetSeedThreshold(float f_fDisThr = 7.0,
		float f_fZLower = -2.5, float f_fZUpper = -1.0) {
		//assigment
		m_fDisThr = f_fDisThr;
		m_fZUpper = f_fZUpper;
		m_fZLower = f_fZLower;
		m_bIniSeed = true;
	};

	//Initialize seed point
	void SelectSeeds(const std::vector<GroundFeature> & vFeatures);
	
	//to make a point clouds as the train input of GaussianProcessRegression
	void ToAddTrainSamples(MatrixXr & vTrainFeaVec,
		                   MatrixXr & vTrainTruVec,
		                   const std::vector<GroundFeature> & vFeatures);

	//get the number of train data set 
	int GetTrainSmpNum() {
		return m_vNewSeedIdx.size();
	};

	//to make a point clouds as the test input of GaussianProcessRegression
    void ToAddTestSamples(std::vector<GroundFeatureType> & vTestFeaVec,
	                  std::vector<GroundTruthType> & vTestTruVec,
	                  const std::vector<GroundFeature> & vFeatures);

	//Eval function to measure the predictive value meet the target or not
	int Eval(const float & fZValue, const float & fMean, const float & fVar);

	//refresh the train data index (get the new train data)
	bool AssignIdx(const std::vector<int> & vUnkownRes);

	//output the final result - the label of each point
	void OutputRes(std::vector<int> & vAllPointResults);

private:

	//seed selection related 
	float m_fDisThr; ///<near distance between query point and base point
	float m_fZUpper; ///<upper bound value of Z 
	float m_fZLower; ///<lower bound value of Z
	bool m_bIniSeed; ///<flag of initializing seed 

	//judge new seed related
	float m_fModelThr;///<Tmodel
	float m_fDataThr;///<Tdata
	float m_fSigmaNP;///<SigmaN^2

	//Incremental management of seeds
	//{ALL P} ={Pseed, Punkown, PnonGround}
    //NEW means this arg only records the new seeds and abandons old seeds  
	std::vector<int> m_vNewSeedIdx;///<NEW seed index in the point clouds of corresponding sector
	std::vector<int> m_vUnkownIdx;///<the unkown point index (in whole input point clouds)
	//attation: the value is situation not index
	std::vector<int> m_vAllPointResults;///<a vector indicates the point has been a seed point or not 
	
};





#endif // !INSAC_H




///*****************************how to use it*******************************
//
//#include "HpdPointCloudDisplay.h"
//#include "LasOperator.h"
//#include "INSAC.h"
//#include <iostream>
//#include <cmath>
//
//int main() {
//
//std::vector<Point3D> vOneFrame;
//pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
//HPDpointclouddataread("_PC_1frame.las", vOneCloud, vOneFrame);
//
//std::vector<Point3D> vRobotPoint;
//pcl::PointCloud<pcl::PointXYZ>::Ptr vRobotCloud(new pcl::PointCloud<pcl::PointXYZ>);
//HPDpointclouddataread("_Traj_1frame.las", vRobotCloud, vRobotPoint);
//
//std::vector<std::vector<int>> oPointSecIdxs;
//DivideSector oSectorDivider(540);
//oSectorDivider.SetOriginPoint(vRobotCloud->points[0]);
//std::vector<std::vector<GroundFeature> > vGroundFeatures = oSectorDivider.ComputePointSectorIdxs(*vOneCloud, oPointSecIdxs);
//std::vector<std::vector<int>> vAllGroundRes;
//
//for (int i = 0; i != vGroundFeatures.size(); ++i) {
//std::vector<int> vTmpRes(vGroundFeatures[i].size(),0);
//vAllGroundRes.push_back(vTmpRes);
//}
//
//
//for (int is = 0; is != oPointSecIdxs.size(); ++is) {
//
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr vOneSectorCloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//for (int j = 0; j != oPointSecIdxs[is].size(); ++j)
//vOneSectorCloud->push_back(vOneCloud->points[oPointSecIdxs[is][j]]);
//
//GaussianProcessRegression<float> GPR(1, 1);
//float fSigmaN = 0.15;
//GPR.SetHyperParams(18.01, 0.88, fSigmaN);
//
//INSAC GPINSAC(fSigmaN,0.2,2.0);
//
//GPINSAC.SelectSeeds(vGroundFeatures[is]);
//
//bool bGrowFlag = true;
//
//int iTimes = 0;
//
//while(bGrowFlag){
//
//Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainFeaVec;
//Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainTruVec;
//GPINSAC.ToAddTrainSamples(vTrainFeaVec, vTrainTruVec, vOneCloud, vGroundFeatures[is]);
//
//std::vector<Eigen::Matrix<float, 1, 1>> vTestFeaVec;
//std::vector<Eigen::Matrix<float, 1, 1>> vTestTruVec;
//GPINSAC.ToAddTestSamples(vTestFeaVec, vTestTruVec, vOneCloud, vGroundFeatures[is]);
//
//GPR.AddTrainingDatas(vTrainFeaVec, vTrainTruVec);
//
//std::vector<int> vOneLoopLabels;
//for (size_t k = 0; k < vTestFeaVec.size(); k++) {
//
//Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
//Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
////std::cout <<"vTestFeaVec[k] :"<< vTestFeaVec[k] << std::endl;
//if (GPR.Regression(vPredValue, vPredVar, vTestFeaVec[k])){
//
////std::cout << "predict" << vPredValue(0) << std::endl;
////std::cout << "groundtruth" << vTestTruVec[k](0) << std::endl;
////std::cout << "var" << vPredVar(0) << std::endl;
//int iPointLabel = GPINSAC.Eval(vTestTruVec[k](0),vPredValue(0), vPredVar(0));
//vOneLoopLabels.push_back(iPointLabel);
//
//}
//else
//vOneLoopLabels.push_back(-1);
//}
//
//if(GPINSAC.AssignIdx(vOneLoopLabels))
//bGrowFlag = false;
//
////std::cout <<"iTimes:  "<< iTimes << std::endl;
//iTimes++;
//}//while
//
//std::vector<int> vGroundLabels;
//GPINSAC.OutputRes(vGroundLabels);
//
//for (int j = 0; j != vGroundLabels.size(); ++j)
//vAllGroundRes[is][j] = vGroundLabels[j];
//
//}//end is
//
//for (int i = 0; i != oPointSecIdxs.size(); ++i) {
//for (int j = 0; j != oPointSecIdxs[i].size(); ++j) {
//vOneFrame[oPointSecIdxs[i][j]].classification = vAllGroundRes[i][j];
//}
//}
//
//CLasOperator saver;
//saver.saveLasFile("res.las", &vOneFrame);
////
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//HpdDisplay hpdisplay;
//viewer = hpdisplay.Showclassification(vOneFrame, "assign");
//while (!viewer->wasStopped())
//{
//viewer->spinOnce();
//}
//
//return 0;
//
//}
//
//*************************************************************/