#ifndef GAUSSIANPROCESS_H
#define GAUSSIANPROCESS_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>


///************************************************************************///
// a class denotes the Gaussian Process Regression algorithm

// the implementation refers to this paper as below:
// Ebden M. Gaussian Processes: A Quick Introduction[J]. Statistics, 2015, 36.

// the raw version is based on https://github.com/jonfink/GP.git
// Modified by Huang Pengdi 2018.10.09, Modified parts:
// 1.Modify a lots of raw codes (The codes now is quite different from the raw one)
// 2.Modify some mathmatics
// 3.Add new functions
// 4.Add new outputs
///************************************************************************///
template<typename REALTYPE>
class GaussianProcessRegression {

	typedef Eigen::Matrix<REALTYPE, Eigen::Dynamic, Eigen::Dynamic> MatrixXr;
	typedef Eigen::Matrix<REALTYPE, Eigen::Dynamic, 1> VectorXr;

public:

	//construction function
	GaussianProcessRegression(int f_iFeatureDim, int f_iTruthDim);

	//****Parameter Related Function*****
	//input function
	void SetHyperParams(double f_dLScale = 28.01,
		                double f_dSigmaF = 1.76, 
		                double f_dSigmaN = 0.12) {
		m_dLScale = f_dLScale;
		m_dSigmaF = f_dSigmaF;
		m_dSigmaN = f_dSigmaN;
	}; 
	//output function
	void OutputHyperParams(double & f_dLScale, double & f_dSigmaF, double & f_dSigmaN) {
		f_dLScale = m_dLScale;
		f_dSigmaF = m_dSigmaF;
		f_dSigmaN = m_dSigmaN;
	};

	//****Training Function*****
	// add data one by one
	void AddTrainingData(const VectorXr& f_mFeatures, const VectorXr& f_mTruthValues);
	
	// batch add data
	void AddTrainingDatas(const MatrixXr& f_mFeatures, const MatrixXr& f_mTruthValues);
	
	//compute covariance matrix based on the train input
	MatrixXr ComputeCovarMatrix(MatrixXr f_mA);

	//compute covariance matrix with the test input
	VectorXr ComputeCovarMatrix(MatrixXr f_mA, VectorXr f_vB);

	//metric function
	inline REALTYPE SqrExpKernelFun(VectorXr f_vA, VectorXr f_vB);

	// the major function of training data
	void TrainData();

	//****Regression Function*****
	// compute two ouputs of the Gaussian Process Regression
	//one is the prediection value y(_):
	void CompPredictiveMean(VectorXr & vPredValue, const VectorXr & f_vTestFeatures);
	
	//another is the variance value var(y):
	void CompPredictiveVar(VectorXr & vPredVar,const VectorXr & f_vTestFeatures);

	//the major function of regression
	bool Regression(VectorXr & vPredValue, VectorXr & vPredVar, const VectorXr & f_vTestFeatures);

	//****Output Function*****
	//Output the number of samples
	int OutputSampleNum() {
		return m_iSampleNum;
	};

	//Output train data
    MatrixXr & GetTrainData() {
		return m_mTrainFeatures;
	};

	//Output train data
	MatrixXr & GetTrainTruthValue() {
		return m_mTrainTruth;
	};

	//clear data
	void ClearTrainingData();
	
private:

	//input training operation data type
	MatrixXr m_mTrainFeatures;///<feature matrix
	MatrixXr m_mTrainTruth;///<truth value of training data 

	//covariance matrix
	MatrixXr m_mKMat;///<K n*n covariance matrix with noise polynomial
	MatrixXr m_mKinverse;///<K with Kronecker delta function	K^T	

	VectorXr m_vKStarMat;///<K_* 
	REALTYPE m_vKStarStar;///<K_**

	//the number of data set 
	int m_iSampleNum;
	//a flag indicates that input data have been trained or not 
	bool m_bTrainFlag;
	
	//the parameters of Gaussian Process Regression algorithm
	double m_dLScale;///<scale l
	double m_dSigmaF;///<sigma f
	double m_dSigmaN;///<sigma n

};

#include "GaussianProcessRegression.hxx"

#endif // !GAUSSIANPROCESS_H

///******One Example indicates how to use this code*******
///***********Any Question contact Huang********************
//
//#include "GaussianProcess.h"
//#include <iostream>
//#include <cmath>
//
//int main() {
//
//	//
//
//	GaussianProcessRegression<float> gpr(1, 1);
//	gpr.SetHyperParams(1.16, 1.268, 0.3);
//	typedef Eigen::Matrix<float, 1, 1> input_type;
//	typedef Eigen::Matrix<float, 1, 1> output_type;
//	std::vector<input_type> test_inputs;//train_inputs, 
//	std::vector<output_type> test_outputs;//train_outputs, 
//
//	Eigen::Matrix<float, 1, 4> train_inputs;
//	train_inputs(0) = -1.50;
//	train_inputs(1) = -1.00;
//	train_inputs(2) = -0.75;
//	train_inputs(3) = -0.40;
//	//train_inputs(4)=-0.25;
//	//train_inputs(5)=0.00;
//	//std::cout<<train_inputs<<std::endl<<"another:"<<std::endl;
//	Eigen::Matrix<float, 1, 4> train_outputs;
//	train_outputs(0) = -1.6;
//	train_outputs(1) = -1.1;
//	train_outputs(2) = -0.4;
//	train_outputs(3) = 0.2;
//	//train_outputs(4)=0.5;
//	//train_outputs(5)=0.8;
//	//std::cout<<train_outputs<<std::endl;
//	gpr.AddTrainingDatas(train_inputs, train_outputs);
//
//	Eigen::Matrix<float, 1, 1> oneTestInputs;
//	oneTestInputs(0) = 0.2;
//	test_inputs.push_back(oneTestInputs);
//
//	Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
//	Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
//	for (size_t k = 0; k<test_inputs.size(); k++) {
//		gpr.Regression(vPredValue, vPredVar, test_inputs[k]);
//		//std::cout<<"y:"<<std::endl<<vPredValue<<std::endl;
//		//std::cout << "var(y):" << std::endl << vPredVar << std::endl;
//	}
//
//	std::vector<input_type> test_inputs2;//train_inputs, 
//	std::vector<output_type> test_outputs2;//train_outputs, 
//
//	Eigen::Matrix<float, 1, 2> train_inputs2;
//
//	train_inputs2(0) = -0.25;
//	train_inputs2(1) = 0.00;
//	//std::cout << train_inputs2 << std::endl << "another:" << std::endl;
//	Eigen::Matrix<float, 1, 2> train_outputs2;
//
//	train_outputs2(0) = 0.5;
//	train_outputs2(1) = 0.8;
//	//std::cout << train_outputs2 << std::endl;
//	gpr.AddTrainingDatas(train_inputs2, train_outputs2);
//
//	Eigen::Matrix<float, 1, 1> oneTestInputs2;
//	oneTestInputs2(0) = 0.2;
//	test_inputs2.push_back(oneTestInputs2);
//	oneTestInputs2(0) = 0.2;
//	test_inputs2.push_back(oneTestInputs2);
//	oneTestInputs2(0) = 0.4;
//	test_inputs2.push_back(oneTestInputs2);
//	oneTestInputs2(0) = 0.2;
//	test_inputs2.push_back(oneTestInputs2);
//
//	Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue2;
//	Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar2;
//	for (size_t k = 0; k<test_inputs2.size(); k++) {
//		gpr.Regression(vPredValue2, vPredVar2, test_inputs2[k]);
//		//std::cout << "y:" << std::endl << vPredValue2 << std::endl;
//		//std::cout << "var(y):" << std::endl << vPredVar2 << std::endl;
//	}
//
//
//	std::cin.get();
//	return 0;
//
//}
//
//*******************************************************/
