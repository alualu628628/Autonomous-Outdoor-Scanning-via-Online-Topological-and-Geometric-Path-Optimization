#ifndef GAUSSIANPROCESSREGRESSION_HXX
#define GAUSSIANPROCESSREGRESSION_HXX


/*************************************************
Function: GaussianProcessRegression
Description: Constructor of GaussianProcessRegression Class
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: f_iFeatureDim the dimension of the feature vector 
       f_iTruthDim the truth value dimension of the input train data set
Output: none
Return: none
Others: set m_bTrainFlag as the data need be trained again if adding any new data
*************************************************/
template<typename R>
GaussianProcessRegression<R>::GaussianProcessRegression(int f_iFeatureDim,int f_iTruthDim)
{
  m_mTrainFeatures.resize(f_iFeatureDim,0);//feature dimension

  m_mTrainTruth.resize(f_iTruthDim,0);//truth value dimension

  m_iSampleNum = 0;// set all sample number is zero 

  m_bTrainFlag = false;//Training has not been performed
}


/*************************************************
Function: AddTrainingData
Description: Add the input training data one by one
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input:  f_mFeatures feature vector
        f_mTruthValues truth vector corresponding with input feature vector
Output: none
Return: none
Others: m_bTrainFlag
*************************************************/
template<typename R>
void GaussianProcessRegression<R>::AddTrainingData(const VectorXr& f_mFeatures,const VectorXr& f_mTruthValues)
{
  m_iSampleNum++;

  if(m_iSampleNum >= m_mTrainFeatures.cols()){

     m_mTrainFeatures.conservativeResize(m_mTrainFeatures.rows(),m_iSampleNum);

     m_mTrainTruth.conservativeResize(m_mTrainTruth.rows(),m_iSampleNum);
  }

  m_mTrainFeatures.col(m_iSampleNum - 1) = f_mFeatures;

  m_mTrainTruth.col(m_iSampleNum - 1) = f_mTruthValues;

  //need to recompute the K matrix if add new train data set
  m_bTrainFlag = false;

}

/*************************************************
Function: AddTrainingDatas
Description: Add a bunch of input training data at once 
Calls: None
Called By:  Major function
Table Accessed: none
Table Updated: none
Input: f_mFeatures feature vector
       f_mTruthValues truth vector corresponding with input feature vector
Output: none
Return: none
Others: set m_bTrainFlag as the data need be trained again if adding any new data
*************************************************/
template<typename R>
void GaussianProcessRegression<R>::AddTrainingDatas(const MatrixXr& f_mFeatures, const MatrixXr& f_mTruthValues)
{
  // sanity check of provided data
  assert(f_mFeatures.cols() == f_mTruthValues.cols());
  // if this is the first data, just add it..
  if(m_iSampleNum == 0){
    m_mTrainFeatures = f_mFeatures;
    m_mTrainTruth = f_mTruthValues;
    m_iSampleNum = m_mTrainFeatures.cols();
  }
  // if we already have data, first check dimensionaly match
  else{
    assert(m_mTrainFeatures.rows() == f_mFeatures.rows());
    assert(m_mTrainTruth.rows() == f_mTruthValues.rows());
    size_t iOldSampleNum = m_iSampleNum;
    m_iSampleNum += f_mFeatures.cols();
    // resize the matrices
    if(m_iSampleNum > m_mTrainFeatures.cols()){
      m_mTrainFeatures.conservativeResize(m_mTrainFeatures.rows(),m_iSampleNum);
      m_mTrainTruth.conservativeResize(m_mTrainTruth.rows(),m_iSampleNum);
    }
    // insert the new data using block operations
    m_mTrainFeatures.block(0,iOldSampleNum,f_mFeatures.rows(),f_mFeatures.cols()) = f_mFeatures;
    m_mTrainTruth.block(0,iOldSampleNum,f_mTruthValues.rows(),f_mTruthValues.cols()) = f_mTruthValues;
  }

  //need to recompute the K matrix if add new train data set
  m_bTrainFlag = false;

}

/*************************************************
Function: SqrExpKernelFun
Description: compute the feature relationship using a measurement formula
Calls: None
Called By: ComputeCovarMatrix which computes the K
           CompPredictiveVar which computes the K_**
Table Accessed: none
Table Updated: none
Input: f_vA,f_vB two vectors
Output: measured distance between two input vectors
Return: dDis
Others: none
*************************************************/
template<typename R>
R GaussianProcessRegression<R>::SqrExpKernelFun(VectorXr f_vA, VectorXr f_vB)
{
  //
  VectorXr vVecDis = f_vA - f_vB;

  double dDis = vVecDis.dot(vVecDis);

  dDis = m_dSigmaF * m_dSigmaF * exp( -1.0 / m_dLScale / m_dLScale / 2.0 * dDis);

  return dDis;
}

/*************************************************
Function: ComputeCovarMatrix
Description: Rolaod of ComputeCovarMatrix, which computes the K_*
Calls: None
Called By: CompPredictiveMean computes the predictive value
           CompPredictiveVar computes the noise of predictive value
Table Accessed: none
Table Updated: none
Input: f_mA the training feature vectors
       f_vB one test feature vector
Output: K_*
Return: mKStarMat
Others: none
*************************************************/
template<typename R>
typename GaussianProcessRegression<R>::VectorXr GaussianProcessRegression<R>::ComputeCovarMatrix(MatrixXr f_mA, VectorXr f_vB){
  
  /******
  //compute K which is the m_mKMat in code
  //where
	|k(x1,x1) .... k(x1,xn)|
  K=|k(x2,x1)  :.  k(x2,xn)|
	|k(xn,x1) .... k(xn,xn)|
  ******/
  int iCols = f_mA.cols();

  VectorXr mKStarMat(iCols);

  for(int i = 0; i < iCols; ++i){

    mKStarMat(i) = SqrExpKernelFun(f_mA.col(i),f_vB);

  }

  return mKStarMat;

}

/*************************************************
Function: ComputeCovarMatrix
Description: ComputeCovarMatrix, which computes the covariance matrix K
Calls: None
Called By: TrainData which trains the input features
Table Accessed: none
Table Updated: none
Input: f_mA the input training feature vectors - col vector means sample
Output: K
Return: mCovarMat
Others: none
*************************************************/
template<typename R>
typename GaussianProcessRegression<R>::MatrixXr GaussianProcessRegression<R>::ComputeCovarMatrix(MatrixXr f_mA) {

	/******
	//compute K which is the m_mKMat in code
	//where
	  |k(x1,x1) .... k(x1,xn)|
	K=|k(x2,x1)  :.  k(x2,xn)|
	  |k(xn,x1) .... k(xn,xn)|
	******/
	int iCols = f_mA.cols();

	MatrixXr mCovarMat(iCols, iCols);
	
	for (int i = 0; i< iCols; i++) {

		for (int j = i; j < iCols; j++) {

			mCovarMat(i, j) = SqrExpKernelFun(f_mA.col(i), f_mA.col(j));

			mCovarMat(j, i) = mCovarMat(i, j);

		}//end j
	}//end i

	return mCovarMat;

}

/*************************************************
Function: TrainData
Description: train the input data
Calls: None
Called By: Regression which is to predict the truth value of new test data
Table Accessed: none
Table Updated: none
Input: none, it depends on the AddTrainingDatas
                        or     AddTrainingData
Output: m_mKMat the generated model K (a matrix)
        m_bTrainFlag indicates that the training is completed
Return: none
Others: none
*************************************************/
template <typename R>
void GaussianProcessRegression<R>::TrainData() {

	//compute the K matrix which is a covariance matrix
	m_mKMat = ComputeCovarMatrix(m_mTrainFeatures);

	// add measurement noise
	for (int i = 0; i<m_mKMat.cols(); ++i)
		// this is the the Kronecker delta function
		// sign^2*selta(x,x')
		m_mKMat(i, i) += m_dSigmaN * m_dSigmaN;

	//compute the K(^-1)
	m_mKinverse = m_mKMat.inverse();

	m_bTrainFlag = true;

}

/*************************************************
Function: CompPredictiveMean
Description: Computes the predictive value of one input test data
Calls: None
Called By: Regression
Table Accessed: none
Table Updated: none
Input: vPredValue - the output about predictive value
       f_vTestFeatures - the input of one test feature vector
Output: vPredValue - a vector denotes the truth vector (maybe 1 by 1 dim)
Return: none
Others: none
*************************************************/
template <typename R>
void GaussianProcessRegression<R>::CompPredictiveMean(VectorXr & vPredValue,const VectorXr & f_vTestFeatures){

  //compute the prediction value (y(*)_)
  //
  m_vKStarMat = ComputeCovarMatrix(m_mTrainFeatures, f_vTestFeatures);

  VectorXr vKiKst(m_mTrainFeatures.cols());
  
  //(AB)^T = B^T A^T
  //particularly, K^-1 is a symmetric matrix,whose transpose matrix is itself
  vKiKst = m_mKinverse*m_vKStarMat;
  
  // the rest is noise in comparison with the above line.
  for(int i=0;i<m_mTrainTruth.rows();i++){
	  //is equal to dot operation
	  //and the dot operation is 
      vPredValue(i)= vKiKst.dot(m_mTrainTruth.row(i));

  }

}

/*************************************************
Function: CompPredictiveVar
Description: Computes the variance of predictive value 
Calls: None
Called By: Regression
Table Accessed: none
Table Updated: none
Input: vPredVar - the output about predictive variance
       f_vTestFeatures - the input of one test feature vector
Output: vPredVar the variance vector corresponding the predictive value in each dimension
        (but all dimension are with a same value)
Return: none
Others: none
*************************************************/
template <typename R>
void GaussianProcessRegression<R>::CompPredictiveVar(VectorXr & vPredVar,const VectorXr & f_vTestFeatures){

  //compute the variance of prediction value Var(y(*))
  //where Var(y(*)) = K_** - K_* K^-1 K_*(^T)
  //we have Var(y(*))^T = K_**^T - (K_* K^-1 K_*(^T))^T
  //compute K_**
  m_vKStarStar = SqrExpKernelFun(f_vTestFeatures, f_vTestFeatures);
  //add noise
  m_vKStarStar += m_dSigmaN * m_dSigmaN;
  //compute the K_*
  m_vKStarMat = ComputeCovarMatrix(m_mTrainFeatures, f_vTestFeatures);

  //compute K(-1)K(*)(T)
  VectorXr vKiKst(m_mTrainFeatures.cols());
  
  vKiKst = m_mKinverse * m_vKStarMat;

  //if output is multi-dimension value
  for(int i=0; i < m_mTrainTruth.rows(); ++i){
	  
	  //since K_* K^-1 K_*(^T))^T = K_*^T K^-1 K_*
      vPredVar(i) = m_vKStarMat.transpose() * vKiKst;
	  //compute the K_** - otherwise
	  vPredVar(i) = m_vKStarStar - vPredVar(i);
      //std::cout<<"Pred Var"<<std::endl<<vPredVar(i)<<std::endl;
  }//end for 

}

/*************************************************
Function: Regression
Description: Computes the predictive value and its variance based on a input test features
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vPredValue output of predictive value 
       vPredVar output of predictive variance 
	   f_vTestFeatures input of test feature
Output: none
Return: true if there are a prediction value output
Others: none
*************************************************/
// This is the right way to do it but this code should be refactored and tweaked so that the decompositon is not recomputed unless new training data has arrived. 
template <typename R>
bool GaussianProcessRegression<R>::Regression(VectorXr & vPredValue, VectorXr & vPredVar, const VectorXr & f_vTestFeatures) {

  //get the dimension of the truth value
  vPredValue.resize(m_mTrainTruth.rows());
  vPredValue.setZero();
  vPredVar.resize(m_mTrainTruth.rows());
  vPredVar.setZero();

  //check whether the model is trained
  if (!m_bTrainFlag && m_iSampleNum)
	  TrainData();
  else if(!m_iSampleNum)
      return false;// can return 0 immediately

  //std::cout << "K:" << std::endl << m_mKMat << std::endl;
  //std::cout << "K':" << std::endl << m_mKinverse << std::endl;
  
  //compute the regression value y based on the given test feature
  CompPredictiveMean(vPredValue, f_vTestFeatures);

  //compute the variance of prediction value
  CompPredictiveVar(vPredVar, f_vTestFeatures);

  return true;

}

/*************************************************
Function: ClearTrainingData
Description: clear the old data
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: none
Output: m_bTrainFlag denotes the train has not been completed
Return: none
Others: none
*************************************************/
template<typename R>
void GaussianProcessRegression<R>::ClearTrainingData()
{

  //clear samples
  m_mTrainFeatures.resize(m_mTrainFeatures.rows(),0);
  m_mTrainTruth.resize(m_mTrainTruth.rows(),0);

  //clear situation
  m_bTrainFlag = false;
  m_iSampleNum = 0;

  //other parameters will be refreshed automatically in new loop
}


#endif 

