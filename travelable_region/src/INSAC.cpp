#include "INSAC.h"

/*************************************************
Function: INSAC
Description: Constructor of INSAC Class
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: f_fSigmaN SigmaN indicates noise of model
       f_fModelThr indicates how difference between model and truth can be accepted
	   f_fDataThr indicates how level of noise can be accepted
Output: seeds have not been selected
Return: none
Others: none
*************************************************/
INSAC::INSAC(float f_fSigmaN, float f_fModelThr, float f_fDataThr) {

	//force initalize judgement parameters
	SetEvalThreshold(f_fSigmaN, f_fModelThr, f_fDataThr);

	//selection threshold have not been set
	m_bIniSeed = false;
}

/*************************************************
Function: ~INSAC
Description: Deconstructor of INSAC Class
Calls: None
Called By: None
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
INSAC::~INSAC() {
}

/*************************************************
Function: SelectSeeds
Description: find the points that satisfy seed selection threshold 
             as seed points from given point clouds
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vFeatures the features of each point
Output: m_vNewSeedIdx - the new seed point index
        m_vAllPointResults - the inital result of nearby points 
Return: none
Others: none
*************************************************/
void INSAC::SelectSeeds(const std::vector<GroundFeature> & vFeatures) {

	//if the threshold have not been set
	if (!m_bIniSeed) {
		SetSeedThreshold();
	}

	m_vNewSeedIdx.clear();
	m_vUnkownIdx.clear();
	//new a whole point 
	//-1 indicates obstacle,0 indicates unknown point,1 indicates ground
	m_vAllPointResults.clear();
	m_vAllPointResults.resize(vFeatures.size(), 0);

	//select seed points
	for (int i = 0; i != vFeatures.size(); ++i) {

		if (vFeatures[i].fDis < m_fDisThr) {

			if (vFeatures[i].fElevation >= m_fZLower
				&& vFeatures[i].fElevation <= m_fZUpper) {
				//get original seed points
				m_vNewSeedIdx.push_back(i);
				m_vAllPointResults[i] = 1;
			}
			else {
				m_vAllPointResults[i] = -1;//nearby obstacles
			}//end else
		}
		else {
			m_vUnkownIdx.push_back(i);
			m_vAllPointResults[i] = 0;//distant unknown points
		}//end else

	}//end for

}

/*************************************************
Function: ToAddTrainSamples
Description:  make a prepare for GP training input
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vTrainFeaVec output of train feature vector as a matrix
	   vTrainTruVec output of train truth vector as a matrix
	   vCloud given point clouds
	   vFeatures features corresponding with point clouds
Output: vTrainFeaVec
        vTrainTruVec
Return: none
Others: none
*************************************************/
void INSAC::ToAddTrainSamples(MatrixXr & vTrainFeaVec,
	MatrixXr & vTrainTruVec,
	const std::vector<GroundFeature> & vFeatures) {

	//clearand prepare
	vTrainFeaVec.resize(1, m_vNewSeedIdx.size());
	vTrainTruVec.resize(1, m_vNewSeedIdx.size());

	//to each new seed
	for (int i = 0; i != m_vNewSeedIdx.size(); ++i) {
		//get the feature value(s)
		vTrainFeaVec(i) = vFeatures[m_vNewSeedIdx[i]].fDis;
		//...........add 2...3...4dimension

		//get the corresponding truth value(s)
		vTrainTruVec(i) = vFeatures[m_vNewSeedIdx[i]].fElevation;
		//...........add 2...3...4dimension
	}

	//clear old data
	//m_vNewSeedIdx.clear();

}

/*************************************************
Function: ToAddTestSamples
Description: make a prepare for GP test input
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vTestFeaVec output of train feature vector as a matrix
       vTestTruVec output of train truth vector as a matrix
       vCloud given point clouds
       vFeatures features corresponding with point clouds
Output: vTestFeaVec
        vTestTruVec
Return: none
Others: none
*************************************************/
void INSAC::ToAddTestSamples(std::vector<GroundFeatureType> & vTestFeaVec,
	std::vector<GroundTruthType> & vTestTruVec,
	const std::vector<GroundFeature> & vFeatures) {

	//clearand prepare
	vTestFeaVec.clear();
	vTestTruVec.clear();
	//ground feature and truth value tmps
	GroundFeatureType oPointGF;
	GroundTruthType oPointGT;

	//to each new seed
	for (int i = 0; i != m_vUnkownIdx.size(); ++i) {
		//get the feature value(s)
		oPointGF(0) = vFeatures[m_vUnkownIdx[i]].fDis;
		//...........add 2...3...4dimension
		vTestFeaVec.push_back(oPointGF);

		//get the corresponding truth value(s)
		oPointGT(0) = vFeatures[m_vUnkownIdx[i]].fElevation;
		//...........add 2...3...4dimension
		vTestTruVec.push_back(oPointGT);
	}

	//don't clear the vector or you dont know which one is real seed point
	//m_vUnkownIdx.clear();

}

/*************************************************
Function: Eval
Description: Decision function to judge the point is a target or not
Calls: none
Called By: none
Table Accessed: none
Table Updated: none
Input: fZValue - the real height value of a query point
       fMean - the predictive height value of a query point
	   fVar - the variance height value of a query point
Output: 0 unknown point, 1 ground point, -1 obstacle point
Return: label of this query point
Others: none
*************************************************/
int INSAC::Eval(const float & fZValue, const float & fMean, const float & fVar){

	//-1 indicates obstacle,0 indicates unknown point,1 indicates ground
	//first condition is to remove distant point
	if (fVar < m_fModelThr) {//first condition (major)
	    
		//Mahalanobis distance
		float fGroundValue = fabs(fMean - fZValue) / sqrt(m_fSigmaNP + fVar*fVar);
		//std::cout << "Gvalue  " << fGroundValue << std::endl;

		//the second condition is to get ground point based on prediction value
		//it means that the GP algorithm predicts an ideal ground value
		//then a comparison is carried out between the ideal value and query value
		if (fGroundValue < m_fDataThr)//second condition (major)
			return 1;
		else
			return -1;
	   
	}else
		return 0;//too far away so that it will be computed again later

}

/*************************************************
Function: AssignIdx
Description: refresh the seed points and unknown points in the given point clouds
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vUnkownRes the label of old unknown point vector
Output: m_vUnkownIdx - new unknown points index
        m_vNewSeedIdx - new seed points index
Return: it is over if there is not a new adding seed point 
Others: none
*************************************************/
bool INSAC::AssignIdx(const std::vector<int> & vUnkownRes){
    
	m_vNewSeedIdx.clear();
	//get the new seed point from unkown point which has been processed
	for (int i = 0; i != vUnkownRes.size(); ++i){
		//
		m_vAllPointResults[m_vUnkownIdx[i]] = vUnkownRes[i];
		//get new seeds
		if (vUnkownRes[i] == 1)
			m_vNewSeedIdx.push_back(m_vUnkownIdx[i]);

    }

	m_vUnkownIdx.clear();

	//if no seed be added, it denotes all ground points have been found
	if (!m_vNewSeedIdx.size())
		return true;

	//fresh new unkown points
	//check each point n(0) -- linear complexity 
	for(int i=0;i!= m_vAllPointResults.size(); ++i){
	
		//if seed point
		if (m_vAllPointResults[i] == 0) {
			m_vUnkownIdx.push_back(i);
		}
	
	}

	return false;
}

/*************************************************
Function: OutputRes
Description: output the final result
Calls: None
Called By: Major function
Table Accessed: none
Table Updated: none
Input: vAllPointResults output of labels
Output: vAllPointResults also
Return: none
Others: none
*************************************************/
void INSAC::OutputRes(std::vector<int> & vAllPointResults) {

	vAllPointResults.clear();
	vAllPointResults.resize(m_vAllPointResults.size());

	for (int i = 0; i != m_vAllPointResults.size(); ++i)
		vAllPointResults[i] = m_vAllPointResults[i];

}