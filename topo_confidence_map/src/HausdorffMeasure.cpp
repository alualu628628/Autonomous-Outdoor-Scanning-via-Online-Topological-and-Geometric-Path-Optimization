#include "HausdorffMeasure.h"
//Edited by Huang Pengdi 2018.12.30 

/*************************************************
Function: HausdorffDimension
Description: constrcution function for HausdorffDimension class
Calls: InitialLoopParams
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_iIterMax - the lower bound of iterations (calculated depth) allowed by default
       f_iIterMin - the upper bound of iterations (root) allowed by default
Output: initial all of member variables
Return: none
Others: none
*************************************************/
HausdorffDimension::HausdorffDimension(int f_iIterMax,
	                                   int f_iIterMin):
                                       m_fRedundancy(0.005){

	//initial the parameter 
	InitialLoopParams(f_iIterMax,f_iIterMin);
	
	//the default calculation method is to measure the occupancy rate.
	m_fParaQ = 0.0;
	
	//flag related
	m_bEdgeFlag = false;///<boundary length

	m_bMaxMinCoFlag = false;///<bounding box corner

	m_bScaleFlag = false;///<measuring scale

	m_bMinDisFlag = false;///<minimum measuring scale
	
}

/*************************************************
Function: ~HausdorffDimension
Description: destrcution function for HausdorffDimension class
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
HausdorffDimension::~HausdorffDimension(){

}

/*************************************************
Function: SetMinDis
Description: get the minimum measuring scale (meter)
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_fMinDis - the minimum measuring scale in meter
Output: m_fMinDis which records the given minimum measuring scale
Return: none
Others: none
*************************************************/
void HausdorffDimension::SetMinDis(float f_fMinDis){

	m_bMinDisFlag = true;

	m_fMinDis = f_fMinDis;
}

/*************************************************
Function: SetMaxMinCoor
Description: give a priori corner value of bounding box
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_oMaxCoor - a value that saves the maximum value at each axis of point set
       f_oMinCoor - a value that saves the minimum value at each axis of point set
Output: m_oMinCoor, m_oMaxCoor 
Return: none
Others: none
*************************************************/
void HausdorffDimension::SetMaxMinCoor(const pcl::PointXYZ & f_oMaxCoor, 
		                               const pcl::PointXYZ & f_oMinCoor){

	//true if call this function
	m_bMaxMinCoFlag = true;

	//change the value if the input order is incorrect
	//any component value of oMaxCoor is larger than oMinCoor
	if(f_oMaxCoor.x < f_oMinCoor.x){

	   m_oMinCoor = f_oMaxCoor;
	   m_oMaxCoor = f_oMinCoor;

	}else{

	   m_oMinCoor = f_oMinCoor;
	   m_oMaxCoor = f_oMaxCoor; 

	}

}

/*************************************************
Function: SetGivenScales
Description: give minimum and maximum scale that is expected adopting in calculation
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_fLargeScale - the largest measuring scale 
	   f_fSmallScale - the smallest measuring scale
Output: m_fLargeScale, m_fSmallScale 
Return: none
Others: none
*************************************************/
void HausdorffDimension::SetGivenScales(const float & f_fLargeScale,
	                                    const float & f_fSmallScale){

	//assigment
	m_fLargeScale = f_fLargeScale;
	m_fSmallScale = f_fSmallScale;

	//turn flag
	m_bScaleFlag = true;
}

/*************************************************
Function: SetParaQ
Description: set the generalized dimensional parameter q, which is proposed by H. G. E. Hentschel
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_iParaQ - q parameter
Output: m_fParaQ - a float type q
Return: none
Others: Dq = lim(Sq/ln(scale)), where Sq = (1/(1-q))*ln(sum(pi^q))
        Sq is also called Renyi information dimension
*************************************************/
void HausdorffDimension::SetParaQ(int f_iParaQ){

	//get the Q parameter
	m_fParaQ = float(f_iParaQ);
	
	// q=1 would cause dividing zero
	if (m_fParaQ == 1){
		std::cout<<"Error: q = 1 is not prepared in this programme£¬choose 0 or 2 please £¡"<<std::endl;
	    std::cin.get();
		exit(0);
	}

}

/*************************************************
Function: ExtractEdgeLength
Description: extract the length of bounding box 
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: oOutLength
Output: oOutLength - the ouput length at each axis
Return: true - There is indeed a calculated length 
        false - the length of bounding box has not been computed yet
Others: none
*************************************************/
bool HausdorffDimension::ExtractEdgeLength(pcl::PointXYZ & oOutLength){

	//clear and reset the output
	oOutLength.x = 0.0;
	oOutLength.y = 0.0;
	oOutLength.z = 0.0;

	//if the boundary length has been computed
	if(m_bEdgeFlag){
		
		oOutLength.x = m_oEdgeLength.x;
		oOutLength.y = m_oEdgeLength.y;
		oOutLength.z = m_oEdgeLength.z;
		return true;
	
	}else{
		
		return false;

	}//end else

}


/*************************************************
Function: InitialLoopParams
Description: limit the calculated depth of counting box number
Calls: none
Called By: class object or HausdorffDimension
Table Accessed: none
Table Updated: none
Input: f_iIterMax - stop at f_iIterMax times iteration to count box number
	   f_iIterMin - begin at f_iIterMin times iteration to count box number
Output: m_iIterMax
	    m_iIterMin
Return: none
Others: f_iIterMax and f_iIterMin is to determine the measuring scale this method would use
        the measuring scale would be about f_iIterMax-th and f_iIterMin-th power of the side length
        this funtion is to defend the overcomputation of box counting
*************************************************/
void HausdorffDimension::InitialLoopParams(const int & f_iIterMax,
	                                       const int & f_iIterMin){

	m_iIterMax = f_iIterMax;
	m_iIterMin = f_iIterMin;

	//the input should be positive integer numbers
	if(m_iIterMax < 0 || m_iIterMin < 0 || m_iIterMax - m_iIterMin < 0){
		std::cout<<" Error: It does not make scene if the scale is equal to 0 or smaller than 0. "<<std::endl;
		std::cout<<" Please choose any measuring scale and try again."<<std::endl;
		std::cin.get();
		exit(0);
	}

}

/*************************************************
Function: BoxCounting
Description: the box-counting method is used to approximate the Hausdorff dimension since Hausdorff dimension is not easy to be obtained in mormal 
             the box counting method counts the box numbers (non-empty voxel numbers) in different scale,
			 and then fits a straight line (trend), the slope of this fitted line is measured dimension.
Calls: GetBoundingLength - need to compute the length of bounding box
       FindMaximum - need find the maximum length to construct the bounding box
	   LinearFitting - need fit the target line (find the trend in spectrum)
	   Round - rounding a value 
Called By: class object 
Table Accessed: none
Table Updated: none
Input: vCloud - a input point clouds that need to be measured
Output: fDimensionRes - an approximate Hausdorff dimension result
Return: fDimensionRes
Others: the result of Hausdorff dimension depends on the given measuring scale (Observation scale)
        Hausdorff dimension only guarantees uniqueness in infinitesimal values (Scale-independent)
		Therefore, the essence of this method is the interception of dimensions in given scale range.
*************************************************/
float HausdorffDimension::BoxCounting(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	//define output as zero (the truth Hausdorff dimension of any point set)
	float fDimensionRes = 0.0;

	//a vector to save the result of scale
	std::vector<HitsInScale> vSpectrum;

	//define the bounding box as a cube with the largest size
	std::vector<float> vBoundLengths = GetBoundingLength(vCloud);
	
	//get the largest size of bounding box
	m_fBoundBoxLen = FindMaximum(vBoundLengths);

	//defend debug caused by precision problem
	m_fBoundBoxLen = m_fBoundBoxLen * (1.0f + m_fRedundancy);

	//if specified scale is required
	if(m_bScaleFlag){
		
		m_iIterMin = Round(log10(m_fLargeScale/m_fBoundBoxLen)/log10(0.5));
		
		//Prevent the side length of the box from being smaller than the given length
	    if(m_fBoundBoxLen == 0 || m_fBoundBoxLen < m_fSmallScale)
			m_fBoundBoxLen = m_fSmallScale;
		
		m_iIterMax = Round(log10(m_fSmallScale/m_fBoundBoxLen)/log10(0.5));
	
	}
	
	//whether using computed minimum scale
	if(m_bMinDisFlag){
		
        //prevent dividing zero
		if(m_fBoundBoxLen == 0 || m_fBoundBoxLen < m_fMinDis)
			m_fBoundBoxLen = m_fMinDis;
		//Base change formula is to compute the loop
		m_iIterMax = floor(log10(m_fMinDis/m_fBoundBoxLen)/log10(0.5));
		
	}

	//if the bounding box is still be zero
	//this situation indicates that the input point set is empty or has only one point
	//therefore, a zero measured result would be output 
	if(!m_fBoundBoxLen)
		return fDimensionRes;
    
	//*****************major part******************
	float boxsize = m_fBoundBoxLen / pow(2.0f,float(m_iIterMax));
	float boxnumber = 0;
	
	//The total number of boxes, in fact, the number of boxes is equal to the resolution
    int xnumber = (int)(pow(2.0f,float(m_iIterMax)));
	//save boxes
	std::vector<std::vector<std::vector<int>>> downbox;//root box
	std::vector<std::vector<std::vector<int>>> upbox;//

	for(int i = 0; i != xnumber; ++i){

		std::vector<std::vector<int>> boxy;
		for(int j = 0; j != xnumber; ++j){
			//initial as zero
			std::vector<int> boxz(xnumber,0);
			boxy.push_back(boxz);
		}
		downbox.push_back(boxy);
	} 

	//the number of non-empty boxes
	int xvalue,yvalue,zvalue;
	for(int i = 0; i != vCloud.size(); ++i){

		xvalue = floor((vCloud[i].x - m_oMinCoor.x) / boxsize);
		yvalue = floor((vCloud[i].y - m_oMinCoor.y) / boxsize);
		zvalue = floor((vCloud[i].z - m_oMinCoor.z) / boxsize);
		downbox[xvalue][yvalue][zvalue]++;

	}

	//using generalized dimension pi^q
	for(int i = 0; i != xnumber; ++i){
		for(int j = 0; j != xnumber; ++j){
			for(int k = 0; k != xnumber; ++k){

				if(downbox[i][j][k])
					boxnumber=boxnumber + pow(float(downbox[i][j][k]),m_fParaQ);
			}
		}
	}

	//store the first pair of results, followers are stored in a loop
	HitsInScale oFirstScaleRes;
	oFirstScaleRes.boxNum = log(float(boxnumber));
	oFirstScaleRes.boxScale = log(boxsize);
	vSpectrum.push_back(oFirstScaleRes);
	
	//current iterion time
	int iters = m_iIterMax - 1;

	//counting boxes at different scale using a loop 
	//the smaller the scale be used, the smaller the box becames
	//it does not make scene when the measuring scale is larger than the length of bounding box
	while(iters - m_iIterMin + 1 > 0){
		
		boxnumber = 0;
		boxsize = boxsize * 2.0;
		// use a structure similiar to octree so that calculations can be reduced
		// set the size of the new upbox
		upbox.clear();
		int ocnumber = (int)(pow(2.0f,float(iters)));
		
		//
		if(ocnumber < 1)
			ocnumber = 1;
		
		//Traversing 
		for(int i = 0;i != ocnumber; ++i){
			
			std::vector<std::vector<int>> boxy;
			//initial as zero
			for(int j = 0;j != ocnumber; ++j){
				std::vector<int> boxz(ocnumber,0);
				boxy.push_back(boxz);
			}
			
			upbox.push_back(boxy);
		} 
		
		//Traversing 
		//compare the position of the lower level and assign result to the superior level
		for(int i = 0;i != downbox.size(); ++i){
			
			for(int j = 0;j != downbox.size(); ++j){
				
				for(int k = 0; k != downbox.size(); ++k){

					if(downbox[i][j][k])
						//merge low level boxes and accumulate non-empty quantity
						upbox[floor(float(i)/2)][floor(float(j)/2)][floor(float(k)/2)]
					     = upbox[floor(float(i)/2)][floor(float(j)/2)][floor(float(k)/2)] + downbox[i][j][k];
				}//end k
			}//end j
		}//end i
		
		//counting
		for(int i = 0;i != upbox.size(); ++i){
			
			for(int j = 0;j != upbox.size(); ++j){
				
				for(int k = 0;k != upbox.size(); ++k){
					
					if(upbox[i][j][k])
						boxnumber = boxnumber + pow(float(upbox[i][j][k]),m_fParaQ);
				}
			}
		}
		
		//get the log value
		HitsInScale oScaleRes;
		oScaleRes.boxNum = log(float(boxnumber));
		oScaleRes.boxScale = log(boxsize);
	
		vSpectrum.push_back(oScaleRes);
		
		//Turn the subordinate boxes into the current boxes
		downbox.clear();
		downbox=upbox;

		iters--;
	}

    //fitting by using the least squares
	//q = 1 or q = 2
	//get the slope value of the fitted line
	fDimensionRes = LinearFitting(vSpectrum) / (m_fParaQ - 1.0);

	//output
	return fDimensionRes;

}

/*************************************************
Function: LinearFitting
Description: Linear fitting of two-dimensional lines
Calls: nothing
Called By: BoxCounting
Table Accessed: none
Table Updated: none
Input: vSpectrum - a sample vector, where boxNum is y and boxScale is x, respectively
Output: fKPara - the slope of fitting line
Return: fKPara
Others: none
*************************************************/
float HausdorffDimension::LinearFitting(std::vector<HitsInScale> & vSpectrum){
	
	//set y = kx + b as the target line to be fitted

	float iSpNum = float(vSpectrum.size());
	float fSumX = 0.0;
	float fSumY = 0.0;
	float fSumXY = 0.0;
	float fSumXX = 0.0;

    //Accumulate samples
	for (size_t i = 0; i < vSpectrum.size(); ++i){
		
		fSumX += vSpectrum[i].boxScale;//x is the scale value
		fSumY += vSpectrum[i].boxNum;//y is the non-empty box number
		fSumXY += vSpectrum[i].boxScale * vSpectrum[i].boxNum;
		fSumXX += vSpectrum[i].boxScale * vSpectrum[i].boxScale;
	}
	
	//prepare for k
	float fMeanX = fSumX / iSpNum;
	float fMeanY = fSumY / iSpNum;
	
	//compute the slope k
	//k = (n*sum(xy)-sum(x)sum(y))/(n*sum(x^2)-sum(x)^2)
	
	//compute the denominator to check whether the denominator is zero 
	float fKPara = fSumXX - iSpNum * fMeanX * fMeanX;
	
	//if denominator is non-zero 
	if(fKPara)
		fKPara = (fSumXY - iSpNum * fMeanX * fMeanY) / fKPara;
	else 
		fKPara = 0;
 
    return fKPara;

};

/*************************************************
Function: GetBoundingLength
Description: Output the length of bounding box
Calls: nothing
Called By: BoxCounting
Table Accessed: none
Table Updated: none
Input: vCloud - a input point clouds
Output: m_oEdgeLength - the maximum length of bounding box at each axis
        vBoundLengths - the same with m_oEdgeLength
Return: vBoundLengths
Others: none
*************************************************/
std::vector<float> HausdorffDimension::GetBoundingLength(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	//check whether the maximum and minimum coordinate value of points is known  
	//if they are unknown
    if(!m_bMaxMinCoFlag){
		
		//reset minimum and maximum of coordinate value at each axis 
		m_oMinCoor.x = FLT_MAX;
		m_oMinCoor.y = FLT_MAX;
		m_oMinCoor.z = FLT_MAX;
		
		m_oMaxCoor.x = -FLT_MAX;
		m_oMaxCoor.y = -FLT_MAX;
		m_oMaxCoor.z = -FLT_MAX;

		//record the coordinate ranges of input point set 
		for(int i = 0; i != vCloud.size(); ++i){
			
			if(vCloud[i].x < m_oMinCoor.x) 
				m_oMinCoor.x = vCloud[i].x;
			
			if(vCloud[i].y < m_oMinCoor.y) 
				m_oMinCoor.y = vCloud[i].y;
			
			if(vCloud[i].z < m_oMinCoor.z) 
				m_oMinCoor.z = vCloud[i].z;
			
			if(vCloud[i].x > m_oMaxCoor.x) 
				m_oMaxCoor.x = vCloud[i].x;
			
			if(vCloud[i].y > m_oMaxCoor.y) 
				m_oMaxCoor.y = vCloud[i].y;
			
			if(vCloud[i].z > m_oMaxCoor.z) 
				m_oMaxCoor.z = vCloud[i].z;
		
		}//end for i != vCloud.size()

	}//end if

	//compute the bounding box length at each axis
	m_oEdgeLength.x = m_oMaxCoor.x - m_oMinCoor.x;///<x
	m_oEdgeLength.y = m_oMaxCoor.y - m_oMinCoor.y;///<y
	m_oEdgeLength.z = m_oMaxCoor.z - m_oMinCoor.z;///<z

	//turn flag
	m_bEdgeFlag = true;

	//output 
	std::vector<float> vBoundLengths;
	vBoundLengths.push_back(m_oEdgeLength.x);
	vBoundLengths.push_back(m_oEdgeLength.y);
	vBoundLengths.push_back(m_oEdgeLength.z);
	return vBoundLengths;

}


/*************************************************
Function: ClearLength
Description: clear the length related parameters
Calls: nothing
Called By: class object
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: reset the parameters or variables which related to the input point clouds
*************************************************/
void HausdorffDimension::ClearLength(){
	
	//the parameters here is related to the input data
	//clear coordinate corner
	m_oMinCoor.x = FLT_MAX;
	m_oMinCoor.y = FLT_MAX;
	m_oMinCoor.z = FLT_MAX;	

    m_oMaxCoor.x = -FLT_MAX;
	m_oMaxCoor.y = -FLT_MAX;
	m_oMaxCoor.z = -FLT_MAX;

	//turn coordinate corner flag
	m_bMaxMinCoFlag = false;

	//clear edge length
	m_oEdgeLength.x = 0.0;
	m_oEdgeLength.y = 0.0;
	m_oEdgeLength.z = 0.0;
	m_fBoundBoxLen = 0.0;

	//turn edge flag
	m_bEdgeFlag = false;

}

/*************************************************
Function: :ClearAll
Description: clear all of parameters
Calls: nothing
Called By: class object
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: reset the all of parameters and variables 
*************************************************/
void HausdorffDimension::ClearAll(){

	//clear coordinate corner value
	m_oMinCoor.x = FLT_MAX;
	m_oMinCoor.y = FLT_MAX;
	m_oMinCoor.z = FLT_MAX;	

    m_oMaxCoor.x = -FLT_MAX;
	m_oMaxCoor.y = -FLT_MAX;
	m_oMaxCoor.z = -FLT_MAX;
	//turn coordinate flag
	m_bMaxMinCoFlag = false;

	//clear boundary length value
	m_oEdgeLength.x = 0.0;
	m_oEdgeLength.y = 0.0;
	m_oEdgeLength.z = 0.0;

	//clear maximum length of those three above
	m_fBoundBoxLen = 0.0;
	
	//turn edge flag
	m_bEdgeFlag = false;

	//reset generalized dimensional parameter
	m_fParaQ = 0;

	//clear scale as nothing input
	m_fLargeScale = 0.0;
	m_fSmallScale = 0.0;
	m_bScaleFlag = false;

	//clear the minimum measure distance
	m_fMinDis = 0.0;
	m_bMinDisFlag = false;

}

