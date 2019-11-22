#ifndef READTXT_H
#define READTXT_H
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>//for istringstream function
using namespace std;
/*================================================
read orignal matrix
=================================================*/
void ReadMatrix(string fileName,std::vector<std::vector<double>> & feamatrix,int n=1);
//对轨迹线进行抽样处理
//实际中，由于轨迹线记录的是每5毫秒的位置，假设汽车速度30km/h，则8米/秒，一米125毫秒即25次间隔
void Sampling(std::vector<std::vector<double>> & feamatrix,int n=25);
/*================================================

=================================================*/
class Dividefeandclass{
public:
	//Classification at the last position 
	void Extlabelatend(std::vector<std::vector<double>> & , std::vector<int> &  );
	//Classification at the first position 
	void Extlabelatbegin(std::vector<std::vector<double>> &, std::vector<int> &  );
	void Extlabelatbegin(std::vector<std::vector<double>> &, std::vector<double> &  );
	//delete some feature
	void Removesomefeature(std::vector<std::vector<double>> &, std::vector<int> & );
};

#endif