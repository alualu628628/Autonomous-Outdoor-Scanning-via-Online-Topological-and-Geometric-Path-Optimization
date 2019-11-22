#include"readtxt.h"

/*================================================
ReadMatrix 函数
功能 读取txt里面存储的矩阵
输入：文件名和要接盘的容器
输出：接盘的容器赋值
=================================================*/
void ReadMatrix(string fileName,std::vector<std::vector<double>> & feamatrix,int n){
	feamatrix.clear();
	//打开文件
  ifstream ifile(fileName.c_str(),ios::in);
  double tmp;//中间量
  string line;//一行的原始量
  vector<double> row;//一行的中间量
  //**编的烦起
  int linenum=0;
  //go
  while(getline(ifile,line)){//读取一行，一行一行给我读
       istringstream istr(line);//立马按空格把行撕成几段
	   //给一段是一段
       while(istr>>tmp){
       row.push_back(tmp);
       }//end small while
	   //整行中间量移交，简直不能忍
  if(!(linenum%n))//是否抽样，n=1不抽样（默认）
  feamatrix.push_back(row);
  row.clear();
  istr.clear();
  line.clear();
  linenum++;
  }//end big while
//过河拆桥
  ifile.close();
}

/*================================================
函数Extlabelatend
功能 分离变量中的特征和类别，这是由于变量的特征和类别一起读出
输入：接盘的特征和类别容器
注：原始特征容器里包含了类别为最后一个元素
输出：类别容器赋值，特征容器删除最后元素
=================================================*/
void Dividefeandclass::Extlabelatend(std::vector<std::vector<double>> & set_feature, 
	std::vector<int> & set_classes ){
		//遍历赋值
	for(size_t i=0;i!=set_feature.size();i++){
		set_classes.push_back(set_feature[i][set_feature[i].size()-1]);
		set_feature[i].pop_back();
	}//end for
}
/*================================================
函数Extlabelatbegin
功能 去除iccv内的序号列即第一列
输入：iccv带序号特征
输出：iccv_feature iccv特征
=================================================*/
void Dividefeandclass::Extlabelatbegin(std::vector<std::vector<double>> & set_feature,
	std::vector<int> & set_classes){
	//perfect
	vector<double>::iterator ite;
	for(size_t i=0;i!=set_feature.size();i++){
		ite=set_feature[i].begin();
		set_classes.push_back(*ite);
		set_feature[i].erase(ite);
		//over
	}//end for

}
void Dividefeandclass::Extlabelatbegin(std::vector<std::vector<double>> & set_feature,
	std::vector<double> & ext_feature){
	//perfect
	vector<double>::iterator ite;
	for(size_t i=0;i!=set_feature.size();i++){
		ite=set_feature[i].begin();
		ext_feature.push_back(*ite);
		set_feature[i].erase(ite);
		//over
	}//end for

}
/*================================================
函数Removesomefeature
功能 只保留几个特征
输入：特征向量vec_feature 及保留特征的索引retaindices
输出：新的特征向量vec_feature
=================================================*/
void Dividefeandclass::Removesomefeature(std::vector<std::vector<double>> &vec_feature, 
	std::vector<int> & retaindices){
    //一个中间变量
	std::vector<double> oneres(retaindices.size(),0.0);
	std::vector<std::vector<double>> midvec;
	//存储变量
	for(size_t i=0;i!=vec_feature.size();i++){
		for(size_t j=0;j!=retaindices.size();j++){
		oneres[j]=vec_feature[i][retaindices[j]];
		}//over
		midvec.push_back(oneres);
	}//end for
	vec_feature.clear();
	vec_feature=midvec;
}
/*================================================
函数Sampling
功能 抽样
输入：std::vector<std::vector<double>> & feamatrix double型的多维容器
      n 抽样数
输出：新的特征矩阵仍然保存在feamatrix
=================================================*/
void Sampling(std::vector<std::vector<double>> & feamatrix,int n){
	//perfect new
	std::vector<std::vector<double>> newfea;
	//count i
	for(int i=0;i!=feamatrix.size();i=i+n)
	newfea.push_back(feamatrix[i]);
	
	//rebuild feamatrix
	feamatrix.clear();
	feamatrix=newfea;
}
