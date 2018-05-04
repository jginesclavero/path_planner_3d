#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

std::vector<std::vector<int> > intMap; //For CSV to CostMap conversion
std::vector<std::vector<float> > costMap;
float sizeMapX=0, sizeMapY=0;
float map_res=0.1;
float cells=10;

void dump()
{
	printf("Size of the Map (X,Y): (%d,%d)\n", (int)intMap.size(),(int)intMap[0].size());

	for(int i=0;i<intMap.size();i++)
	{
		for(int j=0;j<intMap[0].size();j++)
		{
			printf("%d ",intMap[i][j]);
		}
		printf("\n");
	}
}


void dumpCostMap()
{
	std::ofstream file;
	file.open("out.csv");

	printf("Size of the Map (X,Y): (%d,%d)\n", (int)costMap.size(),(int)costMap[0].size());
	for(int i=0;i<costMap.size();i++)
	{
		for(int j=0;j<costMap[0].size();j++)
		{
			if(j<costMap[0].size()-1)
				file<<costMap[i][j]<<",";
			else
				file<<costMap[i][j];
		}
		file<<"\n";
	}

	file.close();

}

void csv2CostMap(std::string fileName){
	std::ifstream file;
	file.open(fileName.c_str());

	if(!file.is_open() ){
		ROS_ERROR("Not able to open file: %s",fileName.c_str());
	}else{
		ROS_INFO("Opened file: %s",fileName.c_str());

		std::vector<int> intsOnFileLine;
		bool endFile=false;
		while(!endFile){

			intsOnFileLine.clear();
			std::string s;
			std::getline(file, s);
			std::stringstream ss(s);

			if( file.eof() ){
				endFile = true;
				continue;
			}

			string val;
			while (getline(ss, val,',')){
				intsOnFileLine.push_back((float)stoi(val));
			}
			intMap.push_back(intsOnFileLine);

		}
		file.close();

		sizeMapX=intMap.size()/map_res;
		sizeMapY=intMap[0].size()/map_res;

		//Create CostMap vectors
		costMap.resize(sizeMapX);
		for(int i=0; i<sizeMapX;i++){
			costMap[i].resize(sizeMapY);
		}
		cout<<"CostMap_Size (X,Y): ("<<costMap.size()<<","<<costMap[0].size()<<")"<<endl;

		for(int i=0; i< sizeMapX*map_res; i++){
			for(int j=0; j< sizeMapY*map_res; j++){
				for(int cellY=0; cellY<cells;cellY++){
					for(int cellX=0; cellX<cells;cellX++){
						costMap[i*cells + cellX][j*cells + cellY] = intMap[i][j];
					}
				}
			}
		}
	}
}


int main(int argc, char* argv[]) {

	std::string filename(argv[1]);
	cout<<"Filename: "<<filename<<endl;

	csv2CostMap(filename);
	dump();
	dumpCostMap();

	return 0;
}
