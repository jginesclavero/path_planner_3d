#include "path_planner_3d/csv2Octomap.h"
#include <fstream>

namespace csv_2_octomap {

	Csv2Octomap::Csv2Octomap():nh_("~"),
	cost_map(),
	octree(NULL){

		map_res = 0.1;
		voxel_res = 0.1;
		cells_size_x = 100; //Habrá que verlo
		cells_size_y = 100; //Habrá que verlo
		origin_x = 0.0; //Habrá que verlo
		origin_y = 0.0; //Habrá que verlo
		default_value = 0;
		sizeMapX=0;
		sizeMapY=0;

		double probHit, probMiss, thresMin, thresMax;
		probHit = 0.7;probMiss=0.4;thresMin=0.12;thresMax=0.97;

		octomap_pub = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1,false);
		cost_map.resizeMap(cells_size_x,cells_size_y, map_res, origin_x, origin_y);
		cost_map.setDefaultValue(default_value);
		cost_map.resetMap(0,0,cost_map.getSizeInCellsX(), cost_map.getSizeInCellsY());

		octree = new octomap::OcTree(voxel_res);
	  octree->setProbHit(probHit);
	  octree->setProbMiss(probMiss);
	  octree->setClampingThresMin(thresMin);
	  octree->setClampingThresMax(thresMax);
		//pasar de csv a costmap
		transform2DtoOct(cost_map);
		publishFullOctoMap();
	}

	void Csv2Octomap::csv2CostMap(std::string fileName){
		 		std::ifstream file;

				file.open(fileName);
				if( ! file.is_open() )
				{
							printf("Not able to open file: %s\n",fileName.c_str());
				}
				else{
					printf("Opened file: %s\n",fileName.c_str());

					std::vector<int> intsOnFileLine;
					bool endFile=false;
					while(!endFile){

						intsOnFileLine.clear();

						if( file.eof() ){
								endFile = true;
								continue;
						}

						std::string s;
						getline(file, s);
						std::stringstream ss(s);

						int d;
						while (ss >> d){
								intsOnFileLine.push_back(d);
								sizeMapX++;
						}
						intMap.push_back( intsOnFileLine );
						endFile = false;
						sizeMapY++;
					}
					sizeMapY=/map_res;
					sizeMapX=/map_res;

					file.close();

					cost_map.resizeMap(sizeMapX,sizeMapY, map_res, origin_x, origin_y);

					for(int i=0; i< sizeMapX*map_res; i++){
							for(int j=0; i< sizeMapY*map_res; j++){
									for(int cell=0; cell<9;cell++){
										cost_map.setCost((i+cell),(j+cell),intMap[i][j]*244);
									}
							}
					}
			}
		}


	void
	Csv2Octomap::transform2DtoOct(costmap_2d::Costmap2D cost_map){
		octomap::KeySet occupied_cells;
		octomap::OcTreeKey key;
		octomap::KeyRay keyRay;
		int cost_now;
		for(unsigned int y = 0; y < cost_map.getSizeInCellsY(); y++) {
			for(unsigned int x = 0; x < cost_map.getSizeInCellsX(); x++) {
				cost_now = cost_map.getCost(x,y);
				if(cost_now > 0){
					octomap::point3d point(x*map_res, y*map_res, 0);
					octomap::point3d point_end(x*map_res,y*map_res,2.5);
					if (octree->computeRayKeys(point, point_end, keyRay)){
			      occupied_cells.insert(keyRay.begin(), keyRay.end());
			    }
				}
			}
		}

		for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
				octree->updateNode(*it,true);
				octree->setNodeValue(*it,1,true);
	  }
	}

	void
	Csv2Octomap::publishFullOctoMap(){
		octomap_msgs::Octomap map;
		if (octomap_msgs::fullMapToMsg(*octree, map)){
			map.header.frame_id = "map";
			map.header.stamp = ros::Time::now();
			octomap_pub.publish(map);
			ROS_DEBUG("ShortTermMap published");
		}else
			ROS_ERROR("Error serializing OctoMap");
	}



}

using namespace csv_2_octomap;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "path_planner_node");
   ros::NodeHandle n;
   Csv2Octomap csv_2_octomap;

   ros::spin();
   return 0;

 }
