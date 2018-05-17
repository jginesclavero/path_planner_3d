#include "path_planner_3d/frontier_explorer.h"

FrontierExplorer::FrontierExplorer():
private_nh_("~"),cost_map_publisher_(&nh_,&cost_map,"/map","/costmap_auto",true){
	map_sub	= nh_.subscribe("/map", 5, &FrontierExplorer::mapCallback,this);
	//longTermMap_pub	 	= nh_.advertise<nav_msgs::OccupancyGrid>("/longTerm_map", 5);
	//effectiveMap_pub	= nh_.advertise<nav_msgs::OccupancyGrid>("/map", 5); //Effective_map

	/*effective_map.info.resolution = longTerm_map.info.resolution;
	effective_map.info.width = longTerm_map.info.width;
	effective_map.info.height = longTerm_map.info.height;
	effective_map.info.origin = longTerm_map.info.origin;
	effective_map.data.resize(longTerm_map.info.width * longTerm_map.info.height);*/
}



void
FrontierExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
	grid2CostMap(*map);
}

void
FrontierExplorer::grid2CostMap(nav_msgs::OccupancyGrid map){
	cost_map.resizeMap(map.info.width,map.info.height, map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);
	cost_map.setDefaultValue(254);
	int i = 0;
	float normalizer = 2.54;
	for(int y=0;y<cost_map.getSizeInCellsY();y++){
		for(int x=0;x<cost_map.getSizeInCellsX();x++){
			if(map.data[i] == -1)
				cost_map.setCost(x,y,255);
			else
				cost_map.setCost(x,y,round(normalizer*map.data[i]));
			i++;
		}
	}
}

void
FrontierExplorer::frontierDetector(){
	for(int y=0;y<cost_map.getSizeInCellsY();y++){
		for(int x=0;x<cost_map.getSizeInCellsX();x++){
			if(cost_map.getCost(x,y) == 0 && isFrontier(x,y)){
				cost_map.setCost(x,y,50);
				geometry_msgs::Point p;
				p.x = x;
				p.y = y;
				frontierPList.push_back(p);
			}
		}
	}
}

void
FrontierExplorer::frontierClass(){
	ROS_INFO("frontierClass");
	for (std::list<geometry_msgs::Point>::iterator it=frontierPList.begin(); it != frontierPList.end(); ++it){
		if(isEndFrontier(*it)){
			cost_map.setCost(it->x,it->y,150);
		}

	}
}


bool
FrontierExplorer::isFrontier(int x, int y){
	return (cost_map.getCost(x+1,y) == 255 || cost_map.getCost(x-1,y) == 255 ||
	 cost_map.getCost(x,y+1) == 255 || cost_map.getCost(x,y-1) == 255);
}

bool
FrontierExplorer::isEndFrontier(geometry_msgs::Point p){
	return (cost_map.getCost(p.x+1,p.y) == 254 || cost_map.getCost(p.x+1,p.y+1) == 254 ||
		cost_map.getCost(p.x,p.y+1) == 254 || cost_map.getCost(p.x-1,p.y+1) == 254 ||
		cost_map.getCost(p.x-1,p.y) == 254 || cost_map.getCost(p.x-1,p.y-1) == 254 ||
		cost_map.getCost(p.x,p.y-1) == 254 || cost_map.getCost(p.x+1,p.y-1) == 254);
}

/*

void
FrontierExplorer::updateLongTermMap(nav_msgs::OccupancyGrid s_map){
	for(int i = 0;i<s_map.data.size();i++){
		if(s_map.data[i] > 95){
			//longTerm_map.data[i] = s_map.data[i];
			if(longTerm_map.data[i] <= 100 - longterm_cost_inc){
				longTerm_map.data[i]= longTerm_map.data[i] + longterm_cost_inc;
			}else{
				longTerm_map.data[i] = 100;
			}
		}else if(s_map.data[i] <= 25 && s_map.data[i] >= 0 && static_map.data[i] < 95){
			if(longTerm_map.data[i] >= longterm_cost_dec){
				longTerm_map.data[i] = longTerm_map.data[i] - longterm_cost_dec;
			}else{
				longTerm_map.data[i] = 0;
			}
		}
	}
}*/
/*
void
FrontierExplorer::buildEffectiveMap(nav_msgs::OccupancyGrid s_map,nav_msgs::OccupancyGrid l_map){
	for(int i = 0;i<s_map.data.size();i++){
		effective_map.data[i] = std::max(static_map.data[i],l_map.data[i]);
	}
}*/
void
FrontierExplorer::publishAll(){
	cost_map_publisher_.publishCostmap();
}


void
FrontierExplorer::step(){
	/*if(map_ready){
		buildEffectiveMap(shortTerm_map,longTerm_map);
		updateLongTermMap(shortTerm_map);

	}*/
	frontierDetector();
	frontierClass();
	publishAll();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_explorer");		//Inicializa el nodo
	FrontierExplorer frontier_explorer;
	ros::Rate loop_rate(20);
  	while (ros::ok()){
  		frontier_explorer.step();
  		ros::spinOnce();
  		loop_rate.sleep();
  	}
  return 0;
}
