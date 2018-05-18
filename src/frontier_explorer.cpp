#include "path_planner_3d/frontier_explorer.h"

FrontierExplorer::FrontierExplorer():
private_nh_("~"),cost_map_publisher_(&nh_,&cost_map,"/map","/costmap_auto",true){
	map_sub	= nh_.subscribe("/map", 5, &FrontierExplorer::mapCallback,this);
	vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 10);
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
	resolution = map->info.resolution;
	originMap.x =  map->info.origin.position.x;
	originMap.y =  map->info.origin.position.y;
	grid2CostMap(*map);
}

void
FrontierExplorer::grid2CostMap(nav_msgs::OccupancyGrid map){
	cost_map.resizeMap(map.info.width,map.info.height, map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);
	ROS_INFO("origin x: %f y:%f",map.info.origin.position.x, map.info.origin.position.y);
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
	//ROS_INFO("----------------------------------------");
	geometry_msgs::Point pLast,pEnd;
	int idMap = 0;
	std::list<geometry_msgs::Point> points;
	for (std::list<geometry_msgs::Point>::iterator it=frontierPList.begin(); it != frontierPList.end(); ++it){
		if(isEndFrontier(*it)){
			if(pEnd.x == 0.0 && pEnd.y == 0.0){
				//cost_map.setCost(it->x,it->y,idMap*50);
				pLast = *it;
				pEnd = *it;
				points.push_back(pEnd);
			}else{
				//cost_map.setCost(it->x,it->y,idMap*50);
				points.push_back(*it);
				frontierMap[idMap] = points;
				idMap++;
				pLast.x=0.0;
				pLast.y=0.0;
				pEnd = pLast;
				points.clear();
			}
		}else if(isNeighbor(*it,pLast) && pEnd.x != 0.0 && pEnd.y != 0.0){
			pLast = *it;
			points.push_back(pLast);
			//cost_map.setCost(it->x,it->y,idMap*50);
		}
		//ROS_INFO("P -- x: %f  y: %f",it->x,it->y);
	}
	//ROS_INFO("----------------------------------------");
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
bool
FrontierExplorer::isNeighbor(geometry_msgs::Point p1,geometry_msgs::Point p2){
	geometry_msgs::Point p;
	p.x = p2.x - p1.x;
	p.y = p2.y - p1.y;
	return (sqrt(powf(p.x, 2) + powf(p.y, 2)) < 1.5);
}

void
FrontierExplorer::frontierVis(){
	geometry_msgs::Point p;
	int count = 0;
	for (std::map<int,std::list<geometry_msgs::Point>>::iterator itMap=frontierMap.begin(); itMap!=frontierMap.end(); ++itMap){
		for (std::list<geometry_msgs::Point>::iterator it=itMap->second.begin(); it != itMap->second.end(); ++it){
			count++;
			//hay que hacer la trasnformada a MAP
			p.x = it->x * resolution + originMap.x;
			p.y = it->y * resolution + originMap.y;
			addLocationVis(count,p,0.0,0.0,itMap->first*0.25 + 0.25,1.0);
		}
	}
}


void
FrontierExplorer::addLocationVis(int id, geometry_msgs::Point point,float r, float g, float b,float alpha){
	visualization_msgs::Marker marker;
	geometry_msgs::	Pose p;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "locations";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position = point;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = alpha; // Don't forget to set the alpha!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	nodes_vis_.markers.push_back(marker);
}

void
FrontierExplorer::publishAll(){
	vis_pub_.publish(nodes_vis_);
	nodes_vis_.markers.clear();
	cost_map_publisher_.publishCostmap();
}

void
FrontierExplorer::step(){
	frontierDetector();
	frontierClass();
	frontierVis();
	publishAll();
	frontierPList.clear();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_explorer");		//Inicializa el nodo
	FrontierExplorer frontier_explorer;
	ros::Rate loop_rate(5);
  	while (ros::ok()){
  		frontier_explorer.step();
  		ros::spinOnce();
  		loop_rate.sleep();
  	}
  return 0;
}
