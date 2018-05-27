#include "path_planner_3d/frontier_explorer.h"

FrontierExplorer::FrontierExplorer():
private_nh_("~"),cost_map_publisher_(&nh_,&cost_map_,"/map","/costmap_auto",true),
debugcost_map_publisher_(&nh_,&debugcost_map_,"/map","/debugcostmap_auto",true),
baseFrameId_("base_footprint"){
	map_sub	= nh_.subscribe("/map", 1, &FrontierExplorer::mapCallback,this);
	global_map_updates_sub	= nh_.subscribe("/move_base/global_costmap/costmap_updates", 1, &FrontierExplorer::globalMapUpdatesCallback,this);
	global_map_sub	= nh_.subscribe("/move_base/global_costmap/costmap", 1, &FrontierExplorer::globalMapCallback,this);
	vel_sub = nh_.subscribe("/mobile_base/commands/velocity", 1, &FrontierExplorer::velCallback,this);
	vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1,true);
	path_pub = nh_.advertise<nav_msgs::Path>("/frontier_path",1,true);
	global_costmap_ready = false;
	map_ready = false;
	vel_null = false;
	unstack_mode = false;
	nearestP.x = 0.0;
	nearestP.y = 0.0;
}

void
FrontierExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
	resolution = map->info.resolution;
	originMap.x =  map->info.origin.position.x;
	originMap.y =  map->info.origin.position.y;
	grid2CostMap(*map,cost_map_);
	grid2CostMap(*map,debugcost_map_);
	map_ready = true;
}

void
FrontierExplorer::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
	grid2CostMap(*map,global_costmap);
	global_costmap_ready = true;
}

void
FrontierExplorer::globalMapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& map){
	float normalizer = 2.54;
	int i = 0;
	for(int y=map->y; y< map->y+map->height; y++){
  	for(int x=map->x; x< map->x+map->width; x++){
			global_costmap.setCost(x,y,round(normalizer*map->data[i++]));
		}
	}
}

void
FrontierExplorer::velCallback(const geometry_msgs::Twist::ConstPtr& vel){
	vel_null = (vel->linear.x == 0.0 && vel->angular.z == 0.0);
	timer	=	nh_.createTimer(ros::Duration(5),&FrontierExplorer::timerCallback, this, true);
}

void
FrontierExplorer::timerCallback(const ros::TimerEvent&){
	if(vel_null){
		unstack_mode = true;
	}
}

void
FrontierExplorer::timerUnstackCallback(const ros::TimerEvent&){
	unstack_mode = false;
}


void
FrontierExplorer::grid2CostMap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D& cost_map){
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
	geometry_msgs::Point p;
	for(int y=0;y<cost_map_.getSizeInCellsY();y++){
		for(int x=0;x<cost_map_.getSizeInCellsX();x++){
			if(cost_map_.getCost(x,y) == 0 && isFrontier(x,y)){
				p.x = x;
				p.y = y;
				if(isFrontierPointFree(p)){
					debugcost_map_.setCost(x,y,50);
					frontierPList.push_back(p);
					setNearestPoint(p);
				}
			}
		}
	}
}

bool
FrontierExplorer::isFrontier(int x, int y){
	return (cost_map_.getCost(x+1,y) == 255 || cost_map_.getCost(x+1,y+1) == 255 ||
	cost_map_.getCost(x,y+1) == 255 || cost_map_.getCost(x-1,y+1) == 255 ||
	cost_map_.getCost(x-1,y) == 255 || cost_map_.getCost(x-1,y-1) == 255 ||
	cost_map_.getCost(x,y-1) == 255 || cost_map_.getCost(x+1,y-1) == 255);
}

bool
FrontierExplorer::isNeighbor(geometry_msgs::Point p1,geometry_msgs::Point p2){
	geometry_msgs::Point p;
	p.x = p2.x - p1.x;
	p.y = p2.y - p1.y;
	return (sqrt(powf(p.x, 2) + powf(p.y, 2)) < 1.5);
}

bool
FrontierExplorer::isFrontierPointFree(geometry_msgs::Point p_in){
	unsigned int x_costmap,y_costmap;
	geometry_msgs::Point p;
	p.x = p_in.x * resolution + originMap.x;
	p.y = p_in.y * resolution + originMap.y;
	global_costmap.worldToMap(p.x,p.y,x_costmap,y_costmap);
	unsigned int cost = global_costmap.getCost(x_costmap,y_costmap);
	return (cost < 150);
}

void
FrontierExplorer::setNearestPoint(geometry_msgs::Point p_in){
	tf::Stamped<tf::Point> p_tf,point_bf;
	geometry_msgs::Point p;
	if(tfListener_.canTransform("/map", baseFrameId_, ros::Time(0))){
		p.x = p_in.x * resolution + originMap.x;
		p.y = p_in.y * resolution + originMap.y;
		p_tf.setX(p.x);
		p_tf.setY(p.y);
		p_tf.setZ(0.0);
		p_tf.setW(1.0);
		p_tf.frame_id_ = "/map";
		tfListener_.transformPoint(baseFrameId_, p_tf, point_bf);
		float d1 = sqrt(powf(nearestP.x, 2) + powf(nearestP.y, 2));
		float d2 = sqrt(powf(point_bf.getX(), 2) + powf(point_bf.getY(), 2));
		if(d1 == 0.0 || d2<d1){
			nearestP=p;
		}
	}
}

void
FrontierExplorer::getRefFrontierPoints(){
	geometry_msgs::PoseStamped pPath;
	geometry_msgs::Point p,p_cell;
	std::list<geometry_msgs::PoseStamped> pList;
	int num_points = 0;

	for (std::list<geometry_msgs::Point>::iterator itList=frontierPList.begin(); itList!=frontierPList.end(); ++itList){
		p_cell.x = p_cell.x + itList->x;
		p_cell.y = p_cell.y + itList->y;
		num_points++;
	}
	p_cell.x = p_cell.x / num_points;
	p_cell.y = p_cell.y / num_points;
	p.x = p_cell.x * resolution + originMap.x;
	p.y = p_cell.y * resolution + originMap.y;
	path.poses.resize(1);
	if(unstack_mode){
		ROS_WARN("Unstack_mode ON");
		pPath.pose.position = nearestP;
		timer_unstack	=	nh_.createTimer(ros::Duration(8),&FrontierExplorer::timerUnstackCallback, this, true);
		addLocationVis(1,pPath.pose.position,0.0,0.0,1.0,1.0);
	}else{
		pPath.pose.position = p;
		addLocationVis(1,pPath.pose.position,0.0,1.0,0.0,1.0);
	}
	path.poses[0] = pPath;
}

void
FrontierExplorer::addLocationVis(int id, geometry_msgs::Point point,float r, float g, float b,float alpha){
	visualization_msgs::Marker marker;
	geometry_msgs::	Pose p;
	marker.header.frame_id = "/map";
	marker.ns = "locations";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position = point;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = alpha;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	nodes_vis_.markers.push_back(marker);
}

void
FrontierExplorer::publishAll(){
	path.header.frame_id = "/map";
	path_pub.publish(path);
	vis_pub_.publish(nodes_vis_);
	cost_map_publisher_.publishCostmap();
	debugcost_map_publisher_.publishCostmap();
}

void
FrontierExplorer::step(){
	if(!global_costmap_ready || !map_ready){
		return;
	}
	frontierDetector();
	getRefFrontierPoints();
	publishAll();
	frontierPList.clear();
	path.poses.clear();
	nodes_vis_.markers.clear();
	nearestP.x = 0.0;
	nearestP.y = 0.0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_explorer");
	FrontierExplorer frontier_explorer;
	ros::Rate loop_rate(5);
  	while (ros::ok()){
  		frontier_explorer.step();
  		ros::spinOnce();
  		loop_rate.sleep();
  	}
  return 0;
}
