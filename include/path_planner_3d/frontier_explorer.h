#ifndef FRONTIEREXPLORER_H_
#define FRONTIEREXPLORER_H_

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include <algorithm>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

class FrontierExplorer {
public:

	FrontierExplorer();

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& short_map);
	void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
	void globalMapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& map);
	void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
	void timerCallback(const ros::TimerEvent&);
	void timerUnstackCallback(const ros::TimerEvent&);
	virtual void step();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber map_sub,global_map_sub,global_map_updates_sub, vel_sub;
	ros::Publisher frontierMap_pub,vis_pub_,path_pub;
	costmap_2d::Costmap2D cost_map_,global_costmap,debugcost_map_;
	costmap_2d::Costmap2DPublisher cost_map_publisher_,debugcost_map_publisher_;
	std::list<geometry_msgs::Point> frontierPList;
	visualization_msgs::MarkerArray nodes_vis_;
	float resolution;
	geometry_msgs::Point originMap,nearestP;
	nav_msgs::Path path;
	bool global_costmap_ready,map_ready,vel_null,unstack_mode;
	ros::Timer timer,timer_unstack;
	tf::TransformListener tfListener_;
	std::string baseFrameId_;

	void grid2CostMap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D& cost_map);
	void frontierDetector();
	void frontierClass();
	bool isFrontier(int x, int y);
	bool isNeighbor(geometry_msgs::Point p1,geometry_msgs::Point p2);
	void getRefFrontierPoints();
	void addLocationVis(int id, geometry_msgs::Point point,float r, float g, float b,float alpha);
	bool isFrontierPointFree(geometry_msgs::Point p_in);
	void setNearestPoint(geometry_msgs::Point p);
	void publishAll();
};

#endif
