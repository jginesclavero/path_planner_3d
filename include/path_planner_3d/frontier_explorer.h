#ifndef FRONTIEREXPLORER_H_
#define FRONTIEREXPLORER_H_

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>



class FrontierExplorer {
public:

	FrontierExplorer();

	virtual void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& short_map);
	virtual void step();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber map_sub;
	ros::Publisher 	frontierMap_pub;
	costmap_2d::Costmap2D cost_map;
	costmap_2d::Costmap2DPublisher cost_map_publisher_;

	void grid2CostMap(nav_msgs::OccupancyGrid map);
	void frontierDetector();
	void frontierClass();
	bool isFrontier(int x, int y);
	bool isEndFrontier(geometry_msgs::Point p);
	std::list<geometry_msgs::Point> frontierPList;

	//nav_msgs::OccupancyGrid static_map,longTerm_map,shortTerm_map,effective_map;

	void publishAll();
};

#endif
