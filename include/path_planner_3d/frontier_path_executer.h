#ifndef FRONTIER_PATH_EXECUTER_H_
#define FRONTIER_PATH_EXECUTER_H_

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <time.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>





class FrontierPathExecuter {
public:

	FrontierPathExecuter();

	void pathCallback(const nav_msgs::Path& path);
	size_t distCalculation();

	void step();



private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber path_sub;
  ros::Publisher 	move_pub;

	nav_msgs::Path path_;
	bool pointReached;
	bool moving;

	tf::TransformListener tfListener_;
	tf::StampedTransform map2bf;

	geometry_msgs::Pose basePose;
};

#endif
