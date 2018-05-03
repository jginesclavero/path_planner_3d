/*
 * obstacle_detector.h
 *
 *  Created on: 27/01/2016
 *      Author: Jonathan Ginés
 */

#ifndef OBSTACLEDETECTOR_H_
#define OBSTACLEDETECTOR_H_

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <list>

#include "dn_mapping/Metadata.h"


namespace obstacle_detector {


class ObstacleDetector {
public:

	ObstacleDetector();

	virtual void laserCallback(const  sensor_msgs::LaserScan::ConstPtr& scan_in);
	virtual void posCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos);
	virtual void step();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber scan_sub,pos_sub;
	ros::Publisher costmap_pub;
	costmap_2d::Costmap2D cost_map;
	float posRobot_x,posRobot_y,posRobot_w,max_lenght,min_lenght;
	int cost_inc,cost_dec;
	bool scan_ready,pos_ready;
	unsigned int cells_size_x, cells_size_y;
	double resolution, origin_x ,origin_y;
	unsigned char default_value;
	costmap_2d::Costmap2DPublisher cost_map_publisher_;

	tf::TransformListener tfListener_;
	std::string baseFrameId_,laser_topic_,srv_name;
	nav_msgs::MapMetaData metadata;
	tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub_;
	tf::Stamped<tf::Point> point_bf_;
	std::list<tf::Transform> pointList_;
	ros::Time last_exec_inc,last_exec_dec ;

	std::list<std::vector<unsigned int> > pVectorList_;

	nav_msgs::MapMetaData getMetadata();
	int getQuadrant(tf::Transform p, tf::Transform p_robot);
	void decrementCost(unsigned int cells_x,unsigned int cells_y);
	void setDecrementCost1Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y);
	void setDecrementCost2Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y);
	void setDecrementCost3Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y);
	void setDecrementCost4Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y);
	void cleanCostMap(int quadrant,tf::Transform p, tf::Transform p_robot);
	unsigned int distance2cell(float n);
	bool cellsOK(unsigned int x, unsigned int y);
	void incrementCostProcedure();
	void decrementCostProcedure();
	void updateCostmap();

};

}

#endif /* OBSTACLEDETECTOR_H_ */
