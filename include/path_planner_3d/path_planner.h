#include <ros/ros.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include "geometry_msgs/Pose.h"
#include "path_planner_3d/rrtstar.h"
#include "nav_msgs/GetPlan.h"

#include <visualization_msgs/MarkerArray.h>


#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

using namespace geometry_msgs;


namespace path_planner {

	class PathPlanner
	{
  public:
      PathPlanner();
      void mapCallback(const octomap_msgs::Octomap::ConstPtr& octomap);
      void getPlan();
      bool isSegmentInObstacle(Point p1,Point p2);
      bool makePlanSrv(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);
      void makePlan(Pose start, Pose goal);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber octoscan_sub;
		ros::Publisher vis_pub_,shortTerm_map_pub;
    octomap::OcTree* octree;
    RRTSTAR *rrtstar;
		visualization_msgs::MarkerArray nodes_vis_,connection_vis_;
		double world_length_,world_width_,world_height_;
		Pose startPose_,goalPose_;
		nav_msgs::Path path;

    ros::ServiceServer plannerSrv;
		void addStartGoalVis(geometry_msgs::Pose start,geometry_msgs::Pose goal);
		void addLocationVis(int id, geometry_msgs::Pose pose,float r, float g, float b,float alpha);
		void addConnectionVis(int id, geometry_msgs::Pose from, geometry_msgs::Pose to,float r, float g, float b,float alpha);
		void publishPlan();


	};
}
#endif
