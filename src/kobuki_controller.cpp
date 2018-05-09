#include "path_planner_3d/kobuki_controller.h"

namespace kobuki_controller {

	KobukiController::KobukiController():nh_("~"){
    poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);
    //pathPlan_sub = nh_.subscribe("/make_plan", &PathPlanner::makePlanSrv,this);
		//vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "markers", 10 );

	}

  void KobukiController::poseUpdaterCallback(const Pose::ConstPtr& actualPose){
    //ROS_INFO("Pose Update");
    actualPose_ = *actualPose;

  }

}


using namespace kobuki_controller;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "path_planner_node");
   ros::NodeHandle n;
   KobukiController kobukiController;

   ros::spin();
   return 0;

 }
