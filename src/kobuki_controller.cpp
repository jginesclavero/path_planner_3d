#include "path_planner_3d/kobuki_controller.h"

namespace kobuki_controller {

	KobukiController::KobukiController():nh_("~"),poseReady(false){
    ROS_INFO("subscribe");

    poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);
    ROS_INFO("serviceClient before");
    pathPlan_srvClt = nh_.serviceClient<GetPlan>("/make_plan");
    ROS_INFO("serviceClient after");
		//vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "markers", 10 );

  }

  void KobukiController::poseUpdaterCallback(const PoseStamped::ConstPtr& actualPose){
    // ROS_INFO("Pose Update");
    actualPose_ = actualPose->pose;

    if(!poseReady){
      getPlanPath();
    }
      poseReady=true;

    // ROS_INFO("Pose received: ");
    // ROS_INFO("\t x: %.2f",actualPose_.position.x);
    // ROS_INFO("\t y: %.2f",actualPose_.position.y);
    // ROS_INFO("\t z: %.2f",actualPose_.position.z);
  }

  void KobukiController::getPlanPath(){


      ROS_INFO("Inside if");
      GetPlan::Request reqPlan;
      GetPlan::Response resPlan;
      actualPose_.position.z = 0.1;

      reqPlan.start.pose = actualPose_;
      goalPose_.position.x = 7.0;
      goalPose_.position.y = 2.0;
      goalPose_.position.z = 0.1;
      goalPose_.orientation.w = 1.0;
      reqPlan.goal.pose = goalPose_;
      if(pathPlan_srvClt.call(reqPlan, resPlan)){
        ROS_INFO("Plan received");

        // ROS_INFO("\t x: %.2f",resPlan.position.x);
        // ROS_INFO("\t y: %.2f",actualPose_.position.y);
        // ROS_INFO("\t z: %.2f",actualPose_.position.z);
      }
    }

}


using namespace kobuki_controller;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "kobuki_controller_node");
   ros::NodeHandle n;
   KobukiController kobukiController;

   ros::spin();
   return 0;

 }
