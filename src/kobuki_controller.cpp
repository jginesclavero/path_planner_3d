#include "path_planner_3d/kobuki_controller.h"

namespace kobuki_controller {

	KobukiController::KobukiController():nh_("~"),poseReady(false){

    poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);
    pathPlan_srvClt = nh_.serviceClient<GetPlan>("/make_plan");
		kobuki_move = nh_.advertise<Twist>("/mobile_base/commands/velocity", 100);
		//moveKobuki();
  }

  void KobukiController::poseUpdaterCallback(const PoseStamped::ConstPtr& actualPose){

		actualPose_ = actualPose->pose;

		if(!poseReady && actualPose_.position.x > 0.2){
			poseReady=true;
			getPlanPath();
		}


  }


  void KobukiController::getPlanPath(){

      // GetPlan::Request reqPlan;
      // GetPlan::Response resPlan;
      actualPose_.position.z = 0.1;

      reqPlan.start.pose = actualPose_;

      goalPose_.position.x = 7.0;
      goalPose_.position.y = 2.0;
      goalPose_.position.z = 0.1;
      goalPose_.orientation.w = 1.0;
      reqPlan.goal.pose = goalPose_;
      if(pathPlan_srvClt.call(reqPlan, resPlan)){
        ROS_INFO("Plan received");
        ROS_INFO("Size of path: %lu",resPlan.plan.poses.size());
        // ROS_INFO("\tPos x: %.2f",resPlan.plan.poses[resPlan.plan.poses.size()-1].pose.position.x);
        // ROS_INFO("\tPos y: %.2f",resPlan.plan.poses[resPlan.plan.poses.size()-1].pose.position.y);
        // ROS_INFO("\tPos x: %.2f",resPlan.plan.poses[0].pose.position.x);
        // ROS_INFO("\tPos y: %.2f",resPlan.plan.poses[0].pose.position.y);
				ROS_INFO("Start following Path\n");
				// moveKobuki(resPlan);
				//moveKobuki();

      }
    }


		void KobukiController::moveForward(double speed){
			Twist movTemp;
			movTemp.linear.x = speed;
			movTemp.linear.y = 0.0;
			movTemp.linear.z = 0.0;
			movTemp.angular.x = 0.0;
			movTemp.angular.y = 0.0;
			movTemp.angular.z = 0.0;
			kobuki_move.publish(movTemp);

		}
		void KobukiController::moveBack(double speed){
			Twist movTemp;
			movTemp.linear.x = -speed;
			// movTemp.linear.y = 0.0;
			// movTemp.linear.z = 0.0;
			// movTemp.angular.y = 0.0;
			// movTemp.angular.x = 0.0;
			// movTemp.angular.z = 0.0;
			kobuki_move.publish(movTemp);
		}
		void KobukiController::turnLeft(double speed){
			Twist movTemp;
			// movTemp.linear.x = 0.0;
			// movTemp.linear.y = 0.0;
			// movTemp.linear.z = 0.0;
			// movTemp.angular.y = 0.0;
			// movTemp.angular.x = 0.0;
			movTemp.angular.z = -speed;
			kobuki_move.publish(movTemp);

		}
		void KobukiController::turnRight(double speed){
			Twist movTemp;
			// movTemp.linear.x = 0.0;
			// movTemp.linear.y = 0.0;
			// movTemp.linear.z = 0.0;
			// movTemp.angular.x = 0.0;
			// movTemp.angular.y = 0.0;
			movTemp.angular.z = speed;
			kobuki_move.publish(movTemp);
		}

		//void KobukiController::moveKobuki(GetPlan::Response plan){
		/*void KobukiController::moveKobuki(){
			ROS_INFO("Moving Kobuki...");

			unsigned long int pathLong= resPlan.plan.poses.size();
			Pose actualPose = actualPose_;
			Pose goalPose = resPlan.plan.poses[0].pose;
			Pose nextPose = resPlan.plan.poses[pathLong-3].pose;
			Pose tempPose;

			// bool poseAchieved=false;
			// for(int i=pathLong-2; i > pathLong; i--){
			// 		nextPose = plan.plan.poses[i];
			//
			// 		while(!poseAchieved){
			//
			//
			// 			poseAchieved=true;
			// 		}
			// }

			double thresholdX=0.2;
			double thresholdY=0.2;
			double thresholdZ=0.2;
			double thresholdW=0.2;
			double diffX = abs(nextPose.position.x - actualPose_.position.x);
			double diffY = abs(nextPose.position.y - actualPose_.position.y);
			double diffZ = abs(nextPose.orientation.z - actualPose_.orientation.z);
			double diffW = abs(nextPose.orientation.w - actualPose_.orientation.w);

			ROS_INFO("Punto Inicial:");
			ROS_INFO("\tPosicion:");
			ROS_INFO("\t\tx: %f ",actualPose_.position.x);
			ROS_INFO("\t\ty: %f ",actualPose_.position.y);
			ROS_INFO("\t\tz: %f ",actualPose_.position.z);
			ROS_INFO("\tOrientacion:");
			ROS_INFO("\t\tx: %f ",actualPose_.orientation.x);
			ROS_INFO("\t\ty: %f ",actualPose_.orientation.y);
			ROS_INFO("\t\tz: %f ",actualPose_.orientation.z);
			ROS_INFO("\t\tw: %f \n",actualPose_.orientation.w);

			ROS_INFO("Punto Final:");
			ROS_INFO("\tPosicion:");
			ROS_INFO("\t\tx: %f ",nextPose.position.x);
			ROS_INFO("\t\ty: %f ",nextPose.position.y);
			ROS_INFO("\t\tz: %f ",nextPose.position.z);
			ROS_INFO("\tOrientacion:");
			ROS_INFO("\t\tx: %f ",nextPose.orientation.x);
			ROS_INFO("\t\ty: %f ",nextPose.orientation.y);
			ROS_INFO("\t\tz: %f ",nextPose.orientation.z);
			ROS_INFO("\t\tw: %f ",nextPose.orientation.w);

			while( diffX > thresholdX){
				diffX = abs(nextPose.position.x - actualPose_.position.x);
				//diffY = abs(nextPose.position.y - actualPose.position.y);
				ROS_INFO("DiffX: %f",diffX);
				moveForward(0.1);
				sleep(1);
			}

			ROS_INFO("Target reached");


			// while( diffZ > thresholdZ && diffW > thresholdW){
			// 	diffZ = abs(nextPose.orientation.z - actualPose.orientation.z);
			// 	diffW = abs(nextPose.orientation.w - actualPose.orientation.w);
			// 	ROS_INFO("Turn Right");
			// 	turnRight(1.0);
			// 	usleep(50);
			// }
			//
			// ROS_INFO("Move Back");
			// moveBack(1.0); //No se porque no funciona el primer mensaje
			// sleep(2);
			// moveBack(1.0);
			// sleep(2);
			//
			// ROS_INFO("Move Forward");
			// moveForward(1.0);
			// sleep(2);
			//
			// ROS_INFO("Turn Left");
			// turnLeft(2.0);
			// sleep(2);
			//
			// ROS_INFO("Turn Right");
			// turnRight(2.0);
			// sleep(2);
		}*/

		void KobukiController::step(){
			unsigned long int pathLong= resPlan.plan.poses.size();
			if(pathLong <= 0){
				return;
			}
			ROS_INFO("Moving Kobuki...");
			Pose actualPose = actualPose_;
			Pose goalPose = resPlan.plan.poses[0].pose;
			Pose nextPose = resPlan.plan.poses[pathLong-3].pose;
			Pose tempPose;

			// bool poseAchieved=false;
			// for(int i=pathLong-2; i > pathLong; i--){
			// 		nextPose = plan.plan.poses[i];
			//
			// 		while(!poseAchieved){
			//
			//
			// 			poseAchieved=true;
			// 		}
			// }

			double thresholdX=0.2;
			double thresholdY=0.2;
			double thresholdZ=0.2;
			double thresholdW=0.2;
			double diffX = abs(nextPose.position.x - actualPose_.position.x);
			double diffY = abs(nextPose.position.y - actualPose_.position.y);
			double diffZ = abs(nextPose.orientation.z - actualPose_.orientation.z);
			double diffW = abs(nextPose.orientation.w - actualPose_.orientation.w);

			ROS_INFO("Punto Inicial:");
			ROS_INFO("\tPosicion:");
			ROS_INFO("\t\tx: %f ",actualPose_.position.x);
			ROS_INFO("\t\ty: %f ",actualPose_.position.y);
			ROS_INFO("\t\tz: %f ",actualPose_.position.z);
			ROS_INFO("\tOrientacion:");
			ROS_INFO("\t\tx: %f ",actualPose_.orientation.x);
			ROS_INFO("\t\ty: %f ",actualPose_.orientation.y);
			ROS_INFO("\t\tz: %f ",actualPose_.orientation.z);
			ROS_INFO("\t\tw: %f \n",actualPose_.orientation.w);

			ROS_INFO("Punto Final:");
			ROS_INFO("\tPosicion:");
			ROS_INFO("\t\tx: %f ",nextPose.position.x);
			ROS_INFO("\t\ty: %f ",nextPose.position.y);
			ROS_INFO("\t\tz: %f ",nextPose.position.z);
			ROS_INFO("\tOrientacion:");
			ROS_INFO("\t\tx: %f ",nextPose.orientation.x);
			ROS_INFO("\t\ty: %f ",nextPose.orientation.y);
			ROS_INFO("\t\tz: %f ",nextPose.orientation.z);
			ROS_INFO("\t\tw: %f ",nextPose.orientation.w);

			if( diffX > thresholdX){
				diffX = abs(nextPose.position.x - actualPose_.position.x);
				//diffY = abs(nextPose.position.y - actualPose.position.y);
				ROS_INFO("DiffX: %f",diffX);
				moveForward(0.1);
			}

			ROS_INFO("Target reached");

		}

}


using namespace kobuki_controller;
int main(int argc, char **argv)
{
   ros::init(argc, argv, "kobuki_controller_node");
   ros::NodeHandle n;
   KobukiController kobukiController;
	 ros::Rate loop_rate(10);
	 while (ros::ok()){
		 kobukiController.step();
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
   return 0;

 }
