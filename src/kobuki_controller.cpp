#include "path_planner_3d/kobuki_controller.h"

namespace kobuki_controller {

	KobukiController::KobukiController():nh_("~"),pathReady(false),turnReached(false),goalReached(false){

		poseUpdater_sub = nh_.subscribe("/pose", 1, &KobukiController::poseUpdaterCallback, this);
		goal_sub = nh_.subscribe("/clicked_point", 1, &KobukiController::goalPoseCallback, this);
		pathPlan_srvClt = nh_.serviceClient<GetPlan>("/make_plan");
		kobuki_move = nh_.advertise<Twist>("/mobile_base/commands/velocity", 100);
		if (nh_.getParam("ground_robot", ground_robot));
	}

	void KobukiController::poseUpdaterCallback(const PoseStamped::ConstPtr& actualPose){
		kobukiPose_ = actualPose->pose;
	}

	void KobukiController::goalPoseCallback(const PointStamped::ConstPtr& goal){
		getPlanPath(goal->point);
	}


	void KobukiController::getPlanPath(Point goal){

		kobukiPose_.position.z = 0.1;
		reqPlan.start.pose = kobukiPose_;
		goalPose_.position.x = goal.x;
		goalPose_.position.y = goal.y;
		goalPose_.position.z = 0.1;
		if(goal.z > 0.1){
			goalPose_.position.z = goal.z;
		}else{
			goalPose_.position.z = 0.1;

		}
		goalPose_.orientation.w = 1.0;
		reqPlan.goal.pose = goalPose_;
		if(pathPlan_srvClt.call(reqPlan, resPlan)){
			ROS_INFO("Plan received");
			ROS_INFO("Size of path: %lu",resPlan.plan.poses.size());
			ROS_INFO("Start following Path\n");
			remainingSteps = resPlan.plan.poses.size()-1;
			pathReady = true;
		}
	}

	void KobukiController::step(){
		if(!pathReady || !ground_robot){
			return;
		}
		unsigned long int pathLong= resPlan.plan.poses.size();
		Pose kobukiPose = kobukiPose_;
		Pose nextPose = resPlan.plan.poses[remainingSteps-1].pose;

		double minThresholdX = 0.2;
		double minThresholdY = 0.2;

		double angleThZ = 2;
		float dist = sqrt(pow(nextPose.position.x-kobukiPose.position.x,2) + pow(nextPose.position.y-kobukiPose.position.y,2));
		double kobukiAngleX,kobukiAngleY,kobukiAngleZ;
		quaternionToDegreeAngle(kobukiPose,kobukiAngleX,kobukiAngleY,kobukiAngleZ);

		double diffX = nextPose.position.x - kobukiPose.position.x;
		double diffY = nextPose.position.y - kobukiPose.position.y;

		float kp = 0.5;
		float vel_x = dist * kp;
		if(!turnReached){
			double angle = coordinatesToAngle(kobukiPose.position.x, kobukiPose.position.y, nextPose.position.x, nextPose.position.y);
			if(kobukiAngleZ - angle < 0){
				turnLeft(0.3);
			}else{
				turnRight(0.3);
			}
			if(abs(kobukiAngleZ - angle) <= angleThZ){
				turnReached=true;
			}
		}else{
			if(remainingSteps>0){
				if(fabs(diffX)>minThresholdX || fabs(diffY)>minThresholdY){
					moveForward(vel_x);
				}else {
					remainingSteps--;
					ROS_INFO(" ");
					ROS_INFO("Point %lu reached",pathLong-remainingSteps);
					ROS_INFO("Robot Position: ");
					ROS_INFO("\tx: %f",kobukiPose.position.x);
					ROS_INFO("\ty: %f",kobukiPose.position.y);
					if(remainingSteps>0){
						ROS_INFO("Next Position: ");
						ROS_INFO("\tx: %f",resPlan.plan.poses[remainingSteps-1].pose.position.x);
						ROS_INFO("\ty: %f",resPlan.plan.poses[remainingSteps-1].pose.position.y);
						turnReached=false;
					}
				}
			}else{

				goalReached=true;
				turnReached=false;
				/*ROS_INFO("Target reached");
				ROS_INFO("Target: ");
				ROS_INFO("\tx: %f",nextPose.position.x);
				ROS_INFO("\ty: %f",nextPose.position.y);
				ROS_INFO("\tangle: %f",angle);
				ROS_INFO("Robot Position: ");
				ROS_INFO("\tx: %f",kobukiPose.position.x);
				ROS_INFO("\ty: %f",kobukiPose.position.y);
				ROS_INFO("\tangle: %f",kobukiAngleZ);*/

			}
		}
	}

	void KobukiController::moveForward(double speed){
		Twist movTemp;
		movTemp.linear.x = speed;
		kobuki_move.publish(movTemp);
	}
	void KobukiController::moveBack(double speed){
		Twist movTemp;
		movTemp.linear.x = -speed;
		kobuki_move.publish(movTemp);
	}
	void KobukiController::turnLeft(double speed){
		Twist movTemp;
		movTemp.angular.z = speed;
		kobuki_move.publish(movTemp);
	}
	void KobukiController::turnRight(double speed){
		Twist movTemp;
		movTemp.angular.z = -speed;
		kobuki_move.publish(movTemp);
	}
	void KobukiController::stop(){
		Twist movTemp;
		kobuki_move.publish(movTemp);
	}

	Pose KobukiController::degreesToQuaternion(double roll, double pitch, double yaw){

		Pose q;

		//Degrees to radians
		roll= roll*(M_PIl/180);
		pitch= pitch*(M_PIl/180);
		yaw= yaw*(M_PIl/180);

		//Calculate matrix components
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);

		//Calculate quaternion coordinates
		q.orientation.x = cy * sr * cp - sy * cr * sp;
		q.orientation.y = cy * cr * sp + sy * sr * cp;
		q.orientation.z = sy * cr * cp - cy * sr * sp;
		q.orientation.w = cy * cr * cp + sy * sr * sp;

		return q;

	}

	void KobukiController::quaternionToDegreeAngle(Pose q, double& roll, double& pitch, double& yaw)
	{
		// roll (x-axis rotation)
		double sinr = +2.0 * (q.orientation.w * q.orientation.x + q.orientation.y * q.orientation.z);
		double cosr = +1.0 - 2.0 * (q.orientation.x * q.orientation.x + q.orientation.y * q.orientation.y);
		roll = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		double sinp = +2.0 * (q.orientation.w * q.orientation.y - q.orientation.z * q.orientation.x);
		if (fabs(sinp) >= 1)
		pitch = copysign(M_PIl/2, sinp); // use 90 degrees if out of range
		else
		pitch = asin(sinp);

		// yaw (z-axis rotation)
		double siny = +2.0 * (q.orientation.w * q.orientation.z + q.orientation.x * q.orientation.y);
		double cosy = +1.0 - 2.0 * (q.orientation.y * q.orientation.y + q.orientation.z * q.orientation.z);
		yaw = atan2(siny, cosy);

		//Conversion to Degrees
		roll = (roll/M_PIl)*180.0;
		pitch = (pitch/M_PIl)*180.0;
		yaw = (yaw/M_PIl)*180.0;
	}

	double KobukiController::coordinatesToAngle(double initX ,double initY, double goalX, double goalY){
		double distX = goalX - initX;
		double distY = goalY - initY;
		double angle=0.0;

		if(distX > 0 && distY > 0){
			angle = (double)atan(distY/distX);
			angle = (angle/M_PIl)*180; //Conversion to degrees
		}else
		if(distX < 0 && distY > 0){
			angle = (double)atan(distY/abs(distX));
			angle = (angle/M_PIl)*180; //Conversion to degrees
			angle = 180.0 - angle;
			//angle = (90.0 + angle);
		}else
		if(distX < 0 && distY < 0){
			angle = (double)atan(abs(distY/distX));
			angle = (angle/M_PIl)*180; //Conversion to degrees
			angle = -1.0*(180.0 - angle);
			//angle = (270.0 - angle);
		}else
		if(distX > 0 && distY < 0){
			angle = (double)atan(abs(distY)/distX);
			angle = (angle/M_PIl)*(-180.0); //Conversion to degrees
			// angle = (360.0 - angle);
		}
		//angle = (angle/M_PIl)*180; //Conversion to degrees

		return angle;
	}

}

using namespace kobuki_controller;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kobuki_controller_node");
	ros::NodeHandle n;
	KobukiController kobukiController;
	ros::Rate loop_rate(20);
	while (ros::ok()){
		if(!kobukiController.goalReached){
			kobukiController.step();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	// Pose temp;
	// for(double z=0; z<=360.0; z+=15.0 ){
	// 	temp = kobukiController.eulerToQuaternion(0.0,0.0,z);
	// 	ROS_INFO("\tAngulo Entrada:");
	// 	ROS_INFO("\t Yaw: %f",z);
	// 	ROS_INFO("\tOrientacion:");
	// 	ROS_INFO("\t\tx: %f ",temp.orientation.x);
	// 	ROS_INFO("\t\ty: %f ",temp.orientation.y);
	// 	ROS_INFO("\t\tz: %f ",temp.orientation.z);
	// 	ROS_INFO("\t\tw: %f ",temp.orientation.w);
	// }
	// ros::spinOnce();
	return 0;

}
