#ifndef KOBUKICONTROLLER_H
#define KOBUKICONTROLLER_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/GetPlan.h"

using namespace geometry_msgs;
using namespace nav_msgs;

using namespace std;

namespace kobuki_controller {

	class KobukiController
	{
  public:
      KobukiController();
      void poseUpdaterCallback(const PoseStamped::ConstPtr& actualPose);
      void getPlanPath();
			void step();
			void moveForward(double speed);
			void moveBack(double speed);
			void turnLeft(double speed);
			void turnRight(double speed);
			Pose degreesToQuaternion(double roll, double pitch, double yaw);
			void quaternionToDegreeAngle(Pose q, double& roll, double& pitch, double& yaw);
			double coordinatesToAngle(double initX ,double initY, double goalX, double goalY);
			bool goalReached;


  private:
    ros::NodeHandle nh_;
    ros::Subscriber poseUpdater_sub;
    ros::ServiceClient pathPlan_srvClt;
		ros::Publisher kobuki_move;

    Pose startPose_,goalPose_;
    Pose kobukiPose_;
    bool poseReady;

		GetPlan::Request reqPlan;
		GetPlan::Response resPlan;

		bool turnReached;
		int remainingSteps;


	};
}
#endif // RRTSTAR_H
