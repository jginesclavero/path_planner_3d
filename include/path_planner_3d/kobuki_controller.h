#ifndef KOBUKICONTROLLER_H
#define KOBUKICONTROLLER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
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

  private:
    ros::NodeHandle nh_;
    ros::Subscriber poseUpdater_sub;
    ros::ServiceClient pathPlan_srvClt;

    Pose startPose_,goalPose_;
    Pose actualPose_;

    bool poseReady;


		//ros::Publisher vis_pub_;

	};
}
#endif // RRTSTAR_H
