#ifndef KOBUKICONTROLLER_H
#define KOBUKICONTROLLER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetPlan.h"

using namespace geometry_msgs;

using namespace std;

namespace kobuki_controller {

	class KobukiController
	{
  public:
      KobukiController();
      void poseUpdaterCallback(const Pose::ConstPtr& actualPose);


  private:
    ros::NodeHandle nh_;
    ros::Subscriber poseUpdater_sub;
    //ros::Subscriber pathPlan_sub;

    Pose startPose_,goalPose_;
    Pose actualPose_;

		//ros::Publisher vis_pub_;

	};
}
#endif // RRTSTAR_H
