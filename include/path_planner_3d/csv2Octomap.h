#include <ros/ros.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>



#ifndef CSV2OCTOMAP_H_
#define CSV2OCTOMAP_H_

namespace csv_2_octomap {

	class Csv2Octomap
	{
  public:
      Csv2Octomap();

  private:

		void csv2CostMap(std::string);
		void transform2DtoOct(costmap_2d::Costmap2D cost_map);
		void publishFullOctoMap();
    ros::NodeHandle nh_;
    ros::Subscriber octoscan_sub;
		ros::Publisher octomap_pub;
    octomap::OcTree* octree;
		costmap_2d::Costmap2D cost_map;

		unsigned int cells_size_x, cells_size_y;
		double  origin_x ,origin_y;
		unsigned char default_value;

		double voxel_res,map_res;
		unsigned m_treeDepth;
		unsigned m_maxTreeDepth;

		 std::vector<std::vector<int>> intMap; //For CSV to CostMap conversion
		 int sizeMapX, sizeMapY;

	};
}
#endif
