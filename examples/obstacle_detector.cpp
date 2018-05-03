/*
 *
 *
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 27/01/2016
 *
 * Nodo para la detección de obstaculos y la construcción del mapa a corto plazo
 *
 */

#include "dn_mapping/obstacle_detector.h"

namespace obstacle_detector {

	ObstacleDetector::ObstacleDetector():
	cost_map(),
	cost_map_publisher_(&nh_,&cost_map,"/map","/costmap_auto",true),
	baseFrameId_("base_footprint"),
	laser_topic_("/scan_filtered"),
	srv_name("MapMetadata"),
  	pointList_(),
  	point_bf_(),
  	last_exec_inc(ros::Time::now()),
  	last_exec_dec(ros::Time::now()),
  	pVectorList_(),
  	private_nh_("~"){

		scanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, laser_topic_, 5);
		pos_sub 	= nh_.subscribe("/amcl_pose", 5, &ObstacleDetector::posCallback,this);

		tfScanSub_ = new tf::MessageFilter<sensor_msgs::LaserScan> (*scanSub_, tfListener_, baseFrameId_, 5);
		tfScanSub_->registerCallback(boost::bind(&ObstacleDetector::laserCallback, this, _1));

		metadata = getMetadata();
		tf::TransformListener tf(ros::Duration(10));
		cells_size_x = metadata.width;
		cells_size_y = metadata.height;
		resolution = metadata.resolution;
		origin_x = metadata.origin.position.x;
		origin_y = metadata.origin.position.y;
		default_value = 255; /*30*/
		scan_ready = false;
		cost_map.resizeMap(cells_size_x,cells_size_y, resolution, origin_x, origin_y);
		cost_map.setDefaultValue(default_value);
		cost_map.resetMap(0,0,cost_map.getSizeInCellsX(), cost_map.getSizeInCellsY());

		private_nh_.getParam("cost_inc",cost_inc);
		private_nh_.getParam("cost_dec",cost_dec);
		private_nh_.getParam("min_lenght",min_lenght);
		private_nh_.getParam("max_lenght",max_lenght);
	}

	void
	ObstacleDetector::laserCallback(const  sensor_msgs::LaserScan::ConstPtr& scan_in){

		float o_t_min, o_t_max, o_t_inc,w;
		double wObstacle=0.0;
		float x, y, z;
		float ax, ay, az;
		tf::Stamped<tf::Point> scan_sensor,point_in_cells_;
		tf::Transform bf2obj,map2obj;
		tf::Quaternion q;
		tf::StampedTransform map2bf;
		unsigned int cells_x,cells_y;
		std::vector<unsigned int> objectPoint(2);

		o_t_min = scan_in->angle_min;
		o_t_max = scan_in->angle_max;
		o_t_inc = scan_in->angle_increment;

		int num_points = (int)2.0*o_t_max/o_t_inc;
		if(tfListener_.canTransform("/map", "/base_footprint", ros::Time(0))){
			tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), map2bf);
			for(int i=0; i<num_points; i++){
				float theta = o_t_min+i*o_t_inc;
				float r = scan_in->ranges[i];
				if (r >= max_lenght) {
					r = max_lenght;
				}
				if(r > min_lenght && !isnan(r)){
					scan_sensor.setX(r*cos(theta));
					scan_sensor.setY(r*sin(theta));
					scan_sensor.setZ(0.0);
					scan_sensor.setW(1.0);
					scan_sensor.stamp_ = scan_in->header.stamp;
					scan_sensor.frame_id_ = scan_in->header.frame_id;
					tfListener_.transformPoint(baseFrameId_, scan_sensor, point_bf_);
					x = point_bf_.getX();
					y = point_bf_.getY();
					z = 0.0;
					ax = 0.0;
					ay = 0.0;
					az = 0.0;
					bf2obj.setOrigin(tf::Vector3(x,y,z));
					q.setRPY(ax, ay, az);
					bf2obj.setRotation(q);
					pointList_.push_back(bf2obj);
					if(r != max_lenght){
	 					map2obj = map2bf * bf2obj;
	 					cells_x = distance2cell(map2obj.getOrigin().x());
	 					cells_y = distance2cell(map2obj.getOrigin().y());
	 					objectPoint[0] = cells_x;
	 					objectPoint[1] = cells_y;
	 					pVectorList_.push_back(objectPoint);
					}
				}
			}
			pVectorList_.unique();
			pointList_.unique();
			scan_ready = true;
		}
	}

	void
	ObstacleDetector::posCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos){
		posRobot_x = pos->pose.pose.position.x;
		posRobot_y = pos->pose.pose.position.y; //Pos del robot respecto al origen
		double yaw = tf::getYaw(pos->pose.pose.orientation) - M_PI;
		float degrees = yaw*180/M_PI;
		posRobot_w = 360 + degrees;
		pos_ready = true;

	}



	void
	ObstacleDetector::step(){
		updateCostmap();
		cost_map_publisher_.publishCostmap();
	}

	nav_msgs::MapMetaData
	ObstacleDetector::getMetadata(){
		ros::ServiceClient client = nh_.serviceClient<dn_mapping::Metadata>(srv_name);
		dn_mapping::Metadata srv;
		while(true){
			if (client.call(srv)){
    			ROS_INFO("GetMetadata Success");
    			break;
  			}else{
    			//ROS_ERROR("Failed to call service Metadata");
			}
		}
		return srv.response.metadata_res;
	}

	int
	ObstacleDetector::getQuadrant(tf::Transform p, tf::Transform p_robot){
		float p_x,p_y,p_robot_x,p_robot_y;
		p_x = p.getOrigin().x();
		p_y = p.getOrigin().y();

		p_robot_x = p_robot.getOrigin().x();
		p_robot_y = p_robot.getOrigin().y();

		if(p_x == p_robot_x && p_y == p_robot_y){
			return 0;
		}else if(p_x >= p_robot_x && p_y >= p_robot_y){
			return 1;
		}else if(p_x <= p_robot_x && p_y >= p_robot_y){
			return 2;
		}else if(p_x <= p_robot_x && p_y <= p_robot_y){
			return 3;
		}else{
			return 4;
		}
	}

	void
	ObstacleDetector::decrementCost(unsigned int cells_x,unsigned int cells_y){
		unsigned char cost_now;
		if(cellsOK(cells_x,cells_y)){
			cost_now = cost_map.getCost(cells_x,cells_y);
			if(cost_now == 255){
				cost_map.setCost(cells_x,cells_y,0);
			}else if(cost_now > cost_dec - 1){
				cost_map.setCost(cells_x,cells_y, cost_now - cost_dec);
			}else{
				cost_map.setCost(cells_x,cells_y,0);
			}
		}
	}

	void
	ObstacleDetector::setDecrementCost1Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y){
		while(point_A_x > point_B_x || point_A_y > point_B_y){
			if(point_A_x > point_B_x){
				point_A_x = point_A_x - mod_x;
			}
			if(point_A_y > point_B_y){
				point_A_y = point_A_y - mod_y;
			}
			decrementCost(point_A_x,point_A_y);
		}
	}

	void
	ObstacleDetector::setDecrementCost2Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y){
		while(point_A_x < point_B_x || point_A_y > point_B_y){
			if(point_A_x < point_B_x){
				point_A_x = point_A_x + mod_x;
			}
			if(point_A_y > point_B_y){
				point_A_y = point_A_y - mod_y;
			}
			decrementCost(point_A_x,point_A_y);
		}
	}

	void
	ObstacleDetector::setDecrementCost3Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y){
		while(point_A_x < point_B_x || point_A_y < point_B_y){
			if(point_A_x < point_B_x){
				point_A_x = point_A_x + mod_x;
			}
			if(point_A_y < point_B_y){
				point_A_y = point_A_y + mod_y;
			}
			decrementCost(point_A_x,point_A_y);
		}
	}

	void
	ObstacleDetector::setDecrementCost4Q(unsigned int point_A_x,unsigned int point_A_y,unsigned int point_B_x,unsigned int point_B_y,int mod_x,int mod_y){
		while(point_A_x > point_B_x || point_A_y < point_B_y){
			if(point_A_x > point_B_x){
				point_A_x = point_A_x - mod_x;
			}
			if(point_A_y < point_B_y){
				point_A_y = point_A_y + mod_y;
			}
			decrementCost(point_A_x,point_A_y);
		}
	}

	void
	ObstacleDetector::cleanCostMap(int quadrant,tf::Transform p, tf::Transform p_robot){
		unsigned int p_cell_x,p_cell_y,p_robot_cell_x,p_robot_cell_y;
		p_cell_x = distance2cell(p.getOrigin().x());
		p_cell_y = distance2cell(p.getOrigin().y());
		p_robot_cell_x = distance2cell(p_robot.getOrigin().x());
		p_robot_cell_y = distance2cell(p_robot.getOrigin().y());
		if(!cellsOK(p_cell_x,p_cell_y)){
			return;
		}
		switch(quadrant){
			case 1:
				setDecrementCost1Q(p_cell_x,p_cell_y,p_robot_cell_x,p_robot_cell_y,1,1);
				break;
			case 2:
				setDecrementCost2Q(p_cell_x,p_cell_y,p_robot_cell_x,p_robot_cell_y,1,1);
				break;
			case 3:
				setDecrementCost3Q(p_cell_x,p_cell_y,p_robot_cell_x,p_robot_cell_y,1,1);
				break;
			case 4:
				setDecrementCost4Q(p_cell_x,p_cell_y,p_robot_cell_x,p_robot_cell_y,1,1);
				break;
			default:
				break;
		}
	}

	unsigned int
	ObstacleDetector::distance2cell(float n){
		unsigned int cells;
		return round(n/resolution);
	}

	bool
	ObstacleDetector::cellsOK(unsigned int x, unsigned int y){
		return (x<=cells_size_x && y<=cells_size_y && x>=0 && y>=0);
	}


	void
	ObstacleDetector::incrementCostProcedure(){
		tf::StampedTransform map2bf;
		tf::Transform map2obj,point;
		unsigned int cells_x,cells_y;
		int cost_now;
		//if(ros::Time::now() > last_exec_inc + ros::Duration(1)){
	 		for (std::list<std::vector<unsigned int> >::iterator it=pVectorList_.begin(); it != pVectorList_.end(); ++it){
	 			std::vector<unsigned int> p;
	 			p = *it;
	 			cells_x = p[0];
	 			cells_y = p[1];
	 			if(cellsOK(cells_x,cells_y)){
	 				cost_now = cost_map.getCost(cells_x,cells_y);
	 				if(cost_now == 255){
	 					cost_map.setCost(cells_x,cells_y,0);
	 				}else if((cost_now < 254 - cost_inc)) {
	 					cost_map.setCost(cells_x,cells_y,cost_now+cost_inc);
	 				}else{
	 					cost_map.setCost(cells_x,cells_y,254);
	 				}
	 			}
			}

			//last_exec_inc = ros::Time::now();
		//}

	}

	void
	ObstacleDetector::decrementCostProcedure(){
		tf::StampedTransform map2bf;
		tf::Transform point,p_robot,map2obj;
		int quadrant;
		unsigned int cell_x,cell_y;
		p_robot.setOrigin(tf::Vector3(posRobot_x,posRobot_y,0));
		tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), map2bf);
		//if(ros::Time::now() > last_exec_dec + ros::Duration(1)){
			for (std::list<tf::Transform>::iterator it=pointList_.begin(); it != pointList_.end(); ++it){
				point = *it;
				map2obj = map2bf * point;
				quadrant = getQuadrant(map2obj,p_robot);
				cleanCostMap(quadrant,map2obj,p_robot);
			}
			//last_exec_dec = ros::Time::now();
		//}
	}

	void
	ObstacleDetector::updateCostmap(){
		if(scan_ready && pos_ready){
			incrementCostProcedure();
			decrementCostProcedure();
		}
		pVectorList_.clear();
		pointList_.clear();
	}
}

using namespace obstacle_detector;

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_detector");		//Inicializa el nodo
	ObstacleDetector obs;
	ros::Rate loop_rate(5);

	while (ros::ok()){
  		obs.step();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
