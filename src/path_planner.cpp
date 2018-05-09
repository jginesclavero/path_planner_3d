#include "path_planner_3d/path_planner.h"

namespace path_planner {

	PathPlanner::PathPlanner():nh_("~"),rrtstar(new RRTSTAR){
    octoscan_sub = nh_.subscribe("/octomap_full", 1, &PathPlanner::mapCallback, this);
    plannerSrv = nh_.advertiseService("/make_plan", &PathPlanner::makePlanSrv,this);
		vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "markers", 10 );
		//shortTerm_map_pub = nh_.advertise<octomap_msgs::Octomap>("octomap_mod", 1,false);

	}

  bool PathPlanner::makePlanSrv(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res){
		if (octree == NULL)
      return false;

		startPose_ = req.start.pose;
		goalPose_ = req.goal.pose;

		ROS_INFO("Planning");
		ROS_INFO("Start Pose: ");
		ROS_INFO("\tx: %f",startPose_.position.x);
		ROS_INFO("\ty: %f",startPose_.position.y);
		ROS_INFO("\tz: %f",startPose_.position.z);
		ROS_INFO("Planning");


    makePlan(startPose_, goalPose_);
		publishPlan();

		res.plan = path;
    return true;
  }

  void PathPlanner::mapCallback(const octomap_msgs::Octomap::ConstPtr& octomap){
    ROS_INFO("Octomap received");
    octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap));
		octree->getMetricSize(world_length_,world_width_,world_height_);
		ROS_INFO("length %f, width %f, height %f",world_length_,world_width_,world_height_);
  }

  void PathPlanner::makePlan(Pose start, Pose goal){
    rrtstar->max_iter = 5000;
		rrtstar->step_size = 0.2;
		rrtstar->end_dist_threshold=0.2;
		rrtstar->rrtstar_neighbor_factor = 3.0;
    rrtstar->startPos = start.position;
    rrtstar->endPos = goal.position;
		rrtstar->world_length = world_length_;
		rrtstar->world_width = world_width_;
		rrtstar->world_height = world_height_;
		rrtstar->ground_robot = false;
    rrtstar->initialize();

		int id=0;
    for(int i = 0; i < rrtstar->max_iter; i++) {
        Node *q = rrtstar->getRandomNode();
        if (q) {
            Node *qNearest = rrtstar->nearest(q->position);

            if (rrtstar->distance(q->position, qNearest->position) > rrtstar->step_size) {
                Pose newConfigPosOrient = rrtstar->newConfig(q, qNearest);
                Point newConfigPos;
                newConfigPos = newConfigPosOrient.position;

                if (!isSegmentInObstacle(newConfigPos, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
										qNew->orientation = 0.0; //apaño momentaneo

                    vector<Node *> Qnear;
                    rrtstar->near(qNew->position, rrtstar->step_size*rrtstar->rrtstar_neighbor_factor, Qnear);
                    Node *qMin = qNearest;
                    double cmin = rrtstar->Cost(qNearest) + rrtstar->PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!isSegmentInObstacle(qNear->position, qNew->position) &&
                          (rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew)) < cmin ){
                          qMin = qNear;
													cmin = rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew);
                        }
                    }

                    rrtstar->add(qMin, qNew);

										Pose p1;
										p1.position = qMin->position;
										Pose p2;
										p2.position = qNew->position;

										addLocationVis(id, p2,0.0,0.0,1.0,0.4);
										addConnectionVis(id,p1,p2,1.0,0.0,0.0,0.4);
										id++;
										vis_pub_.publish(nodes_vis_);
										vis_pub_.publish(connection_vis_);


                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!isSegmentInObstacle(qNew->position, qNear->position) &&
                                (rrtstar->Cost(qNew)+rrtstar->PathCost(qNew, qNear)) < rrtstar->Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());

                            // Add edge between qNew and qNear
                            qNear->cost = rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }
                  }
               }
            }
        if (rrtstar->reached()) {
						ROS_INFO("Reached!");
            break;
        }
    }
    Node *q;
    if (rrtstar->reached()) {
        q = rrtstar->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = rrtstar->nearest(rrtstar->endPos);
				ROS_INFO("Not reached, nearest point returned");

    }
    // generate shortest path to destination.
    while (q != NULL) {
        rrtstar->path.push_back(q);
        q = q->parent;
    }
  }

  bool PathPlanner::isSegmentInObstacle(Point p1, Point p2){
    octomap::KeyRay keyRay;
    octomap::KeySet cells;

    octomap::point3d point(p1.x, p1.y, p1.z);
    octomap::point3d point_end(p2.x,p2.y,p2.z);
    if (octree->computeRayKeys(point, point_end, keyRay)){
      cells.insert(keyRay.begin(), keyRay.end());

			int j = 0;
      for (octomap::KeySet::iterator it = cells.begin(), end=cells.end(); it!= end; it++) {
  				if(octree->search(*it) != NULL){
            return true;
          }
  	  }
			return false;

    }
    ROS_ERROR("computeRayKeys is false");
    return true;
  }

	void PathPlanner::publishPlan()
	{
			Pose p1,p2;
			PoseStamped p;
			p1.orientation.w = 1.0;
			p2.orientation.w = 1.0;
			path.poses.resize((int)rrtstar->path.size());
			//nodes_vis_.markers.clear();
			for(int i = 0; i < (int)rrtstar->path.size(); i++) {
					p1.position = rrtstar->path[i]->position;
					addLocationVis(i, p1,0.0,1.0,0.0,1.0);
					if(i<(int)rrtstar->path.size()-1){
						p2.position=rrtstar->path[i+1]->position;
						addConnectionVis(i,p1,p2,0.0,1.0,0.0,1.0);
					}
					p.pose = p1;
					p.header.frame_id = "/map";
					path.poses[i] = p;
			}
			addLocationVis(1, startPose_,1.0,0.0,0.0,1.0);
			addLocationVis(2, goalPose_,1.0,0.0,0.0,1.0);
			vis_pub_.publish(nodes_vis_);
			vis_pub_.publish(connection_vis_);

	}

	void PathPlanner::addLocationVis(int id, geometry_msgs::Pose pose,float r, float g, float b,float alpha)
  {
    	visualization_msgs::Marker marker;
			Pose p;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "locations";
      marker.id = id;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose = pose;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = alpha; // Don't forget to set the alpha!
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      nodes_vis_.markers.push_back(marker);
  }


	void PathPlanner::addConnectionVis(int id, geometry_msgs::Pose from, geometry_msgs::Pose to,float r, float g, float b,float alpha)
	{
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time();
			marker.ns = "connections";
			marker.id = id;
			marker.type = visualization_msgs::Marker::LINE_STRIP;
			marker.action = visualization_msgs::Marker::ADD;
			marker.points.push_back(from.position);
			marker.points.push_back(to.position);
			marker.scale.x = 0.05;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;

			marker.color.r = r;
			marker.color.g = g;
		  marker.color.b = b;

			marker.color.a = alpha; // Don't forget to set the alpha!

			connection_vis_.markers.push_back(marker);
	}

}


using namespace path_planner;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "path_planner_node");
   ros::NodeHandle n;
   PathPlanner path_planner;

   ros::spin();
   return 0;

 }
