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

		startPose_ = offsetOriginPoint(req.start.pose);
		goalPose_ = offsetOriginPoint(req.goal.pose);

		ROS_INFO("Planning");

    makePlan(startPose_, goalPose_);
		publish_plan();
    return true;

		/*
		octomap::point3d point(-1.5, -1.5, 0.0);
		octomap::point3d point_end(-1.5, -1.5, 1.2);

		octomap::KeyRay keyRay;
		octomap::KeySet cells;

		if (octree->computeRayKeys(point, point_end, keyRay)){
			cells.insert(keyRay.begin(), keyRay.end());

			int j = 0;
			for (octomap::KeySet::iterator it = cells.begin(), end=cells.end(); it!= end; it++) {
					octomap::OcTreeNode* node = octree->search(*it);
					if(node){
						octree->setNodeValue(*it,0,false);
					}else{
						octree->updateNode(*it, false);
						octree->setNodeValue(*it, 0, false);
					}
			}
		}

		octomap_msgs::Octomap map;
		if (octomap_msgs::fullMapToMsg(*octree, map)){
			//ROS_INFO("send shortTerm_map");
			map.header.frame_id = "map";
			map.header.stamp = ros::Time::now();
			shortTerm_map_pub.publish(map);
			ROS_DEBUG("ShortTermMap published");
		}else{
			ROS_ERROR("Error serializing OctoMap");
		}

		return true;*/


  }

  void PathPlanner::mapCallback(const octomap_msgs::Octomap::ConstPtr& octomap){
    ROS_INFO("Octomap received");
    octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap));
		octree->getMetricSize(world_length_,world_width_,world_height_);
		ROS_INFO("length %f, width %f, height %f",world_length_,world_width_,world_height_);
  }

	Pose
	PathPlanner::offsetOriginPoint(Pose p){
		Pose r;
		r.position.x = p.position.x - 1.5;
		r.position.y = p.position.y - 1.5;

		return r;
	}

  void PathPlanner::makePlan(Pose start, Pose goal){
    rrtstar->max_iter = 10000;
		rrtstar->step_size = 1;
    rrtstar->startPos = start.position;
    rrtstar->endPos = goal.position;
		rrtstar->world_length = world_length_;
		rrtstar->world_width = world_width_;
		rrtstar->world_height = world_height_;
    rrtstar->initialize();

    //set length , width, height
    //set maxIterations and stepSize from param
    //rrtstar->setMaxIterations(ui->maxIterations->text().toInt());
    //rrtstar->setStepSize(ui->stepSize->text().toInt());

    // RRTSTAR Algorithm

    for(int i = 0; i < rrtstar->max_iter; i++) {
        Node *q = rrtstar->getRandomNode();
        if (q) {
            Node *qNearest = rrtstar->nearest(q->position);
						//ROS_INFO("q [%f,%f]",q->position.x,q->position.y);
						//ROS_INFO("qNearest [%f,%f]",qNearest->position.x,qNearest->position.y);

            if (rrtstar->distance(q->position, qNearest->position) < rrtstar->step_size) {
                Pose newConfigPosOrient = rrtstar->newConfig(q, qNearest);
                Point newConfigPos;
                newConfigPos = newConfigPosOrient.position;
								ROS_INFO("newConfigPos [%f,%f]",newConfigPos.x,newConfigPos.y);
								ROS_INFO("qNearest [%f,%f]",qNearest->position.x,qNearest->position.y);
                if (!isSegmentInObstacle(newConfigPos, qNearest->position)) {
										ROS_INFO("no isSegmentInObstacle");
                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
                		//  qNew->orientation = newConfigPosOrient.z(); MOVIDA!!!!!!!!!!
										qNew->orientation = 0.0; //apaño momentaneo
                    //qNew->path = path;

                    vector<Node *> Qnear;
                    rrtstar->near(qNew->position, rrtstar->step_size*RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    //std::cerr << "Found Nearby " << Qnear.size() << "\n";
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
										//ROS_INFO("qMin [%f,%f]",qMin->position.x,qMin->position.y);
										//ROS_INFO("qNew [%f,%f]",qNew->position.x,qNew->position.y);
                    rrtstar->add(qMin, qNew);

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

	void PathPlanner::publish_plan()
	{
			Pose p;
			p.orientation.w = 1.0;
			nodes_vis_.markers.clear();
			int j = 0;
			for(int i = 0; i < (int)rrtstar->path.size(); i++) {
					p.position = rrtstar->path[i]->position;
					addLocationVis(i, p);
					j = i;
			}
			addLocationVis(j+1, startPose_);
			addLocationVis(j+2, goalPose_);
			vis_pub_.publish(nodes_vis_);
	}

	void PathPlanner::addLocationVis(int id, geometry_msgs::Pose pose)
  {
    	visualization_msgs::Marker marker;
			Pose p;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "locations";
      marker.id = id;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
			p.position.x = pose.position.x - startPose_.position.x;
			p.position.y = pose.position.y - startPose_.position.y;

      marker.pose = p;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      nodes_vis_.markers.push_back(marker);
  }

	void PathPlanner::addStartGoalVis(geometry_msgs::Pose start,geometry_msgs::Pose goal)
  {
    	visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "locations";
      marker.id = nodes_vis_.markers.size()+1;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = start;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      nodes_vis_.markers.push_back(marker);
			//marker.id = marker.id + 1;
			//marker.pose = goal;
			//marker.header.stamp = ros::Time();
			//nodes_vis_.markers.push_back(marker);

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
