#include "path_planner_3d/frontier_path_executer.h"

FrontierPathExecuter::FrontierPathExecuter():
private_nh_("~"),pointReached(false),moving(false){
  path_sub	= nh_.subscribe("/frontier_path", 5, &FrontierPathExecuter::pathCallback,this);
  move_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,true);
}

void
FrontierPathExecuter::pathCallback(const nav_msgs::Path& path){
  path_ = path;
}

size_t
FrontierPathExecuter::distCalculation(){
  float distance;
  float tempDistance=0;
  int destPose=0;
  for(size_t i=0; i<path_.poses.size(); i++){
    tempDistance = sqrt(pow(path_.poses[i].pose.position.x-basePose.position.x,2) + pow(path_.poses[i].pose.position.y-basePose.position.y,2));
    if(distance>tempDistance){
      distance = tempDistance;
      destPose = i;
    }
  }

  return destPose;

}

void
FrontierPathExecuter::step(){
  if(tfListener_.canTransform("/map", "/base_footprint", ros::Time(0))){
    tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), map2bf);
  }

  basePose.position.x = map2bf.getOrigin().x();
  basePose.position.y = map2bf.getOrigin().y();
  basePose.position.z = map2bf.getOrigin().z();
  basePose.orientation.x = map2bf.getRotation().x();
  basePose.orientation.y = map2bf.getRotation().y();
  basePose.orientation.z = map2bf.getRotation().z();
  basePose.orientation.w = map2bf.getRotation().w();

  size_t nextPose;
  if(path_.poses.size()>0){
  nextPose = distCalculation();
  std::cout<<"Nearest Pose in Path" <<std::endl;
  std::cout<<"Position" <<std::endl;
  std::cout<<"\tPos X: "<<path_.poses[nextPose].pose.position.x<<std::endl;
  std::cout<<"\tPos Y: "<<path_.poses[nextPose].pose.position.y<<std::endl;
  std::cout<<"\tPos Z: "<<path_.poses[nextPose].pose.position.z<<std::endl<<std::endl;
  std::cout<<"Rotation" <<std::endl;
  std::cout<<"\tPos X: "<<path_.poses[nextPose].pose.orientation.x<<std::endl;
  std::cout<<"\tPos Y: "<<path_.poses[nextPose].pose.orientation.y<<std::endl;
  std::cout<<"\tPos Z: "<<path_.poses[nextPose].pose.orientation.z<<std::endl;
  std::cout<<"\tPos W: "<<path_.poses[nextPose].pose.orientation.w<<std::endl<<std::endl;
}

  // std::cout<<"Position" <<std::endl;
  // std::cout<<"\tPos X: "<<basePose.position.x<<std::endl;
  // std::cout<<"\tPos Y: "<<basePose.position.y<<std::endl;
  // std::cout<<"\tPos Z: "<<basePose.position.z<<std::endl<<std::endl;
  // std::cout<<"Rotation" <<std::endl;
  // std::cout<<"\tPos X: "<<basePose.orientation.x<<std::endl;
  // std::cout<<"\tPos Y: "<<basePose.orientation.y<<std::endl;
  // std::cout<<"\tPos Z: "<<basePose.orientation.z<<std::endl;
  // std::cout<<"\tPos W: "<<basePose.orientation.w<<std::endl;
  // if(pointReached && !moving){
  //   nextPose = path_.poses[0];
  // }
  // move_pub.publish(nextPose);


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "frontier_path_executer");		//Inicializa el nodo
  FrontierPathExecuter frontier_path_executer;
  ros::Rate loop_rate(5);
  while (ros::ok()){
    frontier_path_executer.step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
