#include "path_planner_3d/frontier_path_executer.h"

FrontierPathExecuter::FrontierPathExecuter():
private_nh_("~"),pointReached(false),moving(false),distThreshold(0.1),lastStep(false){
  path_sub	= nh_.subscribe("/frontier_path", 5, &FrontierPathExecuter::pathCallback,this);
  move_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,true);
  poseMsg.header.frame_id = "/map";
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

bool FrontierPathExecuter::findInPath(){
  for(size_t i=0; i<path_.poses.size(); i++){
    if(path_.poses[i].pose.position.x == targetPose.position.x && path_.poses[i].pose.position.y == targetPose.position.y)
      return true;
    else
      return false;
  }
}

double FrontierPathExecuter::coordinatesToAngle(double initX ,double initY, double goalX, double goalY){
  double angle=0.0;
  initX = roundf(initX * 10000)/10000; //Precision milimetros
  initY = roundf(initY * 10000)/10000; //Precision milimetros
  goalX = roundf(goalX * 10000)/10000; //Precision milimetros
  goalY = roundf(goalY * 10000)/10000; //Precision milimetros
  double distX = goalX - initX;
  double distY = goalY - initY;

  if(distX > 0 && distY > 0){
    angle = (double)atan(distY/distX);
    angle = (angle/M_PIl)*180; //Conversion to degrees
  }else
  if(distX < 0 && distY > 0){
    angle = (double)atan(distY/abs(distX));
    angle = (angle/M_PIl)*180; //Conversion to degrees
    angle = 180.0 - angle;
  }else
  if(distX < 0 && distY < 0){
    angle = (double)atan(abs(distY/distX));
    angle = (angle/M_PIl)*180; //Conversion to degrees
    angle = -1.0*(180.0 - angle);
  }else
  if(distX > 0 && distY < 0){
    angle = (double)atan(abs(distY)/distX);
    angle = (angle/M_PIl)*(-180.0); //Conversion to degrees
  }else
  if(distY==0){
    angle=180;
  }else
  if(distX==0){
    angle=90;
  }

  return angle;
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
  tf::Quaternion quat(basePose.orientation.x,basePose.orientation.y,basePose.orientation.z,basePose.orientation.w);

  if(!findInPath()){
    moving=false;
  }


  if(path_.poses.size()>0 && !moving){
    lastStep=true;
    double angle;
    tf::Quaternion q;
    destPose = distCalculation();
    targetPose = path_.poses[destPose].pose;
    angle = coordinatesToAngle(basePose.position.x,basePose.position.y,targetPose.position.x, targetPose.position.y);
    q.setEuler(0,0,angle);
    targetPose.orientation.x = q.x();
    targetPose.orientation.y = q.y();
    targetPose.orientation.z = q.z();
    targetPose.orientation.w = q.w();

    poseMsg.pose = targetPose;
    moving=true;
    /*
    std::cout<<"Robot Pose" <<std::endl;
    std::cout<<"Position" <<std::endl;
    std::cout<<"\tPos X: "<<roundf(basePose.position.x*100)/100 <<std::endl;
    std::cout<<"\tPos Y: "<<roundf(basePose.position.y*100)/100 <<std::endl;
    std::cout<<"\tPos Z: "<<roundf(basePose.position.z*100)/100 <<std::endl;
    std::cout<<"Rotation" <<std::endl;
    std::cout<<"\tOri X: "<<basePose.orientation.x<<std::endl;
    std::cout<<"\tOri Y: "<<basePose.orientation.y<<std::endl;
    std::cout<<"\tOri Z: "<<basePose.orientation.z<<std::endl;
    std::cout<<"\tOri W: "<<basePose.orientation.w<<std::endl;
    std::cout<<"Robot Angle: "<<quat.getAngle()<<std::endl<<std::endl;

    std::cout<<"Nearest Pose in Path" <<std::endl;
    std::cout<<"Position" <<std::endl;
    std::cout<<"\tPos X: "<<targetPose.position.x<<std::endl;
    std::cout<<"\tPos Y: "<<targetPose.position.y<<std::endl;
    std::cout<<"\tPos Z: "<<targetPose.position.z<<std::endl;
    std::cout<<"Rotation" <<std::endl;
    std::cout<<"\tOri X: "<<targetPose.orientation.x<<std::endl;
    std::cout<<"\tOri Y: "<<targetPose.orientation.y<<std::endl;
    std::cout<<"\tOri Z: "<<targetPose.orientation.z<<std::endl;
    std::cout<<"\tOri W: "<<targetPose.orientation.w<<std::endl;
    std::cout<<"Dest Angle: "<<q.getAngle()<<std::endl<<std::endl;*/
    move_pub.publish(poseMsg);

  }

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
