#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Core>
#include <stdlib.h>

#define DEFAULT_SIZE 2.0

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::Rate rate(10.0);

  double known_dis;
  double scale;

  switch (argc){
  case 1:
    known_dis = DEFAULT_SIZE;
    break;
  case 2:
    known_dis = std::atof(argv[1]);
    break;
  default:
    ROS_ERROR("Only one argument is needed for 'known distance'");
  }

  tf::StampedTransform transform;
  tf::TransformListener listener;
  geometry_msgs::TransformStamped msg;
    
  try{
    listener.waitForTransform("/ORB_SLAM/Custom_world", "/ORB_SLAM/End_point",
                             ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("/ORB_SLAM/Custom_world", "/ORB_SLAM/End_point",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  
  tf::transformStampedTFToMsg(transform, msg);

  Eigen::Vector3d ORB_dis(msg.transform.translation.x,
                           msg.transform.translation.y,
                           msg.transform.translation.z);

  scale = known_dis / ORB_dis.norm();

  std::cout << std::endl;
  ROS_INFO("ORB-SLAM Scale is: %f \n", scale);

  return 0;
}


