#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
 
  tf::TransformListener listener;
  geometry_msgs::TransformStamped msg;
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/ORB_SLAM/new_world", "/ORB_SLAM/Camera",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
     
    tf::transformStampedTFToMsg(transform, msg);
    std::cout << "transform" << msg << std::endl;
  }
  return 0;
}


