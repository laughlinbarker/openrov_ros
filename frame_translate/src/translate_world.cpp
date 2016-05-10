#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <iostream>
#include <string>

std::string frame_name;

void tfCallback(const tf2_msgs::TFMessage& msg){
  static bool isNewWorld = true;
  static double begin = ros::Time::now().toSec();
  double duration = ros::Time::now().toSec() - begin;
  
  //std::string save_mode;
  //std::getline (std::cin, save_mode);

  ros::Time time = msg.transforms[0].header.stamp;
  tf::StampedTransform camera;
  static tf::StampedTransform new_world;

  //new_world.setIdentity();
  tf::transformStampedMsgToTF( msg.transforms[0], camera);

  //std::cout << "transform: " << msg.transforms[0] << std::endl;
  //std::cout << "camera Rot: " << camera.getRotation().x() << std::endl ;

  //std::cout << "duration " << duration << std::endl;
  
  if (isNewWorld && duration > 2){
    new_world = camera;
    
    //std::cout << "entered if! " << std::endl;
    //std::cout << camera.getRotation().x() << std::endl;
    //std::cout << camera.getRotation().y() << std::endl;
    //std::cout << camera.getRotation().z() << std::endl;
    //std::cout << camera.getRotation().x() << std::endl;
    
    isNewWorld = false;
  }

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(new_world, ros::Time::now(), "ORB_SLAM/World", frame_name));
  /*
  tf::TransformListener listener; 
  tf::StampedTransform nWorldToCam;
  try{
     listener.lookupTransform("/ORB_SLAM/new_world", "/ORB_SLAM/Camera",
			     ros::Time::now(), nWorldToCam);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    //ros::Duration(1.0).sleep();
  }
  */ 
  ros::Duration(1).sleep();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "world_transform");
  
  ros::NodeHandle node;

  if (argc == 1){
    frame_name = "ORB_SLAM/Custom_world";
  }else{
    frame_name = argv[1];
  }

  tf::TransformListener listener;
  //tf::TransformBroadcaster br;
  
  ros::Subscriber sub = node.subscribe("/tf", 100, &tfCallback);
/*
  ros::Rate rate(10.0);

  while (node.ok()){
  tf::StampedTransform transform;

 
  try{
    listener.lookupTransform("/ORB_SLAM/Camera", "/ORB_SLAM/Camera",
			      ros::Time(0), transform);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ss", "ff"));  rate.sleep();
  }
*/

  while (ros::ok()){
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }
 // ros::spin();
  return 0;
};  
