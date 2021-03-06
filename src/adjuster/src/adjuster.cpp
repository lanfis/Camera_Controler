#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/loader.h>
#include <string>
#include "adjuster/adjuster_act.h"

using namespace std;


int main(int argc, char** argv)
{
  string nodeName = "adjuster";
  ROS_INFO("Initializing %s ...", nodeName.c_str());
  ros::init(argc, argv, nodeName.c_str());
  ros::NodeHandle n;

  ROS_INFO("%s activating ok !", nodeName.c_str());
  Adjuster_ACT an(n, 0);
  
  while (ros::ok()) 
  {
    an.run();
  }
  //ros::spin();
  
  ros::shutdown();
  return 0;
}

