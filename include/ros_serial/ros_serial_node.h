#pragma once
#ifndef _ROS_SERIAL_NODE_H_
#define _ROS_SERIAL_NODE_H_

#include <ros/ros.h>
#include <ros/spinner.h>

#include "ros_serial.h"

using namespace std;


class ROS_Serial_Node
{
    private:
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  ROS_Serial *ros_srl;
	  
    public:
      ROS_Serial_Node(ros::NodeHandle& n, int thread);
      ~ROS_Serial_Node();
      void run();
};

ROS_Serial_Node::ROS_Serial_Node(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
	ros_srl = new ROS_Serial(n_);
	ros_srl -> init();
    run();
    spinner.start();
}

ROS_Serial_Node::~ROS_Serial_Node()
{
	delete ros_srl;
}

void ROS_Serial_Node::run()
{
	ros_srl -> run();
}


#endif
