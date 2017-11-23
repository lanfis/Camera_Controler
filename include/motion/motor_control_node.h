#pragma once
#ifndef _MOTOR_CONTROL_NODE_H_
#define _MOTOR_CONTROL_NODE_H_

#include <ros/ros.h>
#include <ros/spinner.h>

#include "Motor_Control.h"

using namespace std;


class Motor_Control_Node
{
    private:
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Motor_Control *ms;
	  
    public:
      Motor_Control_Node(ros::NodeHandle& n, int thread);
      ~Motor_Control_Node();
      void run();
};

Motor_Control_Node::Motor_Control_Node(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
	ms = new Motor_Control(n_);
	ms -> init();
    run();
    spinner.start();
}

Motor_Control_Node::~Motor_Control_Node()
{
	delete ms;
}

void Motor_Control_Node::run()
{
	ms -> run();
}


#endif
