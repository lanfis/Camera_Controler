#pragma once
#ifndef _ADJUSTER_ACT_H_
#define _ADJUSTER_ACT_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "adjuster_node.h"

using namespace std;
using namespace cv;


class Adjuster_ACT
{
    private:
      string ver_ = "1.0";
      ros::AsyncSpinner spinner;
      ros::NodeHandle n_;
	  Adjuster_Node *ac;
	  
    public:
      Adjuster_ACT(ros::NodeHandle& n, int thread);
      ~Adjuster_ACT();
      void run();
};

Adjuster_ACT::Adjuster_ACT(ros::NodeHandle& n, int thread) : n_(n), spinner(thread)
{
	ac = new Adjuster_Node(n_);
	ac -> init();
    run();
    spinner.start();
}

Adjuster_ACT::~Adjuster_ACT()
{
	delete ac;
}

void Adjuster_ACT::run()
{
	//ac -> run();
}


#endif
