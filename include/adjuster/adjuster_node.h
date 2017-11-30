#pragma once
#ifndef _ADJUSTER_NODE_H_
#define _ADJUSTER_NODE_H_

#include <ctime>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>
//#include <std_msgs/UInt16MultiArray.h>


#include "../matrix/MATRIX_LINK.h"
#include "adjuster.h"

using namespace std;
using namespace ros;
using namespace cv;

class Adjuster_Node// : public nodelet::Nodelet
{    
  public:
    string nodeName = "Adjuster_Node";
    string topic_target_roi_sub = "Adjuster_Node/target_roi";
    string topic_move_horizontal_pub = "Adjuster_Node/move_yaw";
    string topic_move_vertical_pub = "Adjuster_Node/move_pitch";
    string topic_control_pub = "Adjuster_Node/control_capture";
    
  private:
    string ver_ = "1.0";
    int queue_size = 1;
	#ifdef _ROS_LINK_H_
	ROS_Link *ros_link;
	#endif
    
    ros::NodeHandle n_;
    boost::shared_ptr<sensor_msgs::RegionOfInterest>/*sensor_msgs::ImagePtr*/ msg_target_roi;
    ros::SubscriberStatusCallback connect_cb_move_horzontal;
    ros::SubscriberStatusCallback disconnect_cb_move_horzontal;
    ros::SubscriberStatusCallback connect_cb_move_vertical;
    ros::SubscriberStatusCallback disconnect_cb_move_vertical;
      
    ros::Subscriber target_roi_sub_;
    ros::Publisher move_horizontal_pub_;
    ros::Publisher move_vertical_pub_;
    ros::Publisher control_pub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void connectCb_move_horzontal(const ros::SingleSubscriberPublisher& ssp)
    {
      if(move_horizontal_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_move_horizontal_pub.c_str());
      if(flag_sub_is_init) return;
      sub_init();
	  sub_topic_get();
	  flag_sub_is_init = true;
    }
    void disconnectCb_move_horzontal(const ros::SingleSubscriberPublisher& ssp)
    {
      if(move_horizontal_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_move_horizontal_pub.c_str());
      if(!flag_sub_is_init) return;
      sub_shutdown();
      flag_sub_is_init = false;
    }
    void connectCb_move_vertical(const ros::SingleSubscriberPublisher& ssp)
    {
      if(move_vertical_pub_.getNumSubscribers() > 1) return;
      ROS_INFO("%s connected !", topic_move_vertical_pub.c_str());
      if(flag_sub_is_init) return;
      sub_init();
	  sub_topic_get();
	  flag_sub_is_init = true;
    }
    void disconnectCb_move_vertical(const ros::SingleSubscriberPublisher& ssp)
    {
      if(move_vertical_pub_.getNumSubscribers() > 0) return;
      ROS_WARN("%s disconnected !", topic_move_vertical_pub.c_str());
      if(!flag_sub_is_init) return;
      sub_shutdown();
      flag_sub_is_init = false;
    }
    
  private:
    void target_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);
    void move_vertical_publish();
    void move_horizontal_publish();
    void control_publish();
    
  private:
    time_t past_time;
    time_t current_time;
    double duration_time_min = 1;
    Adjuster adj;
    vector<Rect> roi_box;
    float delay_time = 1;
    int ready_count = 3;
    int count_down = ready_count;
    int width = 640;
    int height = 480;
    float speed = 1;
    float tor   = 0.000005;
    float spd_pitch = 0;
    float spd_yaw = 0;
    float inc_rate_pitch = speed / (float)(height) * 2;
    float inc_rate_yaw = speed / (float)(width) * 2;
    float eps   = 1.0;
    float damping_v = 1000.0;
    
    float pitch_min = 0.3;
    float pitch_max = 0.7;
    float yaw_min = 0;
    float yaw_max = 1;
    float yaw = 0.5;//0~1
    float pitch = 0.5;//0~1
    float yaw_old = 0.5;
    float pitch_old = 0.5;
    float pitch_vect = 0;
    float yaw_vect = 0;
    float pitch_vect_old = 0;
    float yaw_vect_old = 0;
    float dest_vertical = 0;
    float dest_vertical_old = 0;
    float dest_horizontal = 0;
    float dest_horizontal_old = 0;
    bool pitch_direct = true;
    bool yaw_direct = true;
    bool pitch_fit = false;
    bool yaw_fit = false;
    bool flag_capture = false;
    bool flag_roi_box_update = false;
    bool flag_sub_is_init = false;

  public:
    Adjuster_Node(ros::NodeHandle& nh);
    ~Adjuster_Node();
    void run();
    
  public:
    virtual void init()
    {
      //n_ = getNodeHandle();
      msg_target_roi = boost::shared_ptr<sensor_msgs::RegionOfInterest>(new sensor_msgs::RegionOfInterest);
      connect_cb_move_horzontal    = boost::bind(&Adjuster_Node::connectCb_move_horzontal, this, _1);
      disconnect_cb_move_horzontal = boost::bind(&Adjuster_Node::disconnectCb_move_horzontal, this, _1);
      connect_cb_move_vertical    = boost::bind(&Adjuster_Node::connectCb_move_vertical, this, _1);
      disconnect_cb_move_vertical = boost::bind(&Adjuster_Node::disconnectCb_move_vertical, this, _1);
      pub_init();
      pub_topic_get();
      //sub_init();
      //sub_topic_get();
	  #ifdef _ROS_LINK_H_
	  ros_link = new ROS_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  #endif
    }
    
};

Adjuster_Node::Adjuster_Node(ros::NodeHandle& nh) : n_(nh)
{    
    adj.set_width(this -> width);
    adj.set_height(this -> height);
    sub_init();//temp init
}

Adjuster_Node::~Adjuster_Node()
{
	#ifdef _ROS_LINK_H_
	if(ros_link != NULL)
		delete ros_link;
	#endif
}

void Adjuster_Node::run()
{  
    if(!flag_roi_box_update)
    {
        return;
    }
    if(adj.run(this -> roi_box))
    {
        current_time = time(NULL);
        if(difftime(current_time, past_time) > duration_time_min)
        {
            spd_yaw = 0;
            spd_pitch = 0;
            yaw_vect_old = 0;
            pitch_vect_old = 0;
        }
        move_vertical_publish();
        move_horizontal_publish();
        cout << "find objects : " << adj.get_main_object_count() << " yaw = " << yaw << ", pitch = " << pitch << endl;
        this -> roi_box.clear();
        if(pitch_fit & yaw_fit & !flag_capture)
        {
            if(count_down == 0)
            {
                flag_capture = true;
                control_publish();
                count_down = ready_count;
            }
            else
            {
                count_down -= 1;
                ros::Duration(delay_time).sleep();
            }
        }
        else if(!pitch_fit || !yaw_fit)
        {
            flag_capture = false;
            count_down = ready_count;
        }
        past_time = current_time;
    }
}

void Adjuster_Node::target_roi_callBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
    if(!flag_roi_box_update)
    {
        if(msg -> width == 0 && msg -> height == 0)
        {
            flag_roi_box_update = true;
            run();
            return ;
        }
    }
    if(flag_roi_box_update)
    {
        if(msg -> width != 0 || msg -> height != 0)
        {
            flag_roi_box_update = false;
        }
        else
        {
            return;
        }
    }
	Rect roi;
    roi.width  = msg -> width;
    roi.height = msg -> height;
    roi.x = msg -> x_offset;
    roi.y = msg -> y_offset;
    
    /*roi.x      = msg -> x_offset - roi.width/2;
    roi.y      = msg -> y_offset - roi.height/2;
    int xx = roi.x + roi.width;
    int yy = roi.y + roi.height;
    roi.x      = (roi.x < 0)? 0 : roi.x;
    roi.y      = (roi.y < 0)? 0 : roi.y;
    roi.width  = (xx > width)? width - roi.x : xx - roi.x;
    roi.height = (yy > height)? height - roi.y : yy - roi.y;
    */
    this -> roi_box.push_back(roi);
    return ;
}

void Adjuster_Node::control_publish()
{
    cout << "Capturing ...\n";
    std_msgs::String msg;
    msg.data = "capture";
    control_pub_.publish(msg);
}

void Adjuster_Node::move_vertical_publish()
{
    std_msgs::Float32 msg;
    dest_vertical_old = dest_vertical;
    dest_vertical = (float)(adj.move_vertical)/(float)(height)*2;
    //pitch_vect_old = pitch_vect;
    //pitch_vect = this -> pitch - this -> pitch_old;
    //pitch_direct = (abs(dest_vertical) > abs(dest_vertical_old))? !pitch_direct : pitch_direct;
    if(abs(dest_vertical) < tor / 5)
    {
        pitch_fit = true;
        spd_pitch = 0;
        return;
    }
    else
    {
        spd_pitch += inc_rate_pitch;
        spd_pitch = (spd_pitch > speed)? speed : spd_pitch;
        spd_pitch = (spd_pitch < -1*speed)? -1*speed : spd_pitch;//useless ??
        this -> pitch += (pitch_direct)? -1 * dest_vertical * spd_pitch : dest_vertical * spd_pitch;// / (pow(damping_v*pitch_vect, 2) +eps); 
        this -> pitch = (this -> pitch > pitch_max)? pitch_max : this -> pitch;
        this -> pitch = (this -> pitch < pitch_min)? pitch_min : this -> pitch; 
        pitch_fit = false;
    }
    
    this -> pitch_old = this -> pitch;
    msg.data = this -> pitch;
    move_vertical_pub_.publish(msg);
}

void Adjuster_Node::move_horizontal_publish()
{
    std_msgs::Float32 msg;
    dest_horizontal_old = dest_horizontal;
    dest_horizontal = (float)(adj.move_horizontal)/(float)(width)*2;
    //yaw_vect_old = yaw_vect;
    //yaw_vect = this -> yaw - this -> yaw_old;
    //yaw_direct = (abs(yaw_vect) > abs(yaw_vect_old))? !yaw_direct : yaw_direct;
    if(abs(dest_horizontal) < tor / 5)
    {
        yaw_fit = true;
        spd_yaw = 0;
        return;
    }
    else
    {
        spd_yaw += inc_rate_yaw;
        spd_yaw = (spd_yaw > speed)? speed : spd_yaw;
        spd_yaw = (spd_yaw < -1*speed)? -1*speed : spd_yaw;//useless ??
        this -> yaw += (yaw_direct)? -1 * dest_horizontal * spd_yaw : dest_horizontal * spd_yaw;// / (pow(damping_v*yaw_vect, 2) +eps);
        this -> yaw = (this -> yaw > yaw_max)? yaw_max : this -> yaw;
        this -> yaw = (this -> yaw < yaw_min)? yaw_min : this -> yaw;   
        yaw_fit = false;
    }
    this -> yaw_old = this -> yaw;
    msg.data = this -> yaw;
    move_horizontal_pub_.publish(msg);
}


void Adjuster_Node::pub_topic_get()
{
    topic_move_horizontal_pub = move_horizontal_pub_.getTopic();
    topic_move_vertical_pub = move_vertical_pub_.getTopic();
    topic_control_pub = control_pub_.getTopic();
}

void Adjuster_Node::pub_init()
{
    ROS_INFO("Publisher %s initiating !", topic_move_horizontal_pub.c_str());
    move_horizontal_pub_ = n_.advertise< std_msgs::Float32 >(topic_move_horizontal_pub, queue_size, connect_cb_move_horzontal, disconnect_cb_move_horzontal);
    ROS_INFO("Publisher %s initiating !", topic_move_vertical_pub.c_str());
    move_vertical_pub_ = n_.advertise< std_msgs::Float32 >(topic_move_vertical_pub, queue_size, connect_cb_move_vertical, disconnect_cb_move_vertical);
    ROS_INFO("Publisher %s initiating !", topic_control_pub.c_str());
    control_pub_ = n_.advertise< std_msgs::String >(topic_control_pub, queue_size);
}

void Adjuster_Node::pub_shutdown()
{
    ROS_WARN("Publisher %s shuting down !", topic_move_horizontal_pub.c_str());
    move_horizontal_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_move_vertical_pub.c_str());
    move_vertical_pub_.shutdown();
    ROS_WARN("Publisher %s shuting down !", topic_control_pub.c_str());
    control_pub_.shutdown();
}

void Adjuster_Node::sub_topic_get()
{   
    topic_target_roi_sub = target_roi_sub_.getTopic();
}

void Adjuster_Node::sub_init()
{
    ROS_INFO("Subscriber %s initiating !", topic_target_roi_sub.c_str());
    target_roi_sub_ = n_.subscribe(topic_target_roi_sub.c_str(), queue_size, &Adjuster_Node::target_roi_callBack, this);
}

void Adjuster_Node::sub_shutdown()
{
    ROS_WARN("Subscriber %s shuting down !", topic_target_roi_sub.c_str());
    target_roi_sub_.shutdown();
}
#endif
