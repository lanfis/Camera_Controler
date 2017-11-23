#pragma once
#ifndef _ROS_SERIAL_H_
#define _ROS_SERIAL_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "serial_handler.h"
#include "matrix/MATRIX_LINK.h"

using namespace std;
using namespace ros;

class ROS_Serial
{    
  public:
    string nodeName = "ROS_Serial";
    string topic_data_sub = "ROS_Serial/data_sub";
    string topic_data_pub = "ROS_Serial/data_pub";
    string topic_manual_set_sub = "ROS_Serial/manual_set";
    string topic_port_name_sub = "ROS_Serial/port_name";
    string topic_baudrate_sub = "ROS_Serial/baudrate";
    
  private:
	boost::shared_ptr< std_msgs::String > msg_data_;
    int queue_size = 32;
	ROS_Link *ros_link;
	serial_handler serial;
    
    ros::NodeHandle n_;
    ros::SubscriberStatusCallback connect_cb_data;
    ros::SubscriberStatusCallback disconnect_cb_data;
      
    ros::Subscriber data_sub_;
    ros::Publisher  data_pub_;
    ros::Subscriber manual_set_sub_;
    ros::Subscriber port_name_sub_;
    ros::Subscriber baudrate_sub_;
    
  private:
    void pub_topic_get();
    void pub_init();
    void pub_shutdown();
    void sub_topic_get();
    void sub_init();
    void sub_shutdown();
    void sub_manual_topic_get();
    void sub_manual_init();
    void sub_manual_shutdown();
    void connectCb_data(const ros::SingleSubscriberPublisher& ssp)
    {
      if(data_pub_.getNumSubscribers() > 1) return;
	  string status = topic_data_pub + " connected !";
      OUT_INFO(nodeName.c_str(), status);
	  string port_name = serial.get_port();
	  string baudrate;
	  stringstream token; token << serial.get_baudrate(); token >> baudrate;
	  if(serial.open_port())
	  {
		status = "opening port : " + port_name + " / " + baudrate;
		OUT_INFO(nodeName.c_str(), status);
	  }
	  else
	  {
		status = "opening port : " + port_name + " / " + baudrate + " fail !";
		OUT_WARN(nodeName.c_str(), status);
	  }
      sub_init();
	  sub_topic_get();
    }
    void disconnectCb_data(const ros::SingleSubscriberPublisher& ssp)
    {
      if(data_pub_.getNumSubscribers() > 0) return;
	  string status = topic_data_pub + " disconnected !";
      OUT_WARN(nodeName.c_str(), status);
	  string port_name = serial.get_port();
	  string baudrate;
	  stringstream token; token << serial.get_baudrate(); token >> baudrate;
	  if(serial.close_port())
	  {
		status = "closing port : " + port_name + " / " + baudrate;
		OUT_INFO(nodeName.c_str(), status);
	  }
	  else
	  {
		status = "closing port : " + port_name + " / " + baudrate + " fail !";
		OUT_WARN(nodeName.c_str(), status);
	  }
      sub_shutdown();
    }
    
  private:
    void data_publish();
    void data_callBack(const std_msgs::String::ConstPtr& msg);
    void manual_set_callBack(const std_msgs::Bool::ConstPtr& msg);
    void port_name_callBack(const std_msgs::String::ConstPtr& msg);
    void baudrate_callBack(const std_msgs::Int32::ConstPtr& msg);
    
  private:
	string data;
	int data_size;
	bool flag_manual_set = false;

  public:
    ROS_Serial(ros::NodeHandle& nh);
    ~ROS_Serial();
    void run();
    
  public:
    virtual void init()
    {
      connect_cb_data    = boost::bind(&ROS_Serial::connectCb_data, this, _1);
      disconnect_cb_data = boost::bind(&ROS_Serial::disconnectCb_data, this, _1);
      pub_init();
      pub_topic_get();
	  
	  ros_link = new ROS_Link(n_, nodeName);
	  ros_link -> pub_init();
	  ros_link -> sub_init();
	  ros_link -> add_cell(data_sub_, topic_data_sub);
	  ros_link -> add_cell(data_pub_, topic_data_pub);
	  ros_link -> add_cell(port_name_sub_, topic_port_name_sub);
    }
};

ROS_Serial::ROS_Serial(ros::NodeHandle& nh) : n_(nh), msg_data_(new std_msgs::String)
{    
}

ROS_Serial::~ROS_Serial()
{
	if(ros_link != NULL)
		delete ros_link;
	serial.close_port();
}

void ROS_Serial::run()
{
	data_publish();
}

void ROS_Serial::data_publish()
{
	data_size = serial.read_port(data);
	if(data_size > 0)
	{
		msg_data_ -> data = data;
		data_pub_.publish(msg_data_);
	}
}

void ROS_Serial::data_callBack(const std_msgs::String::ConstPtr& msg)
{
	data = msg -> data;
    data_size = serial.write_port(data);
}

void ROS_Serial::manual_set_callBack(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg -> data & !flag_manual_set)
	{
//		string status = " manual setting activating !";
//		OUT_INFO(nodeName.c_str(), status);
		sub_manual_init();
		sub_manual_topic_get();
		flag_manual_set = msg -> data;
	}
	if(!msg -> data & flag_manual_set)
	{
//		string status = " manual setting deactivating !";
//		OUT_INFO(nodeName.c_str(), status);
		sub_manual_shutdown();
		flag_manual_set = msg -> data;
	}
}

void ROS_Serial::port_name_callBack(const std_msgs::String::ConstPtr& msg)
{
	string status = " port name : " + msg -> data;
	OUT_INFO(nodeName.c_str(), status);
	serial.save_status();
    serial.set_port(msg -> data);
	if(!serial.open_port())
	{
		status = " port : " + msg -> data + " open fail !" ;
		OUT_WARN(nodeName.c_str(), status);
		serial.load_status();
	}
}

void ROS_Serial::baudrate_callBack(const std_msgs::Int32::ConstPtr& msg)
{
	string status = " baudrate : " + msg -> data;
	OUT_INFO(nodeName.c_str(), status);
	serial.save_status();
    serial.set_baudrate(msg -> data);
	if(!serial.open_port())
	{
		stringstream token;
		token << msg -> data;
		string tmp;
		token >> tmp;
		status = " baudrate : " + tmp + " open fail !" ;
		OUT_WARN(nodeName.c_str(), status);
		serial.load_status();
	}
}

void ROS_Serial::pub_topic_get()
{
    topic_data_pub = data_pub_.getTopic();
}

void ROS_Serial::pub_init()
{
	string status = "Publisher " + topic_data_pub + " initiating !";
	OUT_INFO(nodeName.c_str(), status);
    data_pub_ = n_.advertise< std_msgs::String >(topic_data_pub, queue_size, connect_cb_data, disconnect_cb_data);
}

void ROS_Serial::pub_shutdown()
{
	string status = "Publisher " + topic_data_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    data_pub_.shutdown();
}

void ROS_Serial::sub_topic_get()
{   
    topic_data_sub = data_sub_.getTopic();
    topic_manual_set_sub = manual_set_sub_.getTopic();
}

void ROS_Serial::sub_init()
{
	string status = "Subscriber " + topic_data_sub + " initiating !";
	OUT_INFO(nodeName.c_str(), status);
    data_sub_ = n_.subscribe(topic_data_sub.c_str(), queue_size, &ROS_Serial::data_callBack, this);
	status = "Subscriber " + topic_manual_set_sub + " initiating !";
	OUT_INFO(nodeName.c_str(), status);
    manual_set_sub_ = n_.subscribe(topic_manual_set_sub.c_str(), queue_size, &ROS_Serial::manual_set_callBack, this);
}

void ROS_Serial::sub_shutdown()
{
	string status = "Subscriber " + topic_data_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    data_sub_.shutdown();
	status = "Subscriber " + topic_manual_set_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    manual_set_sub_.shutdown();
}

void ROS_Serial::sub_manual_topic_get()
{   
    topic_port_name_sub = port_name_sub_.getTopic();
    topic_baudrate_sub = baudrate_sub_.getTopic();
}

void ROS_Serial::sub_manual_init()
{
	string status = "Subscriber " + topic_port_name_sub + " initiating !";
	OUT_INFO(nodeName.c_str(), status);
    port_name_sub_ = n_.subscribe(topic_port_name_sub.c_str(), queue_size, &ROS_Serial::port_name_callBack, this);
	status = "Subscriber " + topic_baudrate_sub + " initiating !";
	OUT_INFO(nodeName.c_str(), status);
    baudrate_sub_ = n_.subscribe(topic_baudrate_sub.c_str(), queue_size, &ROS_Serial::baudrate_callBack, this);
}

void ROS_Serial::sub_manual_shutdown()
{
	string status = "Subscriber " + topic_port_name_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    port_name_sub_.shutdown();
	status = "Subscriber " + topic_baudrate_sub + " shuting down !";
    OUT_WARN(nodeName.c_str(), status);
    baudrate_sub_.shutdown();
}
#endif
