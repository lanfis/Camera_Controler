#pragma once
#ifndef _ADJUSTER_H_
#define _ADJUSTER_H_

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;


class Adjuster
{
    private:
      string ver_ = "1.0";
      
    public:
      void set_width(int& width){this -> width = width; ratio_line_point_set();};
      void set_height(int& height){this -> height = height; ratio_line_point_set();};
      int get_width(){return this -> width;};
      int get_height(){return this -> height;};
      void set_roi_target(vector<Rect>& rois){roi_target_ = rois;};
	  int move_horizontal = 0;
	  int move_vertical = 0;
	  int move_frwdback = 0;
      
    private:
      int width = 640;
      int height = 480;
      
	  float tor_horizontal_ratio_ = 0.01;
	  float tor_vertical_ratio_ = 0.01;
	  float tor_frwdback_ratio_ = 0.01;
	  vector<Rect> roi_target_;
	  int main_object_count_ = 0;
	  int tor_group_div_value_ = 19200;
	  vector<float> ratio_line_ratio_;
	  vector<Point> ratio_line_point_;
      
      void ratio_line_point_set();
      void region_size_sort(vector<Rect>& region);
      void find_main_object(vector<Rect>& region, int& count, int& div_value);
      bool policy_decision();
	  
    public:
      Adjuster();
      ~Adjuster();
      bool run(vector<Rect>& rois);
};

Adjuster::Adjuster()
{
	ratio_line_ratio_.push_back(0.4);
	ratio_line_ratio_.push_back(0.6);
	ratio_line_point_set();
}

Adjuster::~Adjuster()
{
}

bool Adjuster::run(vector<Rect>& rois)
{
    if(rois.size() == 0) return false;
    this -> roi_target_ = rois;
    //cout << "Find object :";
    find_main_object(roi_target_, main_object_count_, tor_group_div_value_);    
    cout << main_object_count_ << endl;
    //cout << "Making decision ..." << endl;
    return policy_decision();
}

bool Adjuster::policy_decision()
{    
    float tor_horizontal_ratio_ = 0.1;
    float tor_vertical_ratio_ = 0.1;
    float tor_frwdback_ratio_ = 0.1;
    
	if(main_object_count_ == 0)
	{
        move_horizontal = 0;
        move_vertical = 0;
        move_frwdback = 0;
	    return false;
    }
    if(main_object_count_ > roi_target_.size())
        return false;
    
    if(main_object_count_ % 2 == 0)
    {
        move_horizontal = (abs(ratio_line_point_[1].x - (roi_target_[0].x + roi_target_[1].x) / 2) > width * tor_horizontal_ratio_)?
                            ratio_line_point_[1].x - (roi_target_[0].x + roi_target_[1].x) / 2 : 0;
        move_vertical =   (abs(ratio_line_point_[1].y - (roi_target_[0].y + roi_target_[1].y) / 2) > height * tor_vertical_ratio_)?
                            ratio_line_point_[1].y - (roi_target_[0].y + roi_target_[1].y) / 2 : 0;
        return true;
    }    
    if(main_object_count_ % 2 == 1)
    {
        move_horizontal = (abs(ratio_line_point_[1].x - roi_target_[0].x) > width * tor_horizontal_ratio_)?
                            ratio_line_point_[1].x - roi_target_[0].x : 0;
        move_vertical =   (abs(ratio_line_point_[1].y - roi_target_[0].y) > height * tor_vertical_ratio_)?
                            ratio_line_point_[1].y - roi_target_[0].y : 0;
        return true;
    }
}

void Adjuster::find_main_object(vector<Rect>& region, int& count, int& div_value)
{
    region_size_sort(region);
    vector<int> region_size;
    region_size.clear();
    count = 0;
    if(region.size() > 0)
    {
        count = 1;
        region_size.push_back(region[0].width * region[0].height);
    }
    for(int i = 1; i < region.size(); i++)
    {
        int tmp_size = region[i].width * region[i].height;
        region_size.push_back(tmp_size);
        if(region_size[i-1] - region_size[i] < div_value)
        {
            count += 1;
        }
        else
        {
            break;
        }
    }
}

void Adjuster::region_size_sort(vector<Rect>& region)
{
    for(int i = 0; i < region.size(); i++)
    {
        int cmp_size = region[i].width * region[i].height;
        for(int j = i+1; j < region.size(); j++)
        {
            int region_size = region[j].width * region[j].height;
            if(region_size > cmp_size)
            {
                cmp_size = region_size;
                Rect tmp;
                tmp = region[i];
                region[i] = region[j];
                region[j] = tmp;
            }
        }
    }
}

void Adjuster::ratio_line_point_set()
{
    ratio_line_point_.clear();
    Point p;
    //center
    p.x = width / 2;
    p.y = height / 2;
    ratio_line_point_.push_back(p);
    //top center
    p.x = width / 2;
    p.y = height * ratio_line_ratio_[0];
    ratio_line_point_.push_back(p);
    //bottom center
    p.x = width / 2;
    p.y = height * ratio_line_ratio_[1];
    ratio_line_point_.push_back(p);
    //top left corner
    p.x = width * ratio_line_ratio_[0];
    p.y = height * ratio_line_ratio_[0];
    ratio_line_point_.push_back(p);
    //bottom left corner
    p.x = width * ratio_line_ratio_[0];
    p.y = height * ratio_line_ratio_[1];
    ratio_line_point_.push_back(p);
    //top right corner
    p.x = width * ratio_line_ratio_[1];
    p.y = height * ratio_line_ratio_[0];
    ratio_line_point_.push_back(p);
    //bottom right corner
    p.x = width * ratio_line_ratio_[1];
    p.y = height * ratio_line_ratio_[1];
    ratio_line_point_.push_back(p);
}



#endif
