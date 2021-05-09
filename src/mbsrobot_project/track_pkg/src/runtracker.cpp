#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <math.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 0.6
#define Min_linear_speed 0.4
#define Min_distance 1.5
#define Max_distance 5.0
#define Max_rotation_speed 0.75

static const double PI = 3.1415926;

float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.004;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1 = 100;
int ERROR_OFFSET_X_left2 = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 540;
int follow_target;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}



void drawCross(cv::Mat img, cv::Rect point, cv::Scalar color, int size, int thickness)
{
	cv::line(img, cvPoint(point.x - size / 2, point.y), cvPoint(point.x + size / 2, point.y), color, thickness, 8, 0);
	cv::line(img, cvPoint(point.x, point.y - size / 2), cvPoint(point.x, point.y + size / 2), color, thickness, 8, 0);
	return;
}

cv::Rect get_center(cv::Rect srcRect)
{
	cv::Rect res;
	res.x = srcRect.x + srcRect.width / 2;
	res.y = srcRect.y + srcRect.height / 2;
	res.width = srcRect.width;
	res.height = srcRect.height;
	return res;
}

int get_area(cv::Rect srcRect)
{
	return (srcRect.width * srcRect.height);
}

void SendSpeed(cv::Rect site)
{
	double angle_speed, line_speed = 0;
	angle_speed = (1 / (sin(site.x * (PI) / rgbimage.cols)) - 1) * 1.5;
  if(site.x > rgbimage.cols / 2)
  {
      angle_speed *= -1;
  }
	if (angle_speed > 255 || angle_speed < -255)
	{
		angle_speed = 0;
	}
	if (site.y < 400 || site.y > 100)
	{
		line_speed = ((double)site.y - (double)follow_target) / 240.0;
	}
  linear_speed = line_speed;
  rotation_speed = angle_speed;
	std::cout << "Line speed = " << line_speed << ", Angle speed =  " << angle_speed << std::endl;
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  
public:
  ros::Publisher pub;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1, 
      &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image", 1, 
      &ImageConverter::depthCb, this);
    pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    cv::namedWindow(RGB_WINDOW);
    //cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    //cv::destroyWindow(DEPTH_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.copyTo(rgbimage);

    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        // if (selectRect.width <= 0 || selectRect.height <= 0)
        // {
        //     bRenewROI = false;
        //     //continue;
        // }
        tracker.init(selectRect, rgbimage);
        follow_target = selectRect.y ;
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
			std::string point_set;
			result = tracker.update(rgbimage);

			point_set = "X = " +std::to_string(result.x) + ", Y = " + std::to_string(result.y);

			cv::putText(rgbimage, point_set, cvPoint(result.x, result.y), 1, 1, cv::Scalar(0, 255, 255));
			drawCross(rgbimage, get_center(result), cv::Scalar(0, 255, 255), 10, 2);
      cv::rectangle(rgbimage, result, cv::Scalar(0, 255, 255), 1, 8);
      SendSpeed(get_center(result));
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  		cv_ptr->image.copyTo(depthimage);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  	}

    if(enable_get_depth)
    {
      dist_val[0] = depthimage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
      dist_val[1] = depthimage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
      dist_val[2] = depthimage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
      dist_val[3] = depthimage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
      dist_val[4] = depthimage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;

      float distance = 0;
      int num_depth_points = 5;
      for(int i = 0; i < 5; i++)
      {
        if(dist_val[i] > 0.4 && dist_val[i] < 10.0)
          distance += dist_val[i];
        else
          num_depth_points--;
      }
      distance /= num_depth_points;

      //calculate linear speed
      if(distance > Min_distance)
        linear_speed = distance * k_linear_speed + h_linear_speed;
      else
        linear_speed = 0;

      if(linear_speed > Max_linear_speed)
        linear_speed = Max_linear_speed;

      //calculate rotation speed
      int center_x = result.x + result.width/2;
      if(center_x < ERROR_OFFSET_X_left1) 
        rotation_speed =  Max_rotation_speed;
      else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
      else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
      else if(center_x > ERROR_OFFSET_X_right2)
        rotation_speed = -Max_rotation_speed;
      else 
        rotation_speed = 0;

      std::cout <<  "linear_speed = " << linear_speed << "  rotation_speed = " << rotation_speed << std::endl;

      std::cout <<  dist_val[0]  << " / " <<  dist_val[1] << " / " << dist_val[2] << " / " << dist_val[3] <<  " / " << dist_val[4] << std::endl;
      std::cout <<  "distance = " << distance << std::endl;
    }

  	//cv::imshow(DEPTH_WINDOW, depthimage);
  	cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
	ImageConverter ic;
 
	while(ros::ok())
	{
		ros::spinOnce();

        geometry_msgs::Twist twist;
        twist.linear.x = linear_speed; 
        twist.linear.y = 0; 
        twist.linear.z = 0;
        twist.angular.x = 0; 
        twist.angular.y = 0; 
        twist.angular.z = rotation_speed;
        ic.pub.publish(twist);

        if (cvWaitKey(33) == 'q')
            break;
	}

	return 0;
}

