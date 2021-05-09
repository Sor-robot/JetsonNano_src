// simple_camera.cpp
// MIT License
// Copyright (c) 2019 JetsonHacks
// See LICENSE for OpenCV license and additional information
// Using a CSI camera (such as the Raspberry Pi Version 2) connected to a 
// NVIDIA Jetson Nano Developer Kit using OpenCV
// Drivers for the camera and OpenCV are included in the base image

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

// #include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}



using namespace sensor_msgs;

int main (int argc, char **argv) {

    sensor_msgs::ImagePtr msg;
    cv::Mat img;

    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    ros::init(argc, argv, "csi_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
	std::cout<<"Failed to open camera."<<std::endl;
	return (-1);
    }

//    cv::namedWindow("CSI image", cv::WINDOW_AUTOSIZE);
    ros::Rate loop_rate(5);
    while(nh.ok())
    {
    	if (!cap.read(img)) {
		std::cout<<"Capture read error"<<std::endl;
		break;
	}
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	if(0)
	{
	    cv::imshow("CSI image", img);
	    cv::waitKey(30);
	}
	else
	{
	    pub.publish(msg);
            //cv::imshow("CSI image", img);
	    cv::waitKey(3);
            //ros::spin();
	}

    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}


