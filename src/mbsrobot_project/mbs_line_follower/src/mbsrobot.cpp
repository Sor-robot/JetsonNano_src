/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file mbsrobot.cpp
*@author Sudarshan Raghunathan
*@brief  Functions definitions for mbsrobot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "mbsrobot.hpp"
#include "mbs_line_follower/pos.h"

void mbsrobot::dir_sub(mbs_line_follower::pos msg) {
    mbsrobot::dir = msg.direction;
}
void mbsrobot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {
    // If direction is left
    if (mbsrobot::dir == 0) {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.1;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Left");
    }
    // If direction is straight
    if (mbsrobot::dir == 1) {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Straight");
    }
    // If direction is right
    if (mbsrobot::dir == 2) {
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.1;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Right");
    }
    // If robot has to search
    if (mbsrobot::dir == 3) {
        velocity.linear.x = 0;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
}
