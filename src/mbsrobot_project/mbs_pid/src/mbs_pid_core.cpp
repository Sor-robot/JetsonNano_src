#include "mbs_pid/mbs_pid_core.h"

MbsPID::MbsPID()
{
}

MbsPID::~MbsPID()
{
}

void MbsPID::publishMessage(ros::Publisher *pub_message)
{
  mbs_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void MbsPID::messageCallback(const mbs_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void MbsPID::configCallback(mbs_pid::mbsPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
