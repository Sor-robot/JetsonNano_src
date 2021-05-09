#ifndef SR_MBS_PID_CORE_H
#define SR_MBS_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "mbs_msgs/PID.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <mbs_pid/mbsPIDConfig.h>

class MbsPID
{
public:
  MbsPID();
  ~MbsPID();
  void configCallback(mbs_pid::mbsPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const mbs_msgs::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif
