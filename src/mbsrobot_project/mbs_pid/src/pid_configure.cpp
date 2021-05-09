#include "mbs_pid/mbs_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_configure");
  ros::NodeHandle nh;

  MbsPID *mbs_pid = new MbsPID();

  dynamic_reconfigure::Server<mbs_pid::mbsPIDConfig> dr_srv;
  dynamic_reconfigure::Server<mbs_pid::mbsPIDConfig>::CallbackType cb;
  cb = boost::bind(&MbsPID::configCallback, mbs_pid, _1, _2);
  dr_srv.setCallback(cb);

  double p;
  double d;
  double i;
  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("p", p, 0.1);
  pnh.param("d", d, 0.10);
  pnh.param("i", i, 0.10);
  pnh.param("rate", rate, 1);

  ros::Publisher pub_message = nh.advertise<mbs_msgs::PID>("pid", 10);

  ros::Rate r(rate);

  while (nh.ok())
  {
    mbs_pid->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
