#include <ros/ros.h>
#include "mbs_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "mbs_base_node");
    MbsBase mbs;
    ros::spin();
    return 0;
}
