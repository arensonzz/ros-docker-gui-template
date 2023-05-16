#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // initialize the ROS system
    ros::init(argc, argv, "hello_ros");

    // establish this program as a ROS node
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Hello, ROS!");
    return 0;
}
