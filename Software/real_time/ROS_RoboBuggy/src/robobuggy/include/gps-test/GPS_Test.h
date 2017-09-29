#ifndef ROS_ROBOBUGGY_GPS_TEST1_H
#define ROS_ROBOBUGGY_GPS_TEST1_H

#include "ros/ros.h"
#include "robobuggy/GPS.h"
#include "std_msgs/String.h"

class GPS_Test1
{
public:
    void gpsCallBack(const std_msgs::String::ConstPtr& msg);
    int main(int argc, char **argv);
private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
};

#endif
