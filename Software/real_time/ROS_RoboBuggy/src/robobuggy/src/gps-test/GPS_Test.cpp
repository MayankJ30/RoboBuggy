#include "gps-test/GPS_Test.h"

void gpsCallBack(const robobuggy::GPS& msg) {
    ROS_INFO("Lat_deg: %f\tLong_deg: %f\n", msg.Lat_deg, msg.Long_deg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_Test");
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("GPS", 1000, gpsCallBack);

    ros::spin();

    return 0;
}
