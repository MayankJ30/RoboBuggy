#!usr/bin/env python
import rospy
from robobuggy.msg import GPS

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Lat_deg %f\tLong_deg %f", data.Lat_deg, data.Long_deg)

def listener():
    rospy.init_node('GPS_TestPy', anonymous=True)

    rospy.Subscriber("GPS", GPS, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
