#!/usr/bin/env python
import roslib; roslib.load_manifest('slider_gui')
import rospy
import time
from sensor_msgs.msg import Image
def callback(data):
    time.sleep(0.001);

def listener():
    rospy.init_node('image_hack', anonymous=True)
    rospy.Subscriber("/head_mount_kinect/rgb/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
