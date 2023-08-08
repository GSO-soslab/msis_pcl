#!/usr/bin/env python3
import rospy
import numpy as np
from ping360_msgs.msg import SonarEcho

def listener():
    rospy.Subscriber("/echo", SonarEcho, callback= cbEcho)


def cbEcho(msg):
    print(round(np.rad2deg(msg.angle)))

if __name__ == "__main__":
    rospy.init_node("reading_angles")
    listener()
    rospy.spin()