#!/usr/bin/env python

import sys

import rospy

from sensor_msgs.msg import Imu

from wideboy_imu import ImuReader


def main(args):
    rospy.init_node('mip_rover_imu')

    imu_reader = ImuReader()

    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

    rate = rospy.Rate(imu_reader.rate)
    while not rospy.is_shutdown():
        imu_msg = imu_reader.read_from_imu()
        imu_pub.publish(imu_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
