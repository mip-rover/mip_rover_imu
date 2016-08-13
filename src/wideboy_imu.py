import numpy as np

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

from Adafruit_BNO055 import BNO055


class ImuReader:
    def __init__(self):
        calib_path = rospy.get_param("~calib_path")

        # Start is up
        self.bno = BNO055.BNO055(rst=4)
        if not self.bno.begin():
            raise RuntimeError('Failed to initialize BNO055')

        # Run self test
        status, self_test, error = self.bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            print('System error: {0}'.format(error))

        # Display version information
        sw, bl, accel, mag, gyro = self.bno.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

        self.rate = 100

        # self._GRAVITY = 9.81

    def read_from_imu(self):
        while True:
            try:
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = 'imu'

                orientation = self.bno.read_quaternion()
                linear_acceleration = self.bno.read_linear_acceleration()
                angular_velocity = self.bno.read_gyroscope()

                imu_msg.orientation_covariance[0] = -1
                imu_msg.linear_acceleration_covariance[0] = -1
                imu_msg.angular_velocity_covariance[0] = -1

                imu_msg.orientation = Quaternion(orientation[1],
                                                 orientation[2],
                                                 orientation[3],
                                                 orientation[0])
                imu_msg.linear_acceleration = Vector3(linear_acceleration[0],
                                                      linear_acceleration[1],
                                                      linear_acceleration[2])
                imu_msg.angular_velocity = Vector3(np.deg2rad(angular_velocity[0]),
                                                   np.deg2rad(angular_velocity[1]),
                                                   np.deg2rad(angular_velocity[2]))
                return imu_msg
            except IOError:
                pass
