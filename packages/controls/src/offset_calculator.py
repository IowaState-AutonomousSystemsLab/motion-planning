#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np

# from datetime import datetime

import sys
import os
from pathlib import Path
from sensor_msgs.msg import Imu
import rostopic


class OffsetCalculator():
    def __init__(self):
        print("initialzing starting")
        self.start_time = rospy.get_rostime().secs

        self.x_velo = 0
        self.y_velo = 0
        self.z_velo = 0

        self.y_accel = 0
        self.x_accel = 0
        self.z_accel = 0

        self.callback_count = 0
        self.imu_sub = rospy.Subscriber("/duck7/imu_node/data", Imu, callback=self.calculate_average)

        print("initialzed")

    def calculate_average(self, data):
        self.x_velo = self.x_velo + data.angular_velocity.x
        self.y_velo = self.y_velo + data.angular_velocity.y
        self.z_velo = self.z_velo + data.angular_velocity.z

        self.x_accel = self.x_accel + data.linear_acceleration.x
        self.y_accel = self.y_accel + data.linear_acceleration.y
        self.z_accel = self.z_accel + data.linear_acceleration.z

        self.callback_count += 1
        print(f"callback count: {self.callback_count}")

        # when time greater than 10.1 seconds, call imu_sub.unregister()
        if rospy.get_rostime().secs - self.start_time >= 10:
            print(f"x velocity offset = {self.x_velo/self.callback_count}")
            print(f"y velocity offset = {self.y_velo/self.callback_count}")
            print(f"z velocity offset = {self.z_velo/self.callback_count}")

            print(f"x acceleration offset = {self.x_accel/self.callback_count}")
            print(f"y acceleration offset = {self.y_accel/self.callback_count}")
            print(f"z acceleration offset = {self.z_accel/self.callback_count}")

            self.imu_sub.unregister()
            rospy.signal_shutdown("Node is done calculating offsets")




if __name__ == "__main__":
    rospy.init_node("offset_calculator_node")
    node = OffsetCalculator()
    rospy.spin()
