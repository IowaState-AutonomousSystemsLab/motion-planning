#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np

import os
from pathlib import Path
from sensor_msgs.msg import Imu
from include.PIDF import PIDF
from as_msgs.msg import WheelOdometry


class HeadingTracker():
    def __init__(self):
        print("Initializing")

        # IMU
        self.heading = 0
        self.imu_sub = rospy.Subscriber("/duck7/imu_node/data", Imu, callback=self.integrate)
        self.wheel_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.odometery_sub = rospy.Subscriber(f"~wheel_odometry/odometry",WheelOdometry,self.odometry_cb)

        #Whel speed PIDs
        self.left_pid = PIDF(1,1,0.03,0)
        self.right_pid = PIDF(1,1,0.03,0)

        #Wheel Speeds
        self.left_wheel_sub = rospy.Subscriber("/duck7/")



        print("initialzed")

    def integrate(self, imu_out:Imu):
        self.heading += imu_out.angular_velocity.x

    def track_heading():
        


if __name__ == "__main__":
    rospy.init_node("offset_calculator_node")
    node = HeadingTracker()
    rospy.spin()
