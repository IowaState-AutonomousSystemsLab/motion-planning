#!/usr/bin/env python3

import os
import rospy
import os
import math

import numpy as np
from pathlib import Path

from include.PIDF import PIDF

from sensor_msgs.msg import Imu
from as_msgs.msg import WheelOdometry
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped
from offset_calculator import OffsetCalculator


class HeadingTracker():
    def __init__(self):
        rospy.loginfo("Initializing")

        #PID init
        self.ref = 0

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        self.imu_sub = rospy.Subscriber("/duck7/imu_node/data", Imu, callback=self.integrate)

        #Wheel speed speed tracking
        self.left_speed  = -255
        self.right_speed = -255
        self.odometery_sub = rospy.Subscriber(f"~wheel_odometry/odometry",WheelOdometry,self.odometry_cb)

        #Wheel speed publisher
        self.wheel_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        #Wheel speed PIDs
        self.left_pid = PIDF(1,0,0.03,0)
        self.right_pid = PIDF(1,0,0.03,0)

        # rospy.loginfo("     CALCULATING IMU OFFSET")
        # offfset_calc = OffsetCalculator()

        rospy.loginfo("initialized")

    def truncate(self, num:float, dec_places:int):
        mult = 10**dec_places
        return int(num * ()) * mult / mult


    def integrate(self, imu_out:Imu):
        """
        Will integrate the yaw rate command from the Gyroscope. Automatically does not get called if #IMU (line 25 is commented out) 
        """
        self.heading = self.heading + (self.truncate(imu_out.angular_velocity.x, 4) - 0.005)
        rospy.logdebug(" + Heading Values = " + str(self.heading))


    def track_heading(self):
        """
        Uses a modified PID to track wheel speeds
        """
        #CONSTANTS
        kP = 0.02

        error = self.ref - self.heading
        u = kP * error
        rospy.logdebug(" + + PID Control inputs = " + str(u))
        left_goal = self.left_speed + u
        right_goal = self.right_speed - u

        self.set_wheel_speeds(left_goal, right_goal)


    def set_wheel_speeds(self, left_speed:float, right_speed:float):
        
        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        current_time = rospy.get_time()

        wheelsCmd.vel_left = self.left_pid.update(left_speed,0.1)
        wheelsCmd.vel_right = self.right_pid.update(right_speed,0.1)
        self.wheel_pub.publish(wheelsCmd)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.track_heading()
            rate.sleep()


    def on_shutdown(self):
    #send a zero velocity wheel command

        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        wheelsCmd.vel_right = 0
        wheelsCmd.vel_left = 0
        self.wheel_pub.publish(wheelsCmd)

    def odometry_cb(self,data):
        self.left_speed = data.left_wheel_velocity
        self.right_speed = data.right_wheel_velocity
        self.log(data)
        
if __name__ == "__main__":
    rospy.init_node(f"offset_calculator_node", log_level=rospy.DEBUG)
    node = HeadingTracker()
    # node.run()
    rospy.spin()
