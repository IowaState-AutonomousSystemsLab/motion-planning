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
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import WheelsCmdStamped
from offset_calculator import OffsetCalculator
from include.differential_drive_kinematics import DifferentialDriveKinematics


class HeadingTracker():
    def __init__(self):
        rospy.loginfo("Initializing")

        #Robot params      -------- CHANGE THIS  ---------       
        self.robot_width = 25.4     # in cm
        self.wheel_radius = 2.12    # in cm

        self.heading_kP = 0.02
        self.heading_kI = 0
        self.heading_kD = 0

        self.left_wheel_kP = 0.02
        self.left_wheel_kI = 0
        self.left_wheel_kD = 0

        self.right_wheel_kP = 0.02
        self.right_wheel_kI = 0
        self.right_wheel_kD = 0

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        self.imu_sub = rospy.Subscriber("/duck7/imu_node/data", Imu, callback=self.integrate)
        
        # IMU -------------------------- Add optitrack position --------------------------------------------
        self.curr_pose = -255
        self.pose_sub = rospy.Subscriber("<CHANGE THIS>", PoseStamped, callback = self.update_pos)

        # Wheel speed speed tracking
        self.left_speed  = -255
        self.right_speed = -255
        self.odometery_sub = rospy.Subscriber(f"~wheel_odometry/odometry", WheelOdometry,self.odometry_cb)

        # Wheel speed publisher
        self.wheel_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Wheel speed PIDs
        self.left_pid  = PIDF(self.left_wheel_kP,  self.left_wheel_kI,  self.left_wheel_kD,  0)
        self.right_pid = PIDF(self.right_wheel_kP, self.right_wheel_kI, self.right_wheel_kD, 0)
        self.heading_pid = PIDF(self.heading_kP, self.heading_kI, self.heading_kD, 0)
        
        # Globals
        x_ref = -255
        y_ref = -255

        # User IO
        self.enable
        self.enable_pub = rospy.Publisher(f"/{os.environ['VEHICLE_NAME']}/supervisor/experiment_enable", Bool)
        self.enable_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/supervisor/experiment_enable", Bool, self.enable_handler)

        # rospy.loginfo("     CALCULATING IMU OFFSET")
        # offfset_calc = OffsetCalculator()

        # Make the robot stop cleanly
        rospy.on_shutdown(self.zero_command)

        rospy.loginfo("initialized")

    def truncate(self, num:float, dec_places:int):
        mult = 10**dec_places
        return int(num * ()) * mult / mult
    
    def on_goal_reached(self, data: Bool):
        self.enable = False
        self.zero_command()

    def enable_handler(self, data: Bool):
        rate = rospy.Rate(10)
        while self.enable:
            self.track_heading_and_speed()
            rate.sleep()

    def integrate(self, imu_out:Imu):
        """
        Will integrate the yaw rate command from the Gyroscope. Automatically does not get called if #IMU (line 25 is commented out) 
        """
        self.heading = self.heading + (self.truncate(imu_out.angular_velocity.x, 4) - 0.005)
        rospy.logdebug(" + Heading Values = " + str(self.heading))

    def update_pos(self, data):
        pass

    def track_heading_and_speed(self, des_heading, curr_heading, des_speed, curr_speed):
        """
        based on https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/AMOD_2020/20201019-05%20-%20ETHZ%20-%20Control%20in%20Duckietown%20(PID).pdf
        des_heading = theta_r (theta_reference in the robot's frame of reference)
        des_speed = V_r (velocity_reference in robot's frame of reference)
        """
        #CONSTANTS
        dt = 0.01 # Should I calculate this value? self.rospy.Time

        self.heading_pid.set(des_heading)
        omega = self.heading_pid.update(curr_heading)

        left_goal  = des_speed + ((omega * self.length) / self.wheel_radius)
        right_goal = des_speed - ((omega * self.length) / self.wheel_radius)

        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        current_time = rospy.get_time()

        self.left_pid.set(left_goal)
        self.right_pid.set(right_goal)
        wheelsCmd.vel_left = self.left_pid.update(left_goal, dt)
        wheelsCmd.vel_right = self.right_pid.update(right_goal, dt)
        self.wheel_pub.publish(wheelsCmd)
        
    def set_point_to_track(self, x_ref, y_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        
    def track_point(self):
        # TODO add a conditional that ssays when Goal is reached, call a different method
        pass


    def set_wheel_speeds(self, left_speed:float, right_speed:float):
        
        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        current_time = rospy.get_time()

        wheelsCmd.vel_left = self.left_pid.update(left_speed,0.1)
        wheelsCmd.vel_right = self.right_pid.update(right_speed,0.1)
        self.wheel_pub.publish(wheelsCmd)


    def zero_command(self):
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
