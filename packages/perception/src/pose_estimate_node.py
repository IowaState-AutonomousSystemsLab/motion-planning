#!/usr/bin/env python3

import rospy
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from as_msgs.msg import WheelOdometry, Pose
from include.kinematics.differential_drive_kinematics import DifferentialDriveKinematics

class PoseEstimateNode(DTROS):
    """To estimate the pose of the robot using the wheel odometry
    """
    def __init__(self, 
                 node_name: str) -> None:
        
        super(PoseEstimateNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.kinematics = DifferentialDriveKinematics(0.1085)

        self.heading = 0
        self.x = 0
        self.y = 0

        #sub to odometry
        self.left_tick_sub = rospy.Subscriber(
            f"~wheel_odometry/odometry", 
            WheelOdometry,
            self.odometry_cb,
        )

        self.pose_pub = rospy.Publisher(
            f"~pose",
            Pose,
            queue_size=1
        )

        self.previous_time = None

        self.log("Initalized!")

    def odometry_cb(self, odoMsg):
        """Once odometry is published, this method automatically calulates the new pose of the robot

        Args:
            odoMsg (as_msgs/msg/OdometryMsg)
        """
        self.x, self.y, self.heading = self.kinematics.calculate_new_pose(
            self.x,
            self.y,
            self.heading,
            odoMsg.left_wheel_velocity,
            odoMsg.right_wheel_velocity,
            odoMsg.delta
        )

        pose_msg = Pose()
        header = Header()
        header.stamp = rospy.Time.now()
        pose_msg.header = header
        pose_msg.ID = -1
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.heading = self.heading
        self.pose_pub.publish(pose_msg)

if __name__ ==  "__main__":
    node = PoseEstimateNode(node_name="pose_estimate_node")
    rospy.spin()