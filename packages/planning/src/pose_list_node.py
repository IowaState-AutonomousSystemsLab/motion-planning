#!/usr/bin/env python3

import os
import rospy
import numpy as np
import math
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from as_msgs.msg import Pose


class PoseListNode(DTROS):
    def __init__(self, node_name) -> None:
        super(PoseListNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pose_list = []
        self.pub = rospy.Publisher('pose_list', String, queue_size=10)

    def add_pose(self, pose: Pose):
        self.pose_list.append(pose)

    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            message = f"Nodes: {self.pose_list}"
            # rospy.loginfo("pose list: '%s'" % message)
            self.pub.publish(message)


if __name__ == "__main__":
    node = PoseListNode(node_name="pose_list")
    node.run()
    rospy.spin()
