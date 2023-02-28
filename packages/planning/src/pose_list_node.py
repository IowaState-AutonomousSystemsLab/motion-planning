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

    def add_pose(self, pose: Pose):
        self.pose_list.append(pose)


if __name__ == "__main__":
    node = PoseListNode(node_name="pose_list")
    rospy.spin()
