#!/usr/bin/env python3
import os
import time
from datetime import datetime
import rospy
import numpy as np
from dt_apriltags import Detector
from math import sqrt
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Header, Float64
from sensor_msgs.msg import CompressedImage, CameraInfo
from as_msgs.msg import Pose
import Tag

TILE_WIDTH = 0.59
TILE_HEIGHT = 0.59

float_formatter = "{:.4f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})


class ATPoseEstimateNode(DTROS):
    def __init__(self, node_name) -> None:
        super(ATPoseEstimateNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.image = None
        self.camera_info = None
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=2,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.img_sub = rospy.Subscriber(
            f"~camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=2,
        )
        self.info_sub = rospy.Subscriber(
            f"~camera_node/camera_info",
            CameraInfo,
            self.cb_camera_info,
            queue_size=2,
        )

    def do_pose_estimation(self):
        camera_params = self.camera_info

        if camera_params is None:
            rospy.loginfo_once("Camera parameters is None; waiting to set")
            return
        if self.image is None:
            rospy.loginfo_once("Image is None; waiting to set")
            return

        detections = self.at_detector.detect(self.image, estimate_tag_pose=True, camera_params=camera_params,
                                             tag_size=0.065)
        # april tag size is reported as 6.5 cm for full square (8x8), but april tag library wants inner square (6x6),
        # that being said, it seems, from experimenting, that 0.065 is the correct value for the size
        # inner 6x6 size: 0.04875
        # another value is 0.08125

        for detection in detections:
            id = detection.tag_id
            tag = self.lookup(id)
            throttle = 1

            # from some c++ code i found here:
            # https://bitbucket.org/kaess/apriltags/src/3aea96d3239bb83870cb5e0a85f142a2b0589563/example/apriltags_demo.cpp#lines-118
            wRo = detection.pose_R
            yaw = np.radians(np.arctan2(wRo[1][0], wRo[0][0]))
            c = np.cos(yaw)
            s = np.sin(yaw)
            pitch = np.arctan2(-wRo[2][0], wRo[0][0] * c + wRo[1][0] * s)
            roll = np.arctan2(wRo[0][2] * s - wRo[1][2] * c, -wRo[0][1] * s + wRo[1][1] * c)
            # rospy.loginfo_throttle(throttle, f"rotations: y: {yaw} p: {pitch} r: {roll}")

            global_robot_angle = np.radians(tag.degrees) + pitch + np.pi / 2
            # global_robot_angle = pitch

            rotation_matrix_pitch = np.array([
                [np.cos(global_robot_angle), 0, np.sin(global_robot_angle)],
                [0, 1, 0],
                [-np.sin(global_robot_angle), 0, np.cos(global_robot_angle)]
            ])

            rotation_matrix_yaw = np.array([
                [np.cos(global_robot_angle), -np.sin(global_robot_angle), 0],
                [np.sin(global_robot_angle), np.cos(global_robot_angle), 0],
                [0, 0, 1]
            ])

            scale_array = np.array([[TILE_WIDTH], [1], [TILE_HEIGHT]])
            # position in meters relative to the map
            # TODO make this a function call of the tag class

            global_position = tag.world_tile * scale_array

            robot_position = (np.linalg.inv(rotation_matrix_pitch) @ detection.pose_t) + global_position
            position_in_tiles = robot_position / scale_array

            rospy.loginfo_throttle(throttle,
                                   f"pitch: {np.degrees(pitch):.4f} location: {np.squeeze(position_in_tiles)}")

    def cb_image(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    def cb_camera_info(self, msg):
        if self.camera_info is None:
            rospy.loginfo("camera info is none, setting now")
            # subscribing to the ros node seems to only give default values, so the commented out portion would work if
            # the camera parameters in the node were correct

            # p_arr = np.array(msg.P).reshape(3, 4)
            # p0 = p_arr[0][0]
            # p1 = p_arr[1][1]
            # p2 = p_arr[0][2]
            # p3 = p_arr[1][2]
            # params = (p0,  # focal center x of rectified image
            #           p1,  # focal center y of rectified image
            #           p2,  # principal point x of rectified image
            #           p3)  # principal point y of rectified image
            # self.camera_info = params

            # from duckiebot web interface:
            # TODO see if there is a way to get this from a ros node,
            # though it should, in theory, be the same for all the duckiebots
            duckiebot_camera_matrix = np.array(
                [[327.7921447753906, 0.0, 334.8615555610704, 0.0],
                 [0.0, 353.49005126953125, 162.79261982972093, 0.0],
                 [0.0, 0.0, 1.0, 0.0]])

            params = (
                duckiebot_camera_matrix[0][0],
                duckiebot_camera_matrix[1][1],
                duckiebot_camera_matrix[0][2],
                duckiebot_camera_matrix[1][2]
            )

            self.camera_info = params
            rospy.loginfo(f"camera params: {params}")
        # This tuple is the relevant camera parameters needed for april-tag localization
        # for more info see https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html

    def lookup(self, id):
        """
        lookup a tag based on the id and return the info for the tag
        :param id: id of the tag that is being searched for
        :return: the Tag class that has the necessary info to get the robot location
        """
        degrees = 45
        pos = np.array([
            [2 * TILE_WIDTH],  # x in the map view
            [0],  # elevation in the map view - unused
            [2 * TILE_HEIGHT]  # y in the map view
        ])
        tile_pos = np.array([
            [2],  # x in the map view
            [0],  # elevation in the map view - unused
            [2]  # y in the map view
        ])
        tag = Tag.Tag(380, Tag.Position.NorthEast, degrees, tile_pos)

        # found_tag = self.tags.find(lambda x: x.id == id)
        # rospy.logwarn_throttle(1, f"found tag id: {found_tag.id}")
        return tag


if __name__ == "__main__":
    node = ATPoseEstimateNode(node_name="april_tag_pose_estimate")
    while not rospy.is_shutdown():
        node.do_pose_estimation()
    rospy.spin()
