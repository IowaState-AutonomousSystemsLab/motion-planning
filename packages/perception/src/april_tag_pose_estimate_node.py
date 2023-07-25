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
        # Some hard coded values for a csv
        # Tag ID,x_cm,y_cm,theta
        # 380,281.0,110.0,180
        # 379,281.0,224.0,180
        # 378,281.0,342.0,180
        self.known_ids = [380, 379, 378]
        self.known_locations = [
            (2.900, 1.400, 180),
            (2.810, 2.240, 180),
            (2.810, 3.420, 180)
        ]

        self.current_location = (-1, -1)
        self.current_tile = (-1, -1)
        self.image = None
        self.camera_info = None
        self.distance_to_tag = -1
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
        self.pub_distance_to_tag = rospy.Publisher(
            f"~distance_to_tag",
            Float64,
            queue_size=2,
        )

        # position is x,y
        self.position = (-100, -100)

        filename = os.curdir + os.sep + "TagLocations.csv"
        rospy.logwarn(rospy.get_namespace())
        rospy.logwarn(rospy.get_name())
        rospy.logwarn(os.listdir(os.curdir))
        rospy.logwarn_throttle(1, f"tags file: {filename}")
        # self.tags = Tag.read_csv(filename)

    def do_pose_estimation(self):
        r = rospy.Rate(10)
        camera_params = self.camera_info

        if camera_params is None:
            rospy.loginfo_once("Camera parameters is None!")
            return
        if self.image is None:
            rospy.loginfo_once("Image is None!")
            return

        detections = self.at_detector.detect(self.image, estimate_tag_pose=True, camera_params=camera_params,
                                             tag_size=0.065)
        # tag_size=0.04875)
        # april tag size is reported as 6.5 cm for full square (8x8), but april tag library wants inner square (6x6)
        # inner 6x6 size: 0.04875
        # another value is 0.08125

        for detection in detections:
            id = detection.tag_id
            tag = self.lookup(id)
            throttle = 1
            # alpha = np.deg2rad(tag.degrees)
            # global_tag_rotation_pitch = np.array([
            #     [np.cos(alpha), 0, np.sin(alpha)],
            #     [0, 1, 0],
            #     [-np.sin(alpha), 0, np.cos(alpha)]
            # ])
            # r_times_t = detection.pose_R @ detection.pose_t
            #
            # global_position_vector = global_tag_rotation_pitch @ r_times_t
            # global_position_vector = global_tag_rotation_pitch @ detection.pose_t
            # global_position_vector2 = global_tag_rotation_pitch @ r_times_t
            # # global_position_vector -= tag.global_position
            #
            # position = np.squeeze(global_position_vector)
            # position2 = np.squeeze(global_position_vector2)
            # translation = np.squeeze(detection.pose_t)
            # r_t = np.squeeze(r_times_t)
            # # string = f"translation:{translation} position: {position}"
            #
            # position /= np.array([TILE_WIDTH, 1, TILE_HEIGHT])
            # position2 /= np.array([TILE_WIDTH, 1, TILE_HEIGHT])
            # string = f"position: {position} pos2: {position2}"

            # from https://bitbucket.org/kaess/apriltags/src/master/example/apriltags_demo.cpp
            f = np.array([
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, 1]
            ])
            fixed_rot = f @ detection.pose_R
            vec = fixed_rot @ detection.pose_t

            string = np.squeeze(vec)
            # rospy.loginfo(string)

            # from some c++ code i found
            wRo = detection.pose_R
            yaw = np.radians(np.arctan2(wRo[1][0], wRo[0][0]))
            c = np.cos(yaw)
            s = np.sin(yaw)
            pitch = np.arctan2(-wRo[2][0], wRo[0][0] * c + wRo[1][0] * s)
            roll = np.arctan2(wRo[0][2] * s - wRo[1][2] * c, -wRo[0][1] * s + wRo[1][1] * c)
            # rospy.loginfo_throttle(throttle, f"rotations: y: {yaw} p: {pitch} r: {roll}")

            global_robot_angle = np.radians(tag.degrees) + pitch + np.pi / 2
            # global_robot_angle = pitch

            rotation_pitch = np.array([
                [np.cos(global_robot_angle), 0, np.sin(global_robot_angle)],
                [0, 1, 0],
                [-np.sin(global_robot_angle), 0, np.cos(global_robot_angle)]
            ])

            rotation_yaw = np.array([
                [np.cos(global_robot_angle), -np.sin(global_robot_angle), 0],
                [np.sin(global_robot_angle), np.cos(global_robot_angle), 0],
                [0, 0, 1]
            ])

            global_position = tag.world_tile * np.array([[TILE_WIDTH], [1], [TILE_HEIGHT]])

            robot_position = (np.linalg.inv(rotation_pitch) @ detection.pose_t) + global_position
            position_in_tiles = robot_position / np.array([[TILE_WIDTH], [1], [TILE_HEIGHT]])
            np.linalg.norm(position_in_tiles)
            # position_in_tiles = robot_position / [TILE_WIDTH, 1, TILE_HEIGHT]

            rospy.loginfo_throttle(throttle,
                                   f"pitch: {np.degrees(pitch):.4f} location: {np.squeeze(position_in_tiles)}")

        r.sleep()

    def cb_image(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    def cb_camera_info(self, msg):
        if self.camera_info is None:
            rospy.loginfo("camera info is none, setting now")
            p_arr = np.array(msg.P).reshape(3, 4)

            p0 = p_arr[0][0]
            p1 = p_arr[1][1]
            p2 = p_arr[0][2]
            p3 = p_arr[1][2]
            params = (p0,  # focal center x of rectified image
                      p1,  # focal center y of rectified image
                      p2,  # principal point x of rectified image
                      p3)  # principal point y of rectified image

            # from duckiebot web interface:
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
            rospy.logwarn(params)
        # for more info see https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        # This tuple is the relevant camera parameters needed for april-tag localization

    def lookup(self, id):
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
