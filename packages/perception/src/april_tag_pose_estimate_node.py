#!/usr/bin/env python3
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


class ATPoseEstimateNode(DTROS):
    def __init__(self, node_name) -> None:
        super(ATPoseEstimateNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
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
            f"/duck3/camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=2,
        )
        self.info_sub = rospy.Subscriber(
            f"/duck3/camera_node/camera_info",
            CameraInfo,
            self.cb_camera_info,
            queue_size=2,
        )
        self.pub_distance_to_tag = rospy.Publisher(
            f"~distance_to_tag",
            Float64,
            queue_size=2,
        )

    def do_pose_estimation(self):
        r = rospy.Rate(10)
        camera_params = self.camera_info

        if camera_params is None:
            rospy.loginfo("Camera parameters is None!")
            return
        if self.image is None:
            rospy.loginfo("Image is None!")
            return

        detections = self.at_detector.detect(self.image, estimate_tag_pose=True, camera_params=camera_params,
                                             tag_size=0.065)
        for detection in detections:
            mtx = np.matmul(detection.pose_R, detection.pose_t)
            squares = [np.square(val) for val in mtx]
            self.distance_to_tag = sqrt(sum(squares))
        rospy.loginfo("distance to tag: " + str(self.distance_to_tag))
        message = self.distance_to_tag
        self.pub_distance_to_tag.publish(message)
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
            self.camera_info = params
        # for more info see https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        # This tuple is the relevant camera parameters needed for april-tag localization


if __name__ == "__main__":
    node = ATPoseEstimateNode(node_name="april_tag_pose_estimate")
    while not rospy.is_shutdown():
        node.do_pose_estimation()
    rospy.spin()
