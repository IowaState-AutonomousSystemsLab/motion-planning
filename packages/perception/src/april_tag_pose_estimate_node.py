#!/usr/bin/env python3

from datetime import datetime
import rospy
import numpy as np
from dt_apriltags import Detector
from math import sqrt
import cv2 as cv2
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Header, Float64
from sensor_msgs.msg import CompressedImage, CameraInfo
from as_msgs.msg import Pose


class ATPoseEstimateNode(DTROS):
    def __init__(self, node_name) -> None:
        super(ATPoseEstimateNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.message = None
        self.image = None
        self.img_sub = rospy.Subscriber(
            "~/camera_node/image/compressed",
            CompressedImage,
            self.cb_image
        )
        self.img_info_sub = rospy.Subscriber(
            "~camera_info_in",
            CameraInfo,
            self.cb_camera_info
        )

        self.distance_to_tag = -1

        self.pub_distance_to_tag = rospy.Publisher(
            "~distance_to_tag",
            Float64,
            dt_topic_type=TopicType.PERCEPTION,
            dt_help="The pose estimate calculated from april tags"
        )

    def do_pose_estimation(self, cv_image):
        at_detector = Detector(searchpath=['apriltags'],
                               families='tag36h11',
                               nthreads=1,
                               quad_decimate=1.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                               decode_sharpening=0.25,
                               debug=0)

        camera_params = self.camera_info

        bw = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = at_detector.detect(bw, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)
        for detection in detections:
            mtx = np.matmul(detection.pose_R, detection.pose_t)
            squares = [np.square(val) for val in mtx]
            self.distance_to_tag = sqrt(sum(squares))

    def cb_image(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data.data, "bgr8")
        self.image = cv_image
        if not cv2.imwrite("Captured_image_" + datetime.now().strftime("%d/%m/%Y_%H:%M:%S"), cv_image):
            self.message = "IMWRITE FAILED"
            raise Exception("imwrite failed")

        self.do_pose_estimation(cv_image)

    def cb_camera_info(self, data):
        # for more info see https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        # This tuple is the relevant camera parameters needed for april-tag localization
        self.camera_info = (data.P[0][0],  # focal center x of rectified image
                            data.P[1][1],  # focal center y of rectified image
                            data.P[0][2],  # principal point x of rectified image
                            data.P[1][2])  # principal point y of rectified image

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            message = self.distance_to_tag
            self.pub_distance_to_tag.publish(message)
            rospy.loginfo("distance to tag: " + str(message))


if __name__ == "__main__":
    node = ATPoseEstimateNode(node_name="april_tag_pose_estimate")
    node.run()
    rospy.spin()
