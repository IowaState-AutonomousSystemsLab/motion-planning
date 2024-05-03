import os
import rospy
import numpy as np
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from as_msgs.msg import WheelOdometry, Pose
from sensor_msgs.msg import Imu
from include.kinematics.differential_drive_kinematics import DifferentialDriveKinematics
from include.kalman.kalman import kalman

class filtered_odoPose_node(DTROS):
    def __init__(self, node_name) -> None:
        super(filtered_odoPose_node, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    # Obtaining Radius and Initializing Kalman Class    
        self.R = rospy.get_param(f"/{os.environ['VEHICLE_NAME']}/kinematics_node/radius", 100) # Wheel radius using parameter request from pose_estimate_node
        self.kalman = kalman(self, R = self.R) # values that initial the filter

    # Creates Kalman Matrices 
        kalman.create_matrices(self, self.kalman.x0, self.kalman.wheel0, self.kalman.p0)

# Subscribers and Publishers
        #create subscriber to the pose_estimate (get x, y, theta)
        self.odoPose_sub = rospy.Subscriber(
            "pose",
            Pose,
            self.odoPose_cb,
        )

        #create a subscriber to the imu node to unpack the angular velocity
        self.IMU_sub_sub = rospy.Subscriber(
            "Imu", # not sure what the specific topic name is yet
            Imu,
            self.imu_cb,
        )

        #create subscriber to the wheel odometry node (get vr, vl)
        self.wheel_encoder_sub = rospy.Subscriber(
            "wheel_odometry/odometry",
            WheelOdometry,
            self.filter_cb
        )
        
        #create publisher that publishes a new pose estimate of message type
        # as_msgs.msg Pose
        self.filtered_pose_pub = rospy.Publisher(
            "filtered_position",
            Pose,
            queue_size = 1
        )

# Initialize Message
        self.previous_time = None
        self.log("Initialized!")

#Subscriber Callbacks        
    def odoPose_cb(self, poseMsg):
        # Unpack odoPose message into x, y, z
        self.x_measure = poseMsg.x
        self.y_measure = poseMsg.y
        self.theta_measure = poseMsg.heading
        
    def imu_cb(self, Imu):
        #angular velocity about the z axis
        self.imu_w = Imu.angular_velocity.z

    def filter_cb(self, wheelMsg):
        #write code that makes matrix z
        self.vr_measure = wheelMsg.right_wheel_velocity
        self.vl_measure = wheelMsg.left_wheel_velocity
        self.v_measure = ((self.R/2)*(self.vr_measure+self.vl_measure))
        self.wheel = ([[self.vr_measure],
                       [self.vl_measure]])
        
        z = np.array([[self.x_measure],
                      [self.y_measure],
                      [self.theta_measure],
                      [self.v_measure],
                      [self.imu_w]])

        # TODO: Fix the inputs of the predict and update function
        self.previous_x, self.previous_P = self.kalman.predict_and_update(self, z, self.kalman.odoPose.x)
        self.x, self.y, self.theta, self.v, self.w = self.previous_x
        
        #Create New set of matrices to be used in next iteration
        kalman.create_matrices(self, self.previous_x, self.wheel, self.previous_P)

        # Creates Pose message to publish
        pose_msg = Pose()
        header = Header()
        header.stamp = rospy.Time.now()
        pose_msg.header = header
        pose_msg.ID = -1
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.heading = self.theta
        self.filtered_pose_pub.publish(pose_msg)

if __name__ == "__main__":
    node = filtered_odoPose_node(node_name="filtered_odoPose_node")
    rospy.spin()