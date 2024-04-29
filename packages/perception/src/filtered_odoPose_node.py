import os
import rospy
import numpy as np
from typing import Tuple
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from as_msgs.msg import WheelOdometry, Pose
from sensor_msgs.msg import Imu
from include.kinematics.differential_drive_kinematics import DifferentialDriveKinematics
from include.kalman.py import kalman

class filtered_odoPose_node(DTROS):
    def __init__(self, node_name) -> None:
        super(filtered_odoPose_node, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.previous_state = np.zeros(5,1)
        self.P_variable = np.eye(5,5) # used to store P values between creation and update

        self.R = rospy.get_param(f"/{os.environ['VEHICLE_NAME']}/kinematics_node/radius", 100) # Wheel radius using parameter request from pose_estimate_node
        self.L = .1085 # length between wheel centers, in meters
        x0 = np.array([[0.], # x position
                       [0.], # y position
                       [0.], # theta (in degrees)
                       [0.], # velocity magnitude
                       [0.]])# angular velocity
        p0 = np.array([[1., 0., 0., 0., 0.], # variance squared for x position
                       [0., 1., 0., 0., 0.], # variance squared for y position
                       [0., 0., 1., 0., 0.], # variance squared for heading
                       [0., 0., 0., 1., 0.], # variance squared for velocity
                       [0., 0., 0., 0., 1.]])# variance squared for angular velocity
        
        self.kalman = kalman(self, dt = 1, var = 5, R = self.R, L = self.L, x0 = x0, p0 = p0) # values that initial the filter

        # Function that creates kalman matrices based on initialized values
        kalman.create_matrices(self, x0[0], x0[1], x0[2], x0[3], x0[4], self.x0)

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
# Initializer
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

        z = np.array([[self.x_measure],
                      [self.y_measure],
                      [self.theta_measure],
                      [self.v_measure],
                      [self.imu_w]])
        x_state = self.previous_state
        P = self.P_variable
        
        '''
        The previous_state variable is an attempt to store the previous state of the agent
        in a temporary varaible that allows us to feed it back into the filter. Because all
        of the matrices must be recalculated and run in callback functions, the temprorary 
        variable stores the previous state value outside of that process, allowing us to access
        it even as the kalman filter is constantly iterating
        '''

        self.previous_state = self.kalman.predict_and_update(self, z, x_state)
        self.x, self.y, self.theta, self.v, self.w = self.previous_state
        
        #Utilize previous state code to get matrix X
        kalman.create_matrices(self, x_state[0], x_state[1], x_state[2], x_state[3], x_state[4], self.x_state, self.P_variable)

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