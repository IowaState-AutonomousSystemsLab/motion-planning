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
    def odoPose_cb(self, poseMsg:Pose):
        """ Callback functino for when an OdoPose message is published

        Args:
                poseMsg (Pose): automatically supplied OdoPose message
        
        """
        # Unpack odoPose message into x, y, z
        self.x_measure = poseMsg.x
        self.y_measure = poseMsg.y
        self.theta_measure = poseMsg.heading
        
    def imu_cb(self, imuMessage:Imu):
        """ Callback functino when an Imu message is published
        
        Args:
                Imu (Imu): Automatically supplied Imu message
        """
        #angular velocity about the z axis
        self.imu_w = imuMessage.angular_velocity.z

    def filter_cb(self, wheelMsg):
        """Callback funtion when wheelMsg is published
        This function creates the measurement update z for the kalman filter. It 
        also houses the prediction step of the filter, and then publishes a message to
        the filtered_position topic


        Args:
                wheelMsg (wheel): Automatically supplied when message is published
        """
        # Unpacks the wheel velocity messages
        self.vr_measure = wheelMsg.right_wheel_velocity
        self.vl_measure = wheelMsg.left_wheel_velocity

        # Calculates the velocity
        self.v_measure = ((self.R/2)*(self.vr_measure+self.vl_measure))
        
        # Creates a vector of the wheel velocities to be used in matrix calculation
        self.wheel = ([[self.vr_measure],
                       [self.vl_measure]])
        
        # Create measurement matrix Z
        z = np.array([[self.x_measure],
                      [self.y_measure],
                      [self.theta_measure],
                      [self.v_measure],
                      [self.imu_w]])

        # Run predict and update to obtain state estimate
        self.previous_x, self.previous_P = self.kalman.predict_and_update(self, z, self.kalman.odoPose.x)
        
        # Unpack state into state variables
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