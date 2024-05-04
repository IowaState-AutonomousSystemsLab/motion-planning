from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

class kalman():

    def __init__(self, R):
        self.Radius = R # Radius of the wheel in meters
    #VARIABLES THAT NEED INITIALIZED
        self.dt = 1 # time step, in seconds, refresh rate of 60 Hz
        self.var = 2 # variance for the process error of calculating P
        self.L = .01875 # Distance between the two wheels
        self.x0 = np.array([[0.], # x position
                            [0.], # y position
                            [0.], # theta (in degrees)
                            [0.], # velocity magnitude
                            [0.]])# angular velocity
        self.p0 = np.array([[1., 0., 0., 0., 0.], # variance squared for x position
                            [0., 1., 0., 0., 0.], # variance squared for y position
                            [0., 0., 1., 0., 0.], # variance squared for heading
                            [0., 0., 0., 1., 0.], # variance squared for velocity
                            [0., 0., 0., 0., 1.]])# variance squared for angular velocity
        self.wheel0 = np.array([[0.], # right wheel velocity
                                [0.]]) # left wheel velocity

    def create_matrices(self, x_current, wheel, P_current):
        
        self.x, self.y, self.theta, self.v, self.w = x_current
        self.vr, self.vl = wheel
        self.P_current = P_current

        # Create KalmanFilter object
        self.odoPose = KalmanFilter(dim_x = 5, dim_z = 1) 

        # Initialize matrixes x and P
            # row vector of the initial state, if using later use transpose
        self.odoPose.x = x_current  # np.array([[0],  # x position
                                    #           [0],  # y position
                                    #           [0],  # theta (in degrees)
                                    #           [0],  # velocity magnitude
                                    #           [0]]) # angular velocity
            # 5x5 matrix that details the initial error covariance
        self.odoPose.P = P_current  # np.array([[1., 0., 0., 0., 0.],  # variance squared for x position
                                    #           [0., 1., 0., 0., 0.],  # variance squared for y position
                                    #           [0., 0., 1., 0., 0.],  # variance squared for heading
                                    #           [0., 0., 0., 1., 0.],  # variance squared for velocity
                                    #           [0., 0., 0., 0., 1.]]) # variance squared for angular velocity
            # Error Matrix for calculating P
        self.odoPose.Q = Q_discrete_white_noise(dim = 5, dt = self.dt, var = self.var)

        # Create the State Prediction matrixs
            # State Transition Matrix F, or A
        self.odoPose.F = np.array([1., 0., 0., self.dt*np.cos(np.deg2rad(self.theta)), 0.],
                                  [0., 1., 0., self.dt*np.sin(np.deg2rad(self.theta)), 0.],
                                  [0., 0., 1., 0., self.dt],
                                  [0., 0., 0., ((self.Radius/2)*(self.vr + self.vl)), 0.],
                                  [0., 0., 0., 0., 1.])
            # Create the Control Transition matrix
        self.odoPose.B = np.array([[0., 0.],
                                   [0., 0.],
                                   [0., 0.],
                                   [(self.Radius/2), (self.Radius/2)],
                                   [(self.Radius/self.L), (-self.Radius/self.L)]])
            # Control Vector
        self.u = np.array([[self.vr],
                           [self.vl]])
        
        # Creation of the Measurement Update
            # Create the Measurement function
        self.odoPose.H = np.array([[1., 0.],
                                   [1., 0.],      
                                   [1., 0.],
                                   [1., 0.],
                                   [1., 1.]])
            # Measurement noise
        # TODO: Check what this is
        self.R = np.array([[1., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0.],
                           [0., 0., 1., 0., 0.],     # measured value through testing
                           [0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 1.]])

        self.odoPose.noise = 5
    # TODO: Fix the predict and update function to include control inputs
    def predict_and_update(self, z, x_state):

        self.z = z # makes the measurements obtained usable
        self.x_state = x_state # makes state obtained usable
        
        # Prediction step using matrices created and u, B matrices
        self.x_pred, self.P_pred = kalman.predict(x_state, self.odoPose.P, self.odoPose.F, self.odoPose.Q, u=self.u, B=self.B)
        
        # Update new state and return 
        self.new_x, self.new_P = kalman.update(self.x_pred, self.P_pred, self.z, self.R, self.odoPose.H)

        return self.new_x, self.new_P
    # For the control vector u, its used as an argument for the predict