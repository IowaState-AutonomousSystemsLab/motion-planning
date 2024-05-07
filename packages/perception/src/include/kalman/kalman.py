from filterpy.kalman import KalmanFilter
from scipy.stats import norm
import numpy as np

class kalman():
    def __init__(self, R):
        self.Radius = R # Radius of the wheel in meters
    #VARIABLES THAT NEED INITIALIZED
        self.dt = 1 # time step, in seconds, refresh rate of 60 Hz
        self.var = 1 # variance for the process error of calculating P
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
        """function used to create matrices for the kalman filter
        Some of the matrices will stay the same but some will update based on
        new velocities.

        This function also creates the kalman filter instance
        Args:
                x_current : Current State matrix, x, y, heading, velocity, angular velocity
                wheel     : right and left wheel velocities in vector form
                P_current : Current Covariance matrix
        
        """
        self.x, self.y, self.theta, self.v, self.w = x_current
        self.vr, self.vl = wheel
        self.P_current = P_current

        # Create KalmanFilter object
        self.odoPose = KalmanFilter(dim_x = 5, dim_z = 5, dim_u = 2) 

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
        noise_matrix_size = (5, 5)
        self.odoPose.Q = norm.rvs(loc=10, scale=self.var, size=noise_matrix_size)

        # Create the State Prediction matrixs
            # State Transition Matrix F, or A
        self.odoPose.F = np.array([[1., 0., 0., float(self.dt*np.cos(np.deg2rad(self.theta))), 0.],
                                  [0., 1., 0., float(self.dt*np.sin(np.deg2rad(self.theta))), 0.],
                                  [0., 0., 1., 0., float(self.dt)],
                                  [0., 0., 0., float(((self.Radius/2)*(self.vr + self.vl))), 0.],
                                  [0., 0., 0., 0., 1.]])
            # Create the Control Transition matrix
        self.odoPose.B = np.array([[0., 0.],
                                   [0., 0.],
                                   [0., 0.],
                                   [(self.Radius/2), (self.Radius/2)],
                                   [(self.Radius/self.L), (-self.Radius/self.L)]])
            # Control Vector
        self.u = np.array([[float(self.vr)], 
                           [float(self.vl)]])
        
        # Creation of the Measurement Update
            # Create the Measurement function
        self.odoPose.H = np.array([[1., 0., 0., 0., 0.],
                                   [0., 1., 0., 0., 0.], 
                                   [0., 0., 1., 0., 0.],
                                   [0., 0., 0., 1., 0.], 
                                   [0., 0., 0., 0., 1.]])
            # Measurement noise
        self.R = np.array([[1., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0.],
                           [0., 0., 1., 0., 0.],     # measured value through testing
                           [0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 1.]])

    def predict_and_update(self, z, x_state):
        """function that is used to run the predict and update for
        the kalman filter.

        Args:
                z : measurement matrix
                x_state : current system state
        
        """
        self.z = z # makes the measurements obtained usable
        self.x_state = x_state # makes state obtained usable
        
        # Prediction step using matrices created and u, B matrices
        # NOTE: Might need the state and P taken out to work properly
        self.odoPose.predict(u = self.u, B = self.odoPose.B, F = self.odoPose.F, Q = self.odoPose.Q)
        
        # Update new state and return values
        # NOTE: Might need the state and P taken out to work properly
        self.odoPose.update(z = self.z, R = self.R, H = self.odoPose.H)

        return self.new_x, self.new_P