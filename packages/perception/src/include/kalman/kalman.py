from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

class kalman():

    def __init__(self, dt, var, R, L, x0, p0):
        self.dt = dt # time step, in seconds, refresh rate of 60 Hz
        self.var = var # variance for the process error of calculating P
        self.R = R # Radius of the wheel in meters
        self.L = L # Distance between the two wheels
        self.x0 = x0
        self.p0 = p0

    def create_matrices(self, x, y, theta, vr, vl, x_current, P_current):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = ((self.R/2)*(vr+vl))
        self.x_current = x_current
        self.P_current = P_current

        # Create KalmanFilter object
        self.odoPose = KalmanFilter(dim_x = 5, dim_z = 1) 

        # Initialize matrixes x and P
            # row vector of the initial state, if using later use transpose
        self.odoPose.x = x_current #np.array([[0], # x position
                              #[0], # y position
                              #[0], # theta (in degrees)
                              #[0], # velocity magnitude
                              #[0]])# angular velocity
            # 5x5 matrix that details the initial error covariance
        self.odoPose.P = P_current #np.array([[1., 0., 0., 0., 0.], # variance squared for x position
                                   #[0., 1., 0., 0., 0.], # variance squared for y position
                                   #[0., 0., 1., 0., 0.], # variance squared for heading
                                   #[0., 0., 0., 1., 0.], # variance squared for velocity
                                   #[0., 0., 0., 0., 1.]])# variance squared for angular velocity
            # Error Matrix for calculating P
        self.odoPose.Q = Q_discrete_white_noise(dim = 5, dt = self.dt, var = self.var)

        # Create the State Prediction matrixs
            # State Transition Matrix F, or A
        self.odoPose.F = np.array([1., 0., 0., self.dt*np.cos(np.deg2rad(self.theta)), 0.],
                                  [0., 1., 0., self.dt*np.sin(np.deg2rad(self.theta)), 0.],
                                  [0., 0., 1., 0., 0.],
                                  [0., 0., 0., ((self.R/2)*(self.vr + self.vl)), 0.],
                                  [0., 0., 0., 0., 1.])
            # Create the Control Transition matrix
        self.odoPose.B = np.array([[0., 0.],
                                   [0., 0.],
                                   [0., 0.],
                                   [(self.R/2), (self.R/2)],
                                   [(self.R/self.L), (-self.R/self.L)]])
            # Control Vector
        self.u = np.array([[self.vr],
                           [self.vl]])
            # Error Matrix W
        

        # Creation of the Measurement Update
            # Create the Measurement function
        self.odoPose.H = np.array([[1., 0.],
                                   [1., 0.],      
                                   [1., 0.],
                                   [1., 0.],
                                   [1., 1.]])

            # Measurement noise
        self.y = np.array([[1., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0.],
                           [0., 0., 1., 0., 0.],     # measured value through testing
                           [0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 1.]])

        self.odoPose.R = 5
    def predict_and_update(self, z, x_state):

        self.z = z # makes the measurements obtained usable
        self.x_state = x_state # makes state obtained usable
        
        # Prediction step using matrices created and u, B matrices
        self.x_pred, self.P_pred = kalman.predict(x_state, self.odoPose.P, self.odoPose.F, self.odoPose.Q, u=self.u, B=self.B)
        
        # Update new state and return 
        new_x, self.new_P = kalman.update(self.x_pred, self.P_pred, self.z, self.R, self.odoPose.H)

        return new_x
    # For the control vector u, its used as an argument for the predict