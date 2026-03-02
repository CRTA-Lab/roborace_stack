import numpy as np

class DepthFusionExtendedKalmanFilter:
    def __init__(self, initial_distance=1.0, initial_velocity=1.0, polynomial_coefficients=None):
        self.poly_coeffs = polynomial_coefficients

        #Initial state
        self.x = np.array([[initial_distance],
                           [initial_velocity]],dtype=np.float64)

        #Initial state covariance
        self.P = np.array([[10.0, 0.0],
                           [0.0, 1.0]], dtype=np.float64)

        # State transition matrix (constant velocity model)
        self.dt = 0.01  # time step in seconds
        
        # Process noise covariance matrix (adjust for trust in model)
        self.Q = np.array([[0.6, 0.0],
                           [0.0, 0.2]], dtype=np.float64)

        # Measurement matrix (depth camera measures distance only)
        self.H = np.array([[1.0, 0.0]], dtype=np.float64)
       
        # Measurement noise covariance matrix (tune based on sensor accuracy)
        self.R = np.array([[0.095]], dtype=np.float64)  # Lower means high trust in depth camera

    def keypoint_prediction_state(self, dt, poly_coefficients, body_keypoints_distance):
        #Nonlienar state transition
        state_dist_pred= (poly_coefficients[0] * body_keypoints_distance ** 2 + poly_coefficients[1] * body_keypoints_distance + poly_coefficients[2])

        distance_pred = state_dist_pred + self.x[1,0]*dt
        velocity_pred = self.x[1,0]

        state_pred = np.array([[distance_pred],[(velocity_pred)]], dtype=np.float64)
        return state_pred

    def compute_F_jacobian(self, x, dt, body_keypoints_distance):
        # For simple constant velocity model with nonlinear distance:
        return np.array([[0.0, dt],
                        [0.0, 1.0]])

    def predict(self,dt, poly_coefficients, body_keypoints_distance):
        #State prediction and covariance matrix
        self.x = self.keypoint_prediction_state(dt, poly_coefficients, body_keypoints_distance)
        F = self.compute_F_jacobian(self.x, dt, body_keypoints_distance)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        # Measurement residual
        y = np.array([[measurement]]) - self.H @ self.x
       
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
       
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
       
        # State update
        self.x = self.x + K @ y
       
        # Covariance update
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T

    def step(self, dt, body_keypoints_distance, depth_measurement):
        
        poly_coefficients= [0.00978, -5.472, 949.9]

        # Process noise covariance matrix (adjust for trust in model)
        # self.Q = np.array([[60.0*self.dt, 0.0],
        #                     [0.0, 20.0*self.dt]], dtype=np.float64)
        
        self.predict(dt,poly_coefficients,body_keypoints_distance)
        
        self.update(depth_measurement)
        
        return self.x.flatten()
