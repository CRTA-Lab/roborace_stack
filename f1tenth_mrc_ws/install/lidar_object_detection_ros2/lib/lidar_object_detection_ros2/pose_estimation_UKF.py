import numpy as np

class UKF:
    def __init__(self):
        self.dim_x = 6 # state dimension
        self.dim_z_cam = 4 # measurement dimension for camera
        self.dim_z_lidar = 2 # measurement dimension for lidar


        self.Q = np.diag([
            0.001, 0.001, 0.001,    # position (x, y, yaw )
            0.005, 0.005, 0.001       # velocity (vx, vy, yaw_rate)
        ])                       # process noise


        self.R_cam = np.array([
            [0.2,   0.2,    0.0,    0.0],
            [0.2,    0.3,   0.0,    0.0],
            [0.0,    0.0,    0.4,    0.0],
            [0.0,    0.0,    0.0,    0.2] 
            
        ])                          # measurement noise camera

        self.R_lidar = np.diag([0.2, 0.2])  # measurement noise lidar

        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0

        self.lambda_ = self.alpha**2 * (self.dim_x + self.kappa) - self.dim_x
        self.gamma = np.sqrt(self.dim_x + self.lambda_)

        # Weights
        self.Wm = np.zeros(2 * self.dim_x + 1)
        self.Wc = np.zeros(2 * self.dim_x + 1)

        self.Wm[0] = self.lambda_ / (self.dim_x + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)

        for i in range(1, 2 * self.dim_x + 1):
            self.Wm[i] = 1 / (2 * (self.dim_x + self.lambda_))
            self.Wc[i] = self.Wm[i]

        # State
        self.x = np.zeros(self.dim_x)
        self.P = np.eye(self.dim_x)
        
        self.t_CB = [0, 0]
        self.R_CB = np.eye(2)
        #self.t_LB = [-0.26, 0]
        self.t_LB = [0.0, 0]
        self.R_LB = np.eye(2)

    def _sigma_points(self, x, P):
        sigma = np.zeros((2 * self.dim_x + 1, self.dim_x))
        sigma[0] = x

        sqrt_P = np.linalg.cholesky(P)

        for i in range(self.dim_x):
            sigma[i + 1] = x + self.gamma * sqrt_P[:, i]
            sigma[self.dim_x + i + 1] = x - self.gamma * sqrt_P[:, i]

        return sigma

    def predict(self, dt):
        sigma = self._sigma_points(self.x, self.P)

        # Propagate through process model
        sigma_pred = np.array([self.fx(s, dt) for s in sigma])

        # Predicted mean
        self.x = np.sum(self.Wm[:, None] * sigma_pred, axis=0)

        # Predicted covariance
        self.P = self.Q.copy()
        for i in range(len(sigma_pred)):
            dx = sigma_pred[i] - self.x
            self.P += self.Wc[i] * np.outer(dx, dx)

    def update_camera(self, z):
        sigma = self._sigma_points(self.x, self.P)

        # Transform into measurement space
        Z = np.array([self.hx_camera(s) for s in sigma])

        # Predicted measurement
        z_pred = np.sum(self.Wm[:, None] * Z, axis=0)

        # Innovation covariance
        S = self.R_cam.copy()
        for i in range(len(Z)):
            dz = Z[i] - z_pred
            S += self.Wc[i] * np.outer(dz, dz)

        # Cross covariance
        Pxz = np.zeros((self.dim_x, self.dim_z_cam))
        for i in range(len(sigma)):
            dx = sigma[i] - self.x
            dz = Z[i] - z_pred
            Pxz += self.Wc[i] * np.outer(dx, dz)

        # Kalman gain
        K = Pxz @ np.linalg.inv(S)

        # Update
        self.x += K @ (z - z_pred)
        self.P -= K @ S @ K.T

    def update_lidar(self, z):
        sigma = self._sigma_points(self.x, self.P)

        # Transform into measurement space
        Z = np.array([self.hx_lidar(s) for s in sigma])

        # Predicted measurement
        z_pred = np.sum(self.Wm[:, None] * Z, axis=0)

        # Innovation covariance
        S = self.R_lidar.copy()
        for i in range(len(Z)):
            dz = Z[i] - z_pred
            S += self.Wc[i] * np.outer(dz, dz)

        # Cross covariance
        Pxz = np.zeros((self.dim_x, self.dim_z_lidar))
        for i in range(len(sigma)):
            dx = sigma[i] - self.x
            dz = Z[i] - z_pred
            Pxz += self.Wc[i] * np.outer(dx, dz)

        # Kalman gain
        K = Pxz @ np.linalg.inv(S)

        # Update
        self.x += K @ (z - z_pred)
        self.P -= K @ S @ K.T

    def fx(self, x, dt):
        px, py, yaw, vx, vy, yaw_rate = x

        px_new = px + vx * dt
        py_new = py + vy * dt
        yaw_new = yaw + yaw_rate * dt

        return np.array([px_new, py_new, yaw_new, vx, vy, yaw_rate])
    
    def hx_camera(self, x):
        p_B = x[0:2]

        # Transform base -> camera
        p_C = self.R_CB @ p_B + self.t_CB
        px, py, yaw_yolo = p_C[0], p_C[1], x[2]

        r_yolo = np.sqrt(px**2 + py**2)
        theta = np.arctan2(py, px)

        r_depth = r_yolo

        return np.array([r_yolo, r_depth, yaw_yolo, theta])
    
    def hx_lidar(self, x):
        p_B = x[0:2]

        # Transform base -> lidar
        p_L = self.R_LB @ p_B + self.t_LB

        return p_L


