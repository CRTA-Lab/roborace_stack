import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==============================
# Configuration
# ==============================
CSV_FILE = "logged_data.csv"
OUTPUT_SVG = "trajectory_plots.svg"

# ==============================
# Utility functions
# ==============================
def rmse(pred, gt):
    mask = ~np.isnan(pred) & ~np.isnan(gt)
    if np.sum(mask) == 0:
        return np.nan
    return np.sqrt(np.mean((pred[mask] - gt[mask]) ** 2))


def polar_to_cartesian(dist, angle):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return x, y


def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion (x, y, z, w) to yaw angle [rad]
    """
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )

# ==============================
# Load data
# ==============================
df = pd.read_csv(CSV_FILE)

t = df["timestamp"].values

# ==============================
# Ground truth
# ==============================
gt_x = df["gt_x"].values
gt_y = df["gt_y"].values
gt_yaw = df["gt_yaw"].values

# ==============================
# UKF (position + quaternion)
# ==============================
ukf_x = df["UKF_px"].values
ukf_y = df["UKF_py"].values

ukf_qx = df["UKF_qx"].values
ukf_qy = df["UKF_qy"].values
ukf_qz = df["UKF_qz"].values
ukf_qw = df["UKF_qw"].values

ukf_yaw = quaternion_to_yaw(ukf_qx, ukf_qy, ukf_qz, ukf_qw)

# ==============================
# Camera measurements (flag == 2)
# ==============================
cam_mask = df["flag"] == 2

cam_time = df.loc[cam_mask, "timestamp"].values
det_angle = df.loc[cam_mask, "theta"].values

yolo_dist = df.loc[cam_mask, "rgb"].values
depth_dist = df.loc[cam_mask, "depth"].values

yolo_x, yolo_y = polar_to_cartesian(yolo_dist, det_angle)
depth_x, depth_y = polar_to_cartesian(depth_dist, det_angle)

yolo_yaw = df.loc[cam_mask, "yaw"].values

gt_x_cam = df.loc[cam_mask, "gt_x"].values
gt_y_cam = df.loc[cam_mask, "gt_y"].values
gt_yaw_cam = df.loc[cam_mask, "gt_yaw"].values

# ==============================
# LiDAR measurements (flag == 1)
# ==============================
lidar_mask = df["flag"] == 1

lidar_time = df.loc[lidar_mask, "timestamp"].values
lidar_x = df.loc[lidar_mask, "lidar_x"].values
lidar_y = df.loc[lidar_mask, "lidar_y"].values

gt_x_lidar = df.loc[lidar_mask, "gt_x"].values
gt_y_lidar = df.loc[lidar_mask, "gt_y"].values
gt_yaw_lidar = df.loc[lidar_mask, "gt_yaw"].values

# ==============================
# RMSE calculations
# ==============================
rmse_results = {
    "UKF": {
        "x": rmse(ukf_x, gt_x),
        "y": rmse(ukf_y, gt_y),
        "yaw": rmse(ukf_yaw, gt_yaw),
    },
    "Camera YOLO": {
        "x": rmse(yolo_x, gt_x_cam),
        "y": rmse(yolo_y, gt_y_cam),
        "yaw": rmse(yolo_yaw, gt_yaw_cam),
    },
    "Camera Depth": {
        "x": rmse(depth_x, gt_x_cam),
        "y": rmse(depth_y, gt_y_cam),
        "yaw": rmse(yolo_yaw, gt_yaw_cam),
    },
    "LiDAR": {
        "x": rmse(lidar_x, gt_x_lidar),
        "y": rmse(lidar_y, gt_y_lidar),
        "yaw": rmse(np.nan, gt_yaw_lidar),
    },
}

print("RMSE Results")
print("------------")
for method, vals in rmse_results.items():
    print(f"{method}:")
    for k, v in vals.items():
        print(f"  {k}: {v:.4f}")
    print()

# ==============================
# Plotting
# ==============================
fig, axs = plt.subplots(2, 2, figsize=(14, 10))

# X vs Time
axs[0, 0].plot(t, gt_x, label="GT")
axs[0, 0].plot(t, ukf_x, label="UKF")
axs[0, 0].scatter(cam_time, yolo_x, s=10, label="Cam YOLO")
axs[0, 0].scatter(cam_time, depth_x, s=10, label="Cam Depth")
axs[0, 0].scatter(lidar_time, lidar_x, s=10, label="LiDAR")
axs[0, 0].set_title("X vs Time")
axs[0, 0].set_xlabel("Time")
axs[0, 0].set_ylabel("X")
axs[0, 0].legend()

# Y vs Time
axs[0, 1].plot(t, gt_y, label="GT")
axs[0, 1].plot(t, ukf_y, label="UKF")
axs[0, 1].scatter(cam_time, yolo_y, s=10, label="Cam YOLO")
axs[0, 1].scatter(cam_time, depth_y, s=10, label="Cam Depth")
axs[0, 1].scatter(lidar_time, lidar_y, s=10, label="LiDAR")
axs[0, 1].set_title("Y vs Time")
axs[0, 1].set_xlabel("Time")
axs[0, 1].set_ylabel("Y")
axs[0, 1].legend()

# Yaw vs Time
axs[1, 0].plot(t, gt_yaw, label="GT")
axs[1, 0].plot(t, ukf_yaw, label="UKF")
axs[1, 0].scatter(cam_time, yolo_yaw, s=10, label="Cam YOLO")
axs[1, 0].set_title("Yaw vs Time")
axs[1, 0].set_xlabel("Time")
axs[1, 0].set_ylabel("Yaw [rad]")
axs[1, 0].legend()

# X-Y Plane
axs[1, 1].plot(gt_x, gt_y, label="GT")
axs[1, 1].plot(ukf_x, ukf_y, label="UKF")
axs[1, 1].scatter(yolo_x, yolo_y, s=10, label="Cam YOLO")
axs[1, 1].scatter(depth_x, depth_y, s=10, label="Cam Depth")
axs[1, 1].scatter(lidar_x, lidar_y, s=10, label="LiDAR")
axs[1, 1].set_title("X-Y Plane")
axs[1, 1].set_xlabel("X")
axs[1, 1].set_ylabel("Y")
axs[1, 1].axis("equal")
axs[1, 1].legend()

plt.tight_layout()
plt.savefig(OUTPUT_SVG, format="svg")
plt.close()

print(f"Plots saved to {OUTPUT_SVG}")
