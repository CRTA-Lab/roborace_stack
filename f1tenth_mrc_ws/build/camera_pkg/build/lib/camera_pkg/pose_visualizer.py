import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector 

# Initialize AprilTag detector
detector = Detector(families='tag36h11')

# Tag size in meters (adjust for your tag)
TAG_SIZE = 0.078

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get camera intrinsics
profile = pipeline.get_active_profile()
color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                          [0, intr.fy, intr.ppy],
                          [0, 0, 1]])
camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)

print("Camera intrinsics:", camera_params)

def draw_axes(img, camera_matrix, rvec, tvec, length=0.03):
    axis = np.float32([[length, 0, 0],
                       [0, length, 0],
                       [0, 0, length]]).reshape(-1, 3)
    origin = np.float32([[0, 0, 0]]).reshape(-1, 3)

    imgpts, _ = cv2.projectPoints(np.vstack((origin, axis)), rvec, tvec, camera_matrix, distCoeffs=None)

    origin = tuple(imgpts[0].ravel().astype(int))
    x_axis = tuple(imgpts[1].ravel().astype(int))
    y_axis = tuple(imgpts[2].ravel().astype(int))
    z_axis = tuple(imgpts[3].ravel().astype(int))

    cv2.line(img, origin, x_axis, (0, 0, 255), 2)  # X - red
    cv2.line(img, origin, y_axis, (0, 255, 0), 2)  # Y - green
    cv2.line(img, origin, z_axis, (255, 0, 0), 2)  # Z - blue

# Loop
img_id = 0
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Get color image
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)

        for tag in tags:
            corners = np.array(tag.corners, dtype=np.float32)
            center = tuple(np.mean(corners, axis=0).astype(int))
            cv2.polylines(color_image, [np.int32(corners)], True, (0, 255, 255), 2)
            #cv2.putText(color_image, f"ID: {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Estimate pose
            obj_pts = np.array([
                [-TAG_SIZE/2, -TAG_SIZE/2, 0],
                [ TAG_SIZE/2, -TAG_SIZE/2, 0],
                [ TAG_SIZE/2,  TAG_SIZE/2, 0],
                [-TAG_SIZE/2,  TAG_SIZE/2, 0]
            ], dtype=np.float32)
            img_pts = corners.reshape(-1, 2)

            success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, None)
            if success:
                draw_axes(color_image, camera_matrix, rvec, tvec)

        cv2.imshow("RealSense AprilTag Viewer", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"capture_{img_id:03d}.png"
            cv2.imwrite(filename, color_image)
            print(f"Saved {filename}")
            img_id += 1
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
