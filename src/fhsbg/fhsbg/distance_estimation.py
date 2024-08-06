import cv2 as cv
from cv2 import aruco
import numpy as np
import pickle
import pyrealsense2 as rs
import math
import queue
from threading import Thread

class LowPassFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.last_value = None

    def apply(self, value):
        if self.last_value is None:
            self.last_value = value
        else:
            self.last_value = self.alpha * value + (1 - self.alpha) * self.last_value
        return self.last_value

class ArucoDetector:
    def __init__(self, marker_size, target_marker_id, fov_horizontal, fov_vertical, calibration_data, cam_mat, dist_coef, alpha=0.5, data_queue=None):
        self.marker_size = marker_size
        self.target_marker_id = target_marker_id
        self.fov_horizontal = fov_horizontal
        self.fov_vertical = fov_vertical
        self.cam_mat = cam_mat
        self.dist_coef = dist_coef
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)  # Use getPredefinedDictionary
        self.param_markers = aruco.DetectorParameters_create() if hasattr(aruco, 'DetectorParameters_create') else aruco.DetectorParameters()  # Create parameters
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.frame_center = None
        self.x_filter = LowPassFilter(alpha)
        self.y_filter = LowPassFilter(alpha)
        self.depth_filter = LowPassFilter(alpha)

        self.data_queue = data_queue  # Queue to publish data

    def start_pipeline(self):
        try:
            self.pipeline.start(self.config)
            print("Pipeline started.")
        except RuntimeError as e:
            print(f"RealSense pipeline couldn't start: {e}")
            exit(1)

    def stop_pipeline(self):
        self.pipeline.stop()
        print("Pipeline stopped.")

    def pixel_to_meters(self, x_pixel, y_pixel, image_width, image_height, distance_to_object):
        angle_per_pixel_x = self.fov_horizontal / image_width
        angle_per_pixel_y = self.fov_vertical / image_height

        angle_offset_x = x_pixel * angle_per_pixel_x
        angle_offset_y = y_pixel * angle_per_pixel_y

        angle_offset_x_radians = math.radians(angle_offset_x)
        angle_offset_y_radians = math.radians(angle_offset_y)

        x_meters = math.tan(angle_offset_x_radians) * distance_to_object
        y_meters = math.tan(angle_offset_y_radians) * distance_to_object

        return x_meters, y_meters

    def process_frames(self):
        self.start_pipeline()

        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                marker_corners, marker_IDs, reject = aruco.detectMarkers(
                    gray_frame, self.marker_dict, parameters=self.param_markers
                )

                self.frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)

                if marker_corners:
                    rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                        marker_corners, self.marker_size, self.cam_mat, self.dist_coef
                    )
                    found_marker = False
                    for ids, corners, rvec, tvec in zip(marker_IDs, marker_corners, rVec, tVec):
                        if ids[0] == self.target_marker_id:
                            found_marker = True
                            corners = corners.reshape(4, 2)
                            corners = corners.astype(int)
                            center = np.mean(corners, axis=0).astype(int)

                            aruco_distance = np.sqrt(
                                tvec[0][2] ** 2 + tvec[0][0] ** 2 + tvec[0][1] ** 2
                            ) / 100  # convert from cm to meters

                            depth_distance = depth_frame.get_distance(center[0], center[1])

                            fused_distance = 0.7 * depth_distance + 0.3 * aruco_distance

                            pixel_x = center[0] - self.frame_center[0]
                            pixel_y = center[1] - self.frame_center[1]

                            x_meters, y_meters = self.pixel_to_meters(
                                pixel_x, pixel_y, frame.shape[1], frame.shape[0], fused_distance
                            )

                            x_meters_filtered = self.x_filter.apply(x_meters)
                            y_meters_filtered = self.y_filter.apply(y_meters)
                            fused_distance_filtered = self.depth_filter.apply(fused_distance)

                            data = {
                                'x': x_meters_filtered,
                                'y': y_meters_filtered,
                                'depth': fused_distance_filtered
                            }

                            print(f"Detected Marker ID: {ids[0]}")
                            print(f"x_meters: {x_meters:.4f}, y_meters: {y_meters:.4f}, fused_distance: {fused_distance:.4f}")
                            print(f"Filtered - x_meters: {x_meters_filtered:.4f}, y_meters: {y_meters_filtered:.4f}, fused_distance: {fused_distance_filtered:.4f}")

                            if self.data_queue:
                                self.data_queue.put(data)

                            break

                    if not found_marker:
                        data = {
                            'x': 0.0,
                            'y': 0.0,
                            'depth': 0.0
                        }
                        if self.data_queue:
                            self.data_queue.put(data)
                        print("Target marker not found.")
                else:
                    data = {
                        'x': 0.0,
                        'y': 0.0,
                        'depth': 0.0
                    }
                    if self.data_queue:
                        self.data_queue.put(data)
                    print("No markers detected.")

        finally:
            self.stop_pipeline()

if __name__ == "__main__":
    data_queue = queue.Queue()

    # Load calibration data
    with open('/home/naor/Desktop/naor/study/fhsbg_final/src/fhsbg/fhsbg/calibration/calibration.pkl', 'rb') as f:
        calibration_data = pickle.load(f)

    with open('/home/naor/Desktop/naor/study/fhsbg_final/src/fhsbg/fhsbg/calibration/cameraMatrix.pkl', 'rb') as f:
        cam_mat = pickle.load(f)

    with open('/home/naor/Desktop/naor/study/fhsbg_final/src/fhsbg/fhsbg/calibration/dist.pkl', 'rb') as f:
        dist_coef = pickle.load(f)

    detector = ArucoDetector(
        marker_size=6,  # Marker size in centimeters
        target_marker_id=73,  # The target ArUco marker ID
        fov_horizontal=87,  # Field of view in degrees for RealSense D435
        fov_vertical=58,  # Field of view in degrees for RealSense D435
        calibration_data=calibration_data,
        cam_mat=cam_mat,
        dist_coef=dist_coef,
        alpha=0.5,  # Low-pass filter alpha value
        data_queue=data_queue
    )

    detector_thread = Thread(target=detector.process_frames)
    detector_thread.start()

    detector_thread.join()
