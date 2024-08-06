import cv2 as cv
from cv2 import aruco
import numpy as np
import cupy as cp
import pickle
import pyrealsense2 as rs
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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

class ArucoDetector(Node):
    def __init__(self, marker_size, target_marker_id, fov_horizontal, fov_vertical, calibration_data, cam_mat, dist_coef, alpha=0.5):
        super().__init__('aruco_detector')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_data', 10)
        self.marker_size = marker_size
        self.target_marker_id = target_marker_id
        self.fov_horizontal = fov_horizontal
        self.fov_vertical = fov_vertical
        self.cam_mat = cam_mat
        self.dist_coef = dist_coef
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        if hasattr(aruco, 'DetectorParameters_create'):
            self.param_markers = aruco.DetectorParameters_create()
        else:
            self.param_markers = aruco.DetectorParameters()  # Use older method if DetectorParameters_create is not available
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Reduce resolution to improve performance
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.frame_center = None
        self.x_filter = LowPassFilter(alpha)
        self.y_filter = LowPassFilter(alpha)
        self.depth_filter = LowPassFilter(alpha)

    def start_pipeline(self):
        try:
            self.pipeline.start(self.config)
            self.get_logger().info("Pipeline started.")
        except RuntimeError as e:
            self.get_logger().error(f"RealSense pipeline couldn't start: {e}")
            exit(1)

    def stop_pipeline(self):
        self.pipeline.stop()
        self.get_logger().info("Pipeline stopped.")

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
            while rclpy.ok():
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

                            data = Float32MultiArray()
                            data.data = [float(x_meters_filtered), float(y_meters_filtered), float(fused_distance_filtered)]
                            self.publisher_.publish(data)

                            self.get_logger().info(f"Detected Marker ID: {ids[0]}")
                            self.get_logger().info(f"x_meters: {x_meters:.4f}, y_meters: {y_meters:.4f}, fused_distance: {fused_distance:.4f}")
                            self.get_logger().info(f"Filtered - x_meters: {x_meters_filtered:.4f}, y_meters_filtered: {y_meters_filtered:.4f}, fused_distance: {fused_distance_filtered:.4f}")

                            # Draw the center of the ArUco marker
                            cv.circle(frame, (center[0], center[1]), 5, (0, 0, 255), -1)  # Red dot for marker center

                            break

                    if not found_marker:
                        data = Float32MultiArray()
                        data.data = [0.0, 0.0, 0.0]
                        self.publisher_.publish(data)
                        self.get_logger().info("Target marker not found.")
                else:
                    data = Float32MultiArray()
                    data.data = [0.0, 0.0, 0.0]
                    self.publisher_.publish(data)
                    self.get_logger().info("No markers detected.")

                # Draw the center of the frame
                cv.circle(frame, self.frame_center, 5, (255, 0, 0), -1)  # Blue dot for frame center

                # Display the frame
                cv.imshow('Aruco Detector', frame)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop_pipeline()
            cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    detector = ArucoDetector(
        marker_size=6,  # Marker size in centimeters
        target_marker_id=73,  # The target ArUco marker ID
        fov_horizontal=87,  # Field of view in degrees for RealSense D435
        fov_vertical=57,  # Field of view in degrees for RealSense D435
        calibration_data=None,
        cam_mat=np.array([[616.36529541, 0., 316.68893433], [0., 615.4887085, 241.23072815], [0., 0., 1.]]),  # Example camera matrix
        dist_coef=np.array([0.136733, -0.239285, 0.001236, 0.000663, 0.000000]),  # Example distortion coefficients
        alpha=0.5  # Low-pass filter alpha value
    )

    detector_thread = Thread(target=detector.process_frames)
    detector_thread.start()

    rclpy.spin(detector)

    detector_thread.join()
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
