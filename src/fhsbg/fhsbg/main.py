import asyncio
import queue
from threading import Thread
import sys
import os
import pickle
from drone_functino import connect_drone, takeoff_velocity, control_velocity,enable_offboard_mode, set_initial_setpoint
from distance_estimation import ArucoDetector
from mavsdk import System
from camera_calc import adjust_velocity_ratios, calculate_velocity

# Ensure the current directory is in the sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)


class CameraCallback:
    def __init__(self):
        self.data_queue = queue.Queue()
        self.detector = None
        self.detector_thread = None
        self.x_filter = 0
        self.y_filter = 0
        self.z_filter = 0

    def load_calibration_data(self):
        print("Loading calibration data...")
        calibration_files_path = '/home/naor/Desktop/naor/study/fhsbg_final/src/fhsbg/fhsbg/calibration'
        with open(os.path.join(calibration_files_path, 'calibration.pkl'), 'rb') as f:
            calibration_data = pickle.load(f)
        with open(os.path.join(calibration_files_path, 'cameraMatrix.pkl'), 'rb') as f:
            cam_mat = pickle.load(f)
        with open(os.path.join(calibration_files_path, 'dist.pkl'), 'rb') as f:
            dist_coef = pickle.load(f)
        return calibration_data, cam_mat, dist_coef

    def setup_detector(self):
        print("Setting up detector...")
        calibration_data, cam_mat, dist_coef = self.load_calibration_data()
        self.detector = ArucoDetector(
            marker_size=6,  # Marker size in centimeters
            target_marker_id=73,  # The target ArUco marker ID
            fov_horizontal=87,  # Field of view in degrees for RealSense D435
            fov_vertical=58,  # Field of view in degrees for RealSense D435
            calibration_data=calibration_data,
            cam_mat=cam_mat,
            dist_coef=dist_coef,
            alpha=0.5,  # Low-pass filter alpha value
            data_queue=self.data_queue
        )
        self.detector_thread = Thread(target=self.detector.process_frames)

    def start_detector(self):
        print("Starting detector thread...")
        self.detector_thread.start()

    def stop_detector(self):
        print("Stopping detector thread...")
        self.detector_thread.join()

    async def consume_data(self):
        while True:
            try:
                data = self.data_queue.get_nowait()  # Non-blocking get
                # Access x, y, and depth data here
                self.x_filter = data['x']
                self.y_filter = data['y']
                self.z_filter = data['depth']
                #print(f"Retrieved data from queue: x={self.x_filter}, y={self.y_filter}, depth={self.z_filter}")  # Add this line
            except queue.Empty:
                await asyncio.sleep(1)  # Yield control and sleep for 1 second

    async def run(self):
        self.setup_detector()
        self.start_detector()
        await self.consume_data()


class FusenMain(CameraCallback):
    def __init__(self):
        super().__init__()
        self.drone = System()

    async def takeoff(self):
        print("Connecting to drone...")
        await connect_drone(self.drone)
        print("Drone connected, taking off...")
        await takeoff_velocity(self.drone)
        print("Takeoff complete")

    async def move(self):
        x_velocity_ratio, y_velocity_ratio, z_velocity_ratio = adjust_velocity_ratios(self.z_filter)
        v_x = calculate_velocity(self.x_filter, x_velocity_ratio)
        v_y = calculate_velocity(self.y_filter, y_velocity_ratio)
        v_z = calculate_velocity(self.z_filter, z_velocity_ratio)
        print(f"v_x: {v_x*100}, v_y: {v_y*100}, v_z: {v_z}")

        #need to config the move direction 

        await control_velocity(self.drone, v_y*100, v_x*100, 0) #at this stage we dont use the z axis - need to calibatr the vlaue (i muyltiply 10 to get some big moce )
        asyncio.sleep(1)

    async def run(self):
        camera_thread = Thread(target=self.run_camera)
        camera_thread.start()

        await self.takeoff()

        # Ensure move method is called periodically
        await enable_offboard_mode(drone=self.drone)
        await set_initial_setpoint(drone=self.drone)
        while True:
            await self.move()


        camera_thread.join()

    def run_camera(self):
        asyncio.run(super().run())


if __name__ == "__main__":
    print("Starting FusenMain application...")
    drone_app = FusenMain()
    asyncio.run(drone_app.run())
    print("FusenMain application has ended.")
