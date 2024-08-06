import pyrealsense2 as rs
import cv2
import numpy as np

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Start streaming
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

num = 0

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        img = np.asanyarray(color_frame.get_data())

        k = cv2.waitKey(5)

        if k == 27:
            break
        elif k == ord('s'): # wait for 's' key to save and exit
            cv2.imwrite('/home/naor/Desktop/naor/study/fhsbg/calibration/image/' + str(num) + '.png', img)
            print("image saved!")
            num += 1

        cv2.imshow('Img 1', img)

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

