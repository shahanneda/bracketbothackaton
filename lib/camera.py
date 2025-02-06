import pyrealsense2 as rs
import cv2
import numpy as np

class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        
        # Enable streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        
    def get_frames(self):
        # Wait for a coherent pair of frames
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image

    def release(self):
        # Stop streaming
        self.pipeline.stop()

class USBCamera:
    def __init__(self, index=0):
        # Initialize USB camera
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise Exception("Could not open video device")
        # Set properties if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def release(self):
        self.cap.release()

if __name__ == "__main__":
    realsense = RealsenseCamera()
    usb = USBCamera()
