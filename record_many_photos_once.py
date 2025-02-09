import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import os
import shutil

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


# delete the directory if it exists
path = '/home/darkblue/quickstart/images'
if os.path.exists(path):
    shutil.rmtree(path)

# create the directory if it doesn't exist
if not os.path.exists(path):
    os.makedirs(path)

# take in counter max as an arg
counter_max = int(sys.argv[1])
if counter_max is None:
    counter_max = 5

print("counter_max: ", counter_max)
try:
    while True: # Add counter for filenames (at the start of the try block)
        while os.path.exists('/home/darkblue/quickstart/DONT_TAKE_PHOTOS'):
            time.sleep(0.01)

        counter = 0

        skip = 5
        skip_counter = 0
        while True:
            print("counter: ", counter)
            if counter >= counter_max:  # Stop after 1000 photos
                print("Reached maximum number of photos", counter)
                counter = 0
                # exit()
                break;
                    
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                print("Failed to get frames")
                continue  # Skip this iteration instead of exiting

            # if skip_counter < skip:
            #     print("skipping frame {}".format(skip_counter))
            #     skip_counter += 1
            #     time.sleep(0.01)
            #     # skip the first frame to adjust exposure
            #     continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

            # Adjust color image brightness (optional)
            color_image_bright = cv2.convertScaleAbs(color_image, alpha=1.5, beta=0)
            
            # Rotate images 90 degrees clockwise
            color_image_bright = cv2.rotate(color_image_bright, cv2.ROTATE_90_CLOCKWISE)
            depth_colormap = cv2.rotate(depth_colormap, cv2.ROTATE_90_CLOCKWISE)

            # Modified save images section with counter
            cv2.imwrite(f'{path}/color_image_{counter:04d}.png', color_image_bright)
            # cv2.imwrite(f'images/depth_image_{counter:04d}.png', depth_colormap)
            
            print(f"Images saved as color_image_{counter:04d}.png and depth_image_{counter:04d}.png")
            
            counter += 1  # Increment counter
            time.sleep(0.01)  # Add delay between captures

except KeyboardInterrupt:  # Add keyboard interrupt handling
    print("\nStopping image capture...")

finally:
    # Stop streaming
    pipeline.stop()