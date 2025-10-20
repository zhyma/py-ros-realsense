Need to setup:
- RealSense (pyrealsense2)
- OpenCV
- ROS

For connect to RealSense, publish image to YOUR_CAMERA_NAME/color/raw and depth to YOUR_CAMERA_NAME/depth/raw.

- To start recording, use `rosservice call /front_cam/start_recording "PATH_TO_SAVE"`

- To stop recording, use `rosservice call /front_cam/stop_recording`