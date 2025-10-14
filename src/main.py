#!/usr/bin/env python3

# Covert raw RealSense Depth data to RViz PointCloud2 data
# Use Pyrealsense2 to obtain RS data, no launch file is needed

import sys, copy, time, cv2

import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError


import numpy as np

import rospy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField

## convert RealSense depth data to ROS PointCloud2
import struct
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


class rs_get():
    def __init__(self, serial, alias="", width = 1280, height = 720):
        '''
        serial: The serial NO. of the RealSense
        alias: name your camera topic
        The default width and height are set to  1280x720
        '''
        if alias=="":
            self.alias = "/rs_" + str(serial)
        else:
            self.alias = alias

        self.height = height
        self.width = width

        self.image_pub = rospy.Publisher(self.alias+"/color/raw", Image, queue_size = 10)
        self.k_pub = rospy.Publisher(self.alias+"/color/camera_info", CameraInfo, queue_size = 10)
        self.depth_pub = rospy.Publisher(self.alias+"/depth/raw", Image, queue_size = 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(str(serial))
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        self.k = [0.0]*9 # camera's intrinsic parameters
        self.distort = [0.0]*5
        self.get_cam_param()
        print("{} cam param is : {}".format(alias, self.k))
        self.is_data_updated = False

        ## wait for 1s to maker sure color images arrive
        rospy.sleep(1)
        self.color_raw = None
        self.depth_raw = None
        self.get_rgbd()

    def set_config(self, config):
        ## config = "Default", "High Accuracy", "High Density"
        if config not in ["Default", "High Accuracy", "High Density"]:
            config = "Default"
        ## use preset configuration
        preset_range = self.depth_sensor.get_option_range(rs.option.visual_preset)
        for i in range(int(preset_range.max)):
            visual_preset = self.depth_sensor.get_option_value_description(rs.option.visual_preset, i)
            if visual_preset == config:
                self.depth_sensor.set_option(rs.option.visual_preset, i)

        data_retrieve = -1
        while data_retrieve == -1:
            print("waiting for data...")
            data_retrieve = self.get_rgbd()

    def get_cam_param(self):
        st_profile = self.profile.get_stream(rs.stream.depth)
        self.intr = st_profile.as_video_stream_profile().get_intrinsics()
        self.k[0] = self.intr.fx
        self.k[2] = self.intr.ppx
        self.k[4] = self.intr.fy
        self.k[5] = self.intr.ppy
        self.k[8] = 1.0

        for i in range(5):
            self.distort[i] = self.intr.coeffs[i]

    def get_rgbd(self):
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        try:
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                return -1

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            self.color_img = color_image
            self.depth_1d = depth_image
            return 1

        except:
            return -1

    def pub_data(self):
        img_msg = self.bridge.cv2_to_imgmsg(self.color_img)
        self.image_pub.publish(img_msg)
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_1d)
        self.depth_pub.publish(depth_msg)

if __name__ == '__main__':
    print(cv2.__version__)
    rospy.init_node("d405", anonymous = True)
    np.set_printoptions(suppress=True)

    front_cam = rs_get("851112063978", alias="front_cam")

    rospy.sleep(1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        front_cam.get_rgbd()
        front_cam.pub_data()
        rate.sleep()