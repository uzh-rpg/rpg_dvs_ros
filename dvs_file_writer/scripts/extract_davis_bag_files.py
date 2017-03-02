#!/usr/bin/python

import argparse
import rosbag
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import os

# only required for simulated scenes
try:
    import OpenEXR, Imath
    import numpy as np
except ImportError, e:
    print("To extract depth image of simulated scenes, please install OpenEXR and Imath")
    pass

# arguments
parser = argparse.ArgumentParser()
parser.add_argument("bag", help="ROS bag file to extract")
parser.add_argument("--event_topic", default="/dvs/events", help="Event topic")
parser.add_argument("--image_topic", default="/dvs/image_raw", help="Image topic")
parser.add_argument("--depthmap_topic", default="/dvs/depthmap", help="Depth map topic")
parser.add_argument("--imu_topic", default="/dvs/imu", help="IMU topic")
parser.add_argument("--calib_topic", default="/dvs/camera_info", help="Camera info topic")
parser.add_argument("--groundtruth_topic", default="/optitrack/davis", help="Ground truth topic")
parser.add_argument("--reset_time", help="Remove time offset", action="store_true")
args = parser.parse_args()

# Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
bridge = CvBridge()

def get_filename(i):
    return "images/frame_" + str(i).zfill(8)  + ".png"
    
def get_depth_filename(i):
    return "depthmaps/frame_" + str(i).zfill(8)  + ".exr"

def timestamp_str(ts):
    return str(ts.secs) + "." + str(ts.nsecs).zfill(9)

# Create folders and files
if not os.path.exists("images"):
    os.makedirs("images")
    
if not os.path.exists("depthmaps"):
    os.makedirs("depthmaps")

image_index = 0
depthmap_index = 0
event_sum = 0
imu_msg_sum = 0
groundtruth_msg_sum = 0
calib_written = False

events_file = open('events.txt', 'w')
images_file = open('images.txt', 'w')
depthmaps_file = open('depthmaps.txt', 'w')
imu_file = open('imu.txt', 'w')
calib_file = open('calib.txt', 'w')
groundtruth_file = open('groundtruth.txt', 'w')

with rosbag.Bag(args.bag, 'r') as bag:
    # reset time?
    reset_time = rospy.Time()
    if args.reset_time:
        first_msg = True
        for topic, msg, t in bag.read_messages():
            got_stamp = False
            if topic == args.image_topic:
                stamp = msg.header.stamp
                got_stamp = True
            elif topic == args.event_topic:
                stamp = msg.events[0].ts
                got_stamp = True
            elif topic == args.depthmap_topic:
                stamp = msg.header.stamp
                got_stamp = True
            elif topic == args.imu_topic:
                stamp = msg.header.stamp
                got_stamp = True

            if got_stamp:
                if first_msg:
                    reset_time = stamp
                    first_msg = False
                else:
                    if stamp < reset_time:
                        reset_time = stamp
    print "Reset time: " + timestamp_str(reset_time)

    for topic, msg, t in bag.read_messages():
        # Images
        if topic == args.image_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError, e:
                print e

            images_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            images_file.write(get_filename(image_index) + "\n")

            cv2.imwrite(get_filename(image_index), cv_image)

            image_index = image_index + 1

        # events
        elif topic == args.event_topic:
            for e in msg.events:
                events_file.write(timestamp_str(e.ts - reset_time) + " ")
                events_file.write(str(e.x) + " ")
                events_file.write(str(e.y) + " ")
                events_file.write(("1" if e.polarity else "0") + "\n")
                event_sum = event_sum + 1

        # IMU
        elif topic == args.imu_topic:
            imu_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            imu_file.write(str(msg.linear_acceleration.x) + " ")
            imu_file.write(str(msg.linear_acceleration.y) + " ")
            imu_file.write(str(msg.linear_acceleration.z) + " ")
            imu_file.write(str(msg.angular_velocity.x) + " ")
            imu_file.write(str(msg.angular_velocity.y) + " ")
            imu_file.write(str(msg.angular_velocity.z) + "\n")
            imu_msg_sum = imu_msg_sum + 1

        # calibration
        elif topic == args.calib_topic:
            if not calib_written:
                calib_file.write(str(msg.K[0]) + " ")
                calib_file.write(str(msg.K[4]) + " ")
                calib_file.write(str(msg.K[2]) + " ")
                calib_file.write(str(msg.K[5]) + " ")
                calib_file.write(str(msg.D[0]) + " ")
                calib_file.write(str(msg.D[1]) + " ")
                calib_file.write(str(msg.D[2]) + " ")
                calib_file.write(str(msg.D[3]))
                if len(msg.D) > 4:
                    calib_file.write(" " + str(msg.D[4]))
                calib_file.write("\n")
                calib_written = True

        # ground truth
        elif topic == args.groundtruth_topic:
            # case for Optitrack (geometry_msgs/PoseStamped)
            if hasattr(msg, 'pose'):
                pose = msg
            # case for linear slider (std_msgs/Float64)
            elif hasattr(msg, 'data'):
                pose = PoseStamped()
                pose.header.stamp = t
                pose.pose.position.x = msg.data
                pose.pose.orientation.w = 1.

            groundtruth_file.write(timestamp_str(pose.header.stamp - reset_time) + " ")
            groundtruth_file.write(str(pose.pose.position.x) + " ")
            groundtruth_file.write(str(pose.pose.position.y) + " ")
            groundtruth_file.write(str(pose.pose.position.z) + " ")
            groundtruth_file.write(str(pose.pose.orientation.x) + " ")
            groundtruth_file.write(str(pose.pose.orientation.y) + " ")
            groundtruth_file.write(str(pose.pose.orientation.z) + " ")
            groundtruth_file.write(str(pose.pose.orientation.w) + "\n")

            groundtruth_msg_sum = groundtruth_msg_sum + 1
            
        # Depth maps
        elif topic == args.depthmap_topic:
            try:
                cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
            except CvBridgeError, e:
                print e

            depthmaps_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            depthmaps_file.write(get_depth_filename(depthmap_index) + "\n")

            # Write depth map to EXR file
            exr_header = OpenEXR.Header(cv_depth.shape[1], cv_depth.shape[0])
            exr_header['channels'] = {'Z': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT))}
            exr = OpenEXR.OutputFile(get_depth_filename(depthmap_index), exr_header)
            exr.writePixels({'Z' : cv_depth.tostring()})
            exr.close()

            depthmap_index = depthmap_index + 1

# statistics (remove missing groundtruth or IMU file if not available)
print "All data extracted!"
print "Events:       " + str(event_sum)
print "Images:       " + str(image_index)
print "Depth maps:   " + str(depthmap_index)
print "IMU:          " + str(imu_msg_sum)
print "Ground truth: " + str(groundtruth_msg_sum)

# close all files
events_file.close()
images_file.close()
depthmaps_file.close()
imu_file.close()
groundtruth_file.close()

# clean up
if imu_msg_sum == 0:
    os.remove("imu.txt")
    print "Removed IMU file since there were no messages."

if groundtruth_msg_sum == 0:
    os.remove("groundtruth.txt")
    print "Removed ground truth file since there were no messages."
    
if depthmap_index == 0:
    os.remove("depthmaps.txt")
    os.removedirs("depthmaps")
    print "Removed depthmaps file since there were no messages."
