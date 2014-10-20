rpg_dvs_ros
===========

ROS packages for the Dynamic Vision Sensor (DVS).  
Find out more on the website of the [Institute of Neuroinformatics](http://siliconretina.ini.uzh.ch/wiki/index.php).

# Driver Installation
Make sure, libusb is installed on your system:  
1. `$ sudo apt-get install libusb-1.0-0-dev`

Only a udev rule is needed to run the DVS driver. An install script is provided in the package dvs_driver.  
2. `$ roscd dvs_driver`  
3. `$ ./install.sh` (needs root privileges)

You can test the installation by running a provided launch file. It starts the driver, the renderer, an image viewer, and the dynamic reconfigure GUI.  
4. `$ roslaunch dvs_renderer mono.launch`  


# DVS Calibration
The calibration of a DVS is a two-stage procedure. First, the focus must be adjusted. Then, the intrinsic camera parameters are estimated.   

## Focus Adjustment
Adjust the focus of the DVS. One way of achieving this is using a special pattern, e.g. the [Back Focus Pattern](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_calibration/pdf/backfocus.pdf).

## Instrisic Parameters
To run the intrinsic camera calibration, we use a 5x5 LED board that is blinking at 500Hz.
The calibration procedure is then started using  
`$ roslaunch dvs_calibration intrinsic.launch`  
You will see an RQT interface with all necessary information.
Top left is the calibration GUI, which displays the amount of detected patterns.
**Currently, pattern detection does not seem to work indoors. Try close to a window.**
Collect at least 30 samples before starting the calibration.
**This can take up to a few minutes** and freezes your RQT GUI.
Once done, the calibration parameters are shown and can be saved.
The camera parameters will be stored in `~/.ros/camera_info`.
When you plug that DVS again, this calibration file will be loaded and published as `/dvs/camera_info`. 

The image viewers below show the following:

1. Accumulated DVS renderings: you should see the blinking LEDs and the gradients in the scene
2. Detected blinking: black regions mean more detections. Once the pattern is detected, the counter in the calibration GUI should increment. The detected pattern is also visualized for a short moment.
3. Rectified DVS rendering: once the calibration is done, you can see how well it turned out. Check if straight lines are still straight, especially in the border of the image.


# Stereo DVS Calibration

## Setup
Connect the two DVS from OUT (master) to IN (slave). 
GND must not be connected if both DVS are connected over USB to the same computer, to avoid ground loops.
Time synchronization is performed automatically in the driver software.
Since each DVS has a separate driver, the ROS messages might arrive at different times. 
Hover, the timestamps within the messages are synchronized.

## Calibration
1. Calibrate each DVS independently
2. Use `$ roslaunch dvs_calibration stereo.launch`  
3. Use the same checkerboard with blinking LEDs and make sure it is visible in both cameras. Collect at least 30 samples.
4. Start the calibration and check the reprojection error. Then save it (this will extend your intrinsic camera info files with the stereo information).


# Recording datasets with RGB-D data
To record datasets, run the following ROS launch files (from different terminals):
```
$ roslaunch dvs_renderer dvs.launch
$ roslaunch openni_launch openni.launch
```
You can then record the data using  
`$ rosbag record /camera/depth/camera_info /camera/depth/image_raw /camera/rgb/camera_info              /camera/rgb/image_raw /dvs/camera_info /dvs/events /optitrack/camera_rig`  
If you do not use OptiTrack for ground truth, you can remove the last topic.
