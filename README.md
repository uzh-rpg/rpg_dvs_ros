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

# Recording datasets with RGB-D data
To record datasets, run the following ROS launch files (from different terminals):
```
$ roslaunch dvs_renderer dvs.launch
$ roslaunch openni_launch openni.launch
```
You can then record the data using  
`$ rosbag record /camera/depth/camera_info /camera/depth/image_raw /camera/rgb/camera_info              /camera/rgb/image_raw /dvs/camera_info /dvs/events /optitrack/camera_rig`  
If you do not use OptiTrack for ground truth, you can remove the last topic.


# DVS Calibration
The calibration of a DVS is a two-stage procedure. First, the focus must be adjusted. Then, the intrinsic camera parameters are estimated. If you can *dim your screen* (background LEDs), calibration is more convinient. If not, the patterns can also blink in software. 

## Focus Adjustment (currently not working)
Run the following command to display a logarithmic pattern on the screen:  
`$ roslaunch dvs_calibration focus.launch`  
If you cannot dim your screen, use the following command instead:  
`$ roslaunch dvs_calibration focus_blinking.launch`  

Adjust the focus ring of your lens until you can see the white rectangles separated.
You can also use your keyboard or any other fine-structured scene to adjust the focus.

## Instrisic Parameters
To run the intrinsic camera calibration, we use a 5x5 LED board that is blinking at 500Hz.
The calibration procedure is then started using  
`$ roslaunch dvs_calibration intrinsic.launch`  
You will see an RQT interface with all necessary information.
Top left is the calibration GUI, which displays the amount of detected patterns and, once done, the calibration parameters. 
The image viewers below show the following:

1. Accumulated DVS renderings: you should see the blinking LEDs and the gradients in the scene
2. Detected blinking: black regions mean more detections. Once the pattern is detected, the counter in the calibration GUI should increment. The detected pattern is also visualized for a short moment.
3. Rectified DVS rendering: once the calibration is done, you can see how well it turned out. Check if straight lines are still straight, especially in the border of the image.

Once you have collected enough patterns (in the range of 50), click on "Start Calibration". Note that this might take a few minutes and freezes your RQT GUI.
When this is done, the reprojection error and the camera calibration parameters are shown. 
If you click on "Save Calibration", these parameters are stored in `~/.ros/camera_info/` together with the DVS serial number.
When you plug that DVS again, this calibration file will be loaded and published as `/dvs/camera_info`. 
If you are unhappy with the calibration, use the "Reset" button at any time.
