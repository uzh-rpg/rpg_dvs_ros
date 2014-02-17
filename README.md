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
4. `$ roslaunch dvs_renderer dvs.launch`  

# DVS Calibration (not yet implemented here)
The calibration of a DVS is a two-stage procedure. First, the focus must be adjusted. Then, the intrinsic camera parameters are estimated. If you can *dim your screen* (background LEDs), calibration is more convinient. If not, the patterns can also blink in software. 

## Focus Adjustment
Run the following command to display a logarithmic pattern on the screen:  
`$ roslaunch dvs_calibration focus.launch`  
If you cannot dim your screen, use the following command instead:  
`$ roslaunch dvs_calibration focus_blinking.launch`  

Adjust the focus ring of your lens until you can see the white rectangles separated.

## Instrisic Parameters
Run the following command to display a logarithmic pattern on the screen:  
`$ roslaunch dvs_calibration instrinsics.launch`  
If you cannot dim your screen, use the following command instead:  
`$ roslaunch dvs_calibration instrinsics_blinking.launch`  

This procedure is similar to the [standard camera calibration in ROS](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). 
