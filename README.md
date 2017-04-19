rpg_dvs_ros
===========

# Disclaimer and License

The RPG ROS DVS package has been tested under ROS Indigo (Ubuntu 14.04) and ROS Kinetic (Ubuntu 16.04).

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a **GNU General Public License (GPL)**.


# Package Overview

The ROS DVS package provides C++ drivers for the [Dynamic Vision Sensor (DVS)](https://inilabs.com/products/dynamic-vision-sensors/) and the [Dynamic and Active-pixel Vision Sensor (DAVIS)](https://inilabs.com/products/dynamic-and-active-pixel-vision-sensor/).
It also provides a calibration tool for both intrinsic and stereo calibration.
To find out more about event cameras, visit the website of the [Institute of Neuroinformatics](http://siliconretina.ini.uzh.ch/wiki/index.php).
The package is based on [libcaer](https://github.com/inilabs/libcaer).

Authors: Elias Mueggler, Basil Huber, Luca Longinotti, Tobi Delbruck


## Publications

If you use this work in an academic context, please cite the following publications:

* E. Mueggler, B. Huber, D. Scaramuzza: **Event-based, 6-DOF Pose Tracking for High-Speed Maneuvers**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Chicago, 2014. ([PDF](http://rpg.ifi.uzh.ch/docs/IROS14_Mueggler.pdf))
* P. Lichtsteiner, C. Posch, T. Delbruck: **A 128×128 120dB 15us Latency Asynchronous Temporal Contrast Vision Sensor**. IEEE Journal of Solid State Circuits, Feb. 2008, 43(2), pp. 566-576. ([PDF](https://www.ini.uzh.ch/~tobi/wiki/lib/exe/fetch.php?media=lichtsteiner_dvs_jssc08.pdf))
* C. Brandli, R. Berner, M. Yang, S. C. Liu and T. Delbruck: **A 240 × 180 130 dB 3 us Latency Global Shutter Spatiotemporal Vision Sensor**. IEEE Journal of Solid-State Circuits, Oct. 2014, 49(10), pp. 2333-2341. ([Link](ieeexplore.ieee.org/document/6889103))


# Driver Installation

1. Make sure, libusb is installed on your system:  
   `$ sudo apt-get install libusb-1.0-0-dev`

2. Install ROS dependencies:  
   `$ sudo apt-get install ros-kinetic-camera-info-manager`  
   `$ sudo apt-get install ros-kinetic-image-view`  

3. Install catkin tools:  
   `$ sudo apt-get install python-catkin-tools`

4. Create a catkin workspace (if you have not done it yet):  
   `$ cd`  
   `$ mkdir -p catkin_ws/src`  
   `$ cd catkin_ws`  
   `$ catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`  

5. Clone the `catkin_simple` package (https://github.com/catkin/catkin_simple), which will be used to build the DVS/DAVIS driver packages:  
   `$ cd ~/catkin_ws/src`  
   `$ git clone https://github.com/catkin/catkin_simple.git`  

6. Clone this repository:  
   `$ cd ~/catkin_ws/src`  
   `$ git clone https://github.com/uzh-rpg/rpg_dvs_ros.git`  

7. Build the packages:  
  * `$ catkin build dvs_ros_driver`  (if you are using the DVS)  
  * `$ catkin build davis_ros_driver`  (if you are using the DAVIS)  

8. Only a udev rule is needed to run the DVS driver. An installation script is provided in the package `libcaer_catkin`.  
  `$ roscd libcaer_catkin`  (need to source your setup.bash file first, or just do `$ cd libcaer_catkin`)  
  `$ sudo ./install.sh`
  
9. You can test the installation by running a provided launch file. It starts the driver (DVS or DAVIS) and the renderer (an image viewer).  
  First, build the renderer:  
    `$ catkin build dvs_renderer`  
    `$ source ~/catkin_ws/devel/setup.bash`  
  Then, launch the example:  
  * `$ roslaunch dvs_renderer dvs_mono.launch`  (if you are using the DVS)
  * `$ roslaunch dvs_renderer davis_mono.launch` (if you are using the DAVIS)  
  You should get an image like this:

![dvs_rendering_screenshot_19 04 2017](https://cloud.githubusercontent.com/assets/8024432/25172262/b96baaa0-24f0-11e7-9c3e-e33f6d398a4a.png)

10. Optional: adjust the DVS/DAVIS parameters to your needs using the dynamic reconfigure GUI. Run  
   `$ rosrun rqt_reconfigure rqt_reconfigure`  
   and a window will appear. Select the `davis_ros_driver` (on the left panel) and you should get the following GUI that allows you to modify the parameters of the sensor.
   
   ![davis_ros_driver_rqt_reconfigure](https://cloud.githubusercontent.com/assets/8024432/25172274/c1267b8a-24f0-11e7-8130-af551a8a958d.png)
   
   A guide on how to modify the parameters in the bottom half of the GUI (biases) can be found here: https://inilabs.com/support/hardware/biasing/

# DVS Calibration
The calibration of a DVS is a two-stage procedure.
First, the focus must be adjusted.
Then, the intrinsic camera parameters are estimated.

## Focus Adjustment
Adjust the focus of the DVS. One way of achieving this is using a special pattern, e.g. the [Back Focus Pattern](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_calibration/pdf/backfocus.pdf).

## Instrisic Parameters
To run the intrinsic camera calibration, we use a 5x5 LED board that is blinking at 500Hz.
The calibration procedure is then started using  
`$ roslaunch dvs_calibration dvs_intrinsic.launch`  
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
However, the timestamps within the messages are synchronized.

## Calibration
1. Calibrate each DVS independently
2. Use `$ roslaunch dvs_calibration dvs_stereo.launch`  
3. Use the same checkerboard with blinking LEDs and make sure it is visible in both cameras. Collect at least 30 samples.
4. Start the calibration and check the reprojection error. Then save it (this will extend your intrinsic camera info files with the stereo information).


# Calibration Details and Parameters
The calibration requires a board with a regular grid of blinking LEDs.
In our case we have a 5x5 grid with a 0.05m distance between the LEDs.
One of the rows can be turned off (to make a 5x4 grid) to avoid confusion in the stereo case.
The following parameters can be tuned using ROS parameters:
* `dots_w`, `dots_h` (default: 5) is the number of rows and columns in the grid of LEDs
* `dot_distance` (default: 0.05) is the distance in **meters** between the LEDs

If you have your own LED board with different LEDs or blinking frequencies, you might want to tweak these parameters as well:
* `blinking_time_us` (default: 1000) is the blinking time in **micro**-seconds
* `blinking_time_tolerance_us` (default: 500) is the tolerance in **micro**-seconds to still count the transition
* `enough_transitions_threshold` (default: 200) is the minimum number of transitions before searching the LEDs
* `minimum_transitions_threshold` (default: 10) is the minimum number of transitions required to be considered in the LED search
* `minimum_led_mass` (default: 50) is the minimum "mass" of an LED blob, i.e., the sum of transitions in this blop
* `pattern_search_timeout` (default: 2.0) is the timeout in **seconds** when the transition map is reset (it is also reset when the LED grid was found)


# DAVIS Calibration
We recommend to use the frames for intrinsic calibration for the DAVIS.
More details can be found here for [monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) and [stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration).


# Troubleshooting
## New dvs_msgs format
If you recorded rosbags with a previous version of this package, they must be migrated.
The format for the timestamps changed from uint64 to rostime.
To convert an "old" bag file, use   
`$ rosbag fix old.bag new.bag`.

## Compiling error
On Ubuntu 14.04 with GCC 4.8, you will receive an error about missing file (`stdatomic.h`).
This is a [problem](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58016) related to GCC 4.8 and can be resolved by [updating to version 4.9](http://askubuntu.com/a/581497/218846):

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install gcc-4.9 g++-4.9
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9
