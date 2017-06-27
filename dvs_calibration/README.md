# DAVIS Calibration
We recommend to use the frames for intrinsic calibration for the DAVIS.
More details can be found here for [monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) and [stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration).

# DVS Calibration
The calibration of a DVS is a two-stage procedure.
First, the focus must be adjusted.
Then, the intrinsic camera parameters are estimated.

## Focus Adjustment
Adjust the focus of the DVS. One way of achieving this is using a special pattern, e.g. the [Back Focus Pattern](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_calibration/pdf/backfocus.pdf).

## Intrinsic Parameters
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
