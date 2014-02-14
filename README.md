rpg_dvs_ros
===========

ROS packages for DVS

# Driver installation
Only a udev rule is needed to run the DVS driver. An install script is provided in the package dvs_driver.  
1. `$ roscd dvs_driver`  
2. `$ ./install.sh` (needs root privileges)

You can test the installation by running a provided launch file. It starts the driver, the renderer, an image viewer, and the dynamic reconfigure GUI.  
3. `$ roslaunch dvs_renderer dvs.launch`  
