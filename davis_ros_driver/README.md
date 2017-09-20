# Photometric Calibration of the DAVIS

## Recording calibration data

* Create a folder for the calibration data:

        mkdir ~/photometric_calib
        mkdir ~/photometric_calib/images

* (Optional) If the folder was already here and filled with data, clean it:

        rm ~/photometric_calib/images/*.png ~/photometric_calib/*.txt

* Launch the DAVIS driver:

        rosrun davis_ros_driver davis_ros_driver _photometric_calibration_sample_size:=1500 _photometric_calibration_min_exposure:=50 _photometric_calibration_max_exposure:=50000 _photometric_calibration_data_folder:=/home/$USER/photometric_calib

* Set the logger level to INFO so you can follow the status of the photometric calibration procedure:

        rosservice call /davis_ros_driver/set_logger_level ros INFO

* To visualize the process live:

        rqt_image_view /dvs/image_raw

* Trigger the photometric calibration procedure:

        rostopic pub /dvs/calibrate_photometric std_msgs/Empty "{}"

The driver will slowly sweep a range of exposures. When it is done, it will save the images + exposure times in the folder of your choice (default: /tmp).

## Calibrating the sensor

* Clone [this repository](https://github.com/uzh-rpg/mono_dataset_code), and compile it following the instructions in the README (no need to follow the aruco marker detection step).

* Run the photometric calibration program (where maxValidIntensityValue contains the max intensity value returned by your sensor, default: 255):

        cd mono_dataset_code/bin
        ./responseCalib ~/photometric_calib maxValidIntensityValue=255

This will create a ```photometricCalibResult``` folder containing the response curve in ```pcalib.txt``` + some debug outputs.

* You can use the scripts in the ```scripts``` folder to display the response curve, or the images used for calibration.

