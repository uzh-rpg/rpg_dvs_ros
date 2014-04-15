#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtCore import pyqtSlot
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty
import dynamic_reconfigure.client


class CalibWidget(QWidget):

  _sub_pattern_detections = None
  _sub_dvs_camera_info = None
  _sub_calibration_reprojection_error = None
  _client = None

  _num_pattern_detections = 0
  _dots = 4
  _border_size = 20

  _messages = []

  def __init__(self):
    print('constructor')
    # Init QWidget
    super(CalibWidget, self).__init__()
    self.setObjectName('CalibWidget')

    # load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('dvs_calib_gui'), 'resource', 'widget.ui')
    loadUi(ui_file, self)
     
    # init and start update timer for data, the timer calls the function update_info all 40ms    
    self._update_info_timer = QTimer(self)
    self._update_info_timer.timeout.connect(self.update_info)
    self._update_info_timer.start(40)
    
    self.button_reset.pressed.connect(self.on_button_reset_pressed)
    self.button_start.pressed.connect(self.on_button_start_calibration_pressed)

    self.windowOutlineButton.pressed.connect(self.on_select_window_outline_pressed)
    self.focusAdjustmentButton.pressed.connect(self.on_select_focus_adjustment_pressed)
    self.intrinsicCalibrationButton.pressed.connect(self.on_select_intrinsic_calibration_pressed)

    self.dotsSpinBox.valueChanged.connect(self.on_dots_changed)
    self.borderSizeSpinBox.valueChanged.connect(self.on_border_size_changed)
        
    self._sub_pattern_detections = rospy.Subscriber('/pattern_detections', Int32, self.pattern_detections_cb)
    self._sub_dvs_camera_info = rospy.Subscriber('/dvs_camera_info', CameraInfo, self.dvs_camera_info_cb)
    self._sub_calibration_reprojection_error = rospy.Subscriber('/calibration_reprojection_error', Float64, self.calibration_reprojection_error_cb)

    self._client = dynamic_reconfigure.client.Client("dvs_calibration")
    config = self._client.get_configuration()
    self._dots = config['dots']
    self._border_size = config['border_size']

    self.on_select_window_outline_pressed()
    
    print('Done')

  def unregister(self):
    print('Nothing to do')
  
  def pattern_detections_cb(self, msg):
    self._num_pattern_detections = msg.data
    if (msg.data == 100):
      self._messages.append('Optimization is running (this might take a while...)')

  def dvs_camera_info_cb(self, msg):
    self._messages.append('Camera info:')
    self.calibrationResultText.appendPlainText('K:' + str(msg.K))
    self.calibrationResultText.appendPlainText('D: '+ str(msg.D))

  def calibration_reprojection_error_cb(self, msg):
    self._messages.append('Reprojection Error: ' + str(msg.data))

  def update_info(self):
    self.detectionsProgressBar.setValue(self._num_pattern_detections)
    self.dotsSpinBox.setValue(self._dots)
    self.borderSizeSpinBox.setValue(self._border_size)

    while len(self._messages)>0:
      self.calibrationResultText.appendPlainText(self._messages.pop())


  @Slot(bool)  
  def on_select_window_outline_pressed(self):
    self.select_pattern(0)

    self.dotsSpinBox.setEnabled( False )
    self.borderSizeSpinBox.setEnabled( True )
    self.detectionsProgressBar.setEnabled( False )
    self.button_start.setEnabled( False )
    self.button_reset.setEnabled( False )
    self.calibrationResultText.setEnabled( False )

  @Slot(bool)  
  def on_select_focus_adjustment_pressed(self):
    self.select_pattern(1)

    self.dotsSpinBox.setEnabled( False )
    self.borderSizeSpinBox.setEnabled( False )
    self.detectionsProgressBar.setEnabled( False )
    self.button_start.setEnabled( False )
    self.button_reset.setEnabled( False )
    self.calibrationResultText.setEnabled( False )

  @Slot(bool)  
  def on_select_intrinsic_calibration_pressed(self):
    self.select_pattern(2)

    self.dotsSpinBox.setEnabled( True )
    self.borderSizeSpinBox.setEnabled( False )
    self.detectionsProgressBar.setEnabled( True )
    self.button_start.setEnabled( True )
    self.button_reset.setEnabled( True )
    self.calibrationResultText.setEnabled( True )
  
  @Slot(bool)
  def on_dots_changed(self, value):
    params = { 'dots' : value}
    self._dots = value
    config = self._client.update_configuration(params)

  @Slot(bool)
  def on_border_size_changed(self, value):
    params = { 'border_size' : value}
    self._border_size = value
    config = self._client.update_configuration(params)
  
  @Slot(bool)
  def on_button_reset_pressed(self):
    self._num_pattern_detections = 0
    self.calibrationResultText.clear()

    self.button_start.setEnabled( True )

    rospy.wait_for_service('reset_calibration')
    try:
        reset_service = rospy.ServiceProxy('reset_calibration', Empty)
        resp1 = reset_service()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    print('PRESSED RESET')

  @Slot(bool)
  def on_button_start_calibration_pressed(self):
    self._messages.append('Starting calibration...')
    self.button_start.setEnabled( False )

    rospy.wait_for_service('start_calibration')
    try:
        start_calibration_service = rospy.ServiceProxy('start_calibration', Empty)
        resp1 = start_calibration_service()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def select_pattern(self, pattern):
    params = { 'pattern' : pattern}
    config = self._client.update_configuration(params)
