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

class CalibWidget(QWidget):
  
  _sub_pattern_detections = None
  _sub_dvs_camera_info = None
  _sub_calibration_reprojection_error = None

  _num_pattern_detections = 0

  _messages = []

  def __init__(self):
    print('constructor')
    # Init QWidget
    super(CalibWidget, self).__init__()
    self.setObjectName('CalibWidget')

    # load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('dvs_calibration_gui'), 'resource', 'widget.ui')
    loadUi(ui_file, self)
     
    # init and start update timer for data, the timer calls the function update_info all 40ms    
    self._update_info_timer = QTimer(self)
    self._update_info_timer.timeout.connect(self.update_info)
    self._update_info_timer.start(40)
    
    self.button_reset.pressed.connect(self.on_button_reset_pressed)
    self.button_start.pressed.connect(self.on_button_start_calibration_pressed)
    self.button_save.pressed.connect(self.on_button_save_calibration_pressed)
        
    self._sub_pattern_detections = rospy.Subscriber('dvs_calibration/pattern_detections', Int32, self.pattern_detections_cb)
    self._sub_dvs_camera_info = rospy.Subscriber('dvs_calibration/camera_info', CameraInfo, self.dvs_camera_info_cb)
    self._sub_calibration_reprojection_error = rospy.Subscriber('dvs_calibration/calibration_reprojection_error', Float64, self.calibration_reprojection_error_cb)

    print('reset')

    self.on_button_reset_pressed()
    
    print('reset done')

  def unregister(self):
    print('Nothing to do')
  
  def pattern_detections_cb(self, msg):
    self._num_pattern_detections = msg.data
    if (self._num_pattern_detections > 5):
      self.button_start.setEnabled( True )

  def dvs_camera_info_cb(self, msg):
    self._messages.append('D: '+ str(msg.D))
    self._messages.append('K:' + str(msg.K))
    self._messages.append('Camera info:')

  def calibration_reprojection_error_cb(self, msg):
    self._messages.append('Reprojection Error: ' + str(msg.data))

  def update_info(self):
    self.numDetectionsLabel.setText(str(self._num_pattern_detections))

    while len(self._messages)>0:
      self.calibrationResultText.appendPlainText(self._messages.pop())
   
  @Slot(bool)
  def on_button_reset_pressed(self):
    self._num_pattern_detections = 0
    self.calibrationResultText.clear()

    self.button_start.setEnabled( False )

    try:
      rospy.wait_for_service('dvs_calibration/reset', 1)
      try:
        reset_service = rospy.ServiceProxy('dvs_calibration/reset', Empty)
        resp = reset_service()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except:
      print "service not available..."
      pass

  @Slot(bool)
  def on_button_start_calibration_pressed(self):
    self._messages.append('Starting calibration...')
    self.button_start.setEnabled( False )

    try:
      rospy.wait_for_service('dvs_calibration/start', 1)
      try:
        start_calibration_service = rospy.ServiceProxy('dvs_calibration/start', Empty)
        resp = start_calibration_service()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except:
      print "service not available..."
      pass

  @Slot(bool)
  def on_button_save_calibration_pressed(self):
    self._messages.append('Saving calibration...')

    try:
      rospy.wait_for_service('dvs_calibration/save', 1)
      try:
        save_calibration_service = rospy.ServiceProxy('dvs_calibration/save', Empty)
        resp = save_calibration_service()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except:
      print "service not available..."
      pass

