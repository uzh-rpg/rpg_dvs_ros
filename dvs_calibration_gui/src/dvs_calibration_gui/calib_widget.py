#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtCore import pyqtSlot
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty

class CalibWidget(QWidget):
  
  _sub_pattern_detections = None
  _sub_calibration_output = None

  _num_pattern_detections = 0
  _calibration_output = ""
  _update_required = True

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
    self._sub_calibration_output = rospy.Subscriber('dvs_calibration/output', String, self.calibration_output_cb)

    print('reset')

    self.on_button_reset_pressed()
    
    print('reset done')

  def unregister(self):
    print('Nothing to do')
  
  def pattern_detections_cb(self, msg):
    self._num_pattern_detections = msg.data
    if (self._num_pattern_detections > 0):
		self.button_start.setEnabled( True )
    self._update_required = True

  def calibration_output_cb(self, msg):
  	self._calibration_output = msg.data
  	self._update_required = True

  def update_info(self):
    if (self._update_required):
      self.numDetectionsLabel.setText(str(self._num_pattern_detections))
      self.calibrationResultText.setPlainText(self._calibration_output)
      self._update_required = False

  @Slot(bool)
  def on_button_reset_pressed(self):
    self._num_pattern_detections = 0
    self._calibration_output = ""
    self._update_required = True

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

