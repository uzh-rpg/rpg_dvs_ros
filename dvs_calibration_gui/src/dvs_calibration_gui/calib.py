#!/usr/bin/env python
import os
import rospy
import argparse
from qt_gui.plugin import Plugin
from .calib_widget import CalibWidget

class Calib(Plugin):
  """
  Subclass of Plugin to display Quad status
  """
  def __init__(self, context):
    
    # Init Plugin
    super(Calib, self).__init__(context)
    self.setObjectName('CalibPlugin')

    # Load arguments
    # TODO load topic name from args
    args = self._parse_args(context.argv())

    # Create QWidget
    self._widget = CalibWidget()
    
    # Show _widget.windowTitle on left-top of each plugin (when 
    # it's set in _widget). This is useful when you open multiple 
    # plugins at once. Also if you open multiple instances of your 
    # plugin at once, these lines add number to make it easy to 
    # tell from pane to pane.
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    
    # Add widget to the user interface
    context.add_widget(self._widget)
    
  def shutdown_plugin(self):
    self._widget.unregister()
    pass    
    
  def _parse_args(self, argv):
    parser = argparse.ArgumentParser(prog='dvs_calibration_gui', add_help=False)
    group = parser.add_argument_group('Options for dvs_calibration_gui plugin')
    group.add_argument('topic', type=argparse.FileType('r'), nargs='*', default=[], help='Calib Info Topic to display')
    return parser.parse_args(argv)

  def save_settings(self, plugin_settings, instance_settings):
     # TODO save intrinsic configuration, usually using:
     # instance_settings.set_value(k, v)
     # if(self._widget._connected):      
     #     print('Save Settings')  
     #     namespace = self._widget._quad_namespace
     #     instance_settings.set_value('namespace', namespace)
     pass
# 
  def restore_settings(self, plugin_settings, instance_settings):
     # TODO restore intrinsic configuration, usually using:
     # v = instance_settings.value(k)
     # namespace = instance_settings.value('namespace', 'default')
     # self._widget.namespace_text.setText(namespace)
     pass

  #def trigger_configuration(self):
      # Comment in to signal that the plugin has a way to configure
      # This will enable a setting button (gear icon) in each dock widget title bar
      # Usually used to open a modal configuration dialog
