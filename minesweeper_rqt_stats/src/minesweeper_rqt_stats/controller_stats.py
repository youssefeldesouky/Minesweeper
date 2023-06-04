import os
import rospy
import rospkg
from minesweeper_msgs.msg import ControllerStats
from std_msgs.msg import Int8

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *

class StatsPlugin(Plugin):

    def __init__(self, context):
        super(StatsPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StatsPlugin')

        self._stat_sub = rospy.Subscriber("/controller_stats", ControllerStats, self.stats_cb)
        self._stat_ir_sub = rospy.Subscriber("/ir_state", Int8, self.ir_cb)
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('minesweeper_rqt_stats'), 'resource', 'controller_stats.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ControllerStatsUI')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def stats_cb(self, data):
        self._widget.profile.setText("Profile #{}".format(str(data.profile)))
        self._widget.magnet.setText("ON" if data.magnet == 1 else "OFF")
        self._widget.max_lin.setText(data.max_lin)
        self._widget.max_ang.setText(data.max_ang)
        self._widget.cur_lin.setText(data.cur_lin)
        self._widget.cur_ang.setText(data.cur_ang)

    def ir_cb(self, data):
        state = "ON" if not data.data else "OFF"
        self._widget.ir.setText(state)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
