import os
import rospy
import rospkg
from std_msgs.msg import Int8MultiArray

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *

class MapPlugin(Plugin):

    def __init__(self, context):
        super(MapPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MapPlugin')

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

        self._mine_location_sub = rospy.Subscriber('rqt_mine_location', Int8MultiArray, self.mine_callback)
        self._mine_location = Int8MultiArray()
        self._black_color = QBrush(QColor(0, 0, 0))
        self._gray_color = QBrush(QColor(150, 150, 150))
        self._buried_items = []
        self._buried_lock = False
        self._surface_lock = False
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('minesweeper_rqt'), 'resource', 'mines_map.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MinesMapUI')
        self._alphabet = ['a', 'b', 'c', 'd', 'e', 'f',
                    'g', 'h', 'i', 'j', 'k', 'l',
                    'm', 'n', 'o', 'p', 'q', 'r',
                    's']
        x = QTableWidgetItem(0)
        x.setBackground(QBrush(QColor(255, 215, 0)))
        self._widget.map.setItem(19, 0, x)
        for i in range(0, 19, 1):
            x = QTableWidgetItem(str(19-i))
            x.setBackground(QBrush(QColor(255, 215, 0)))
            self._widget.map.setItem(i, 0, x)

            x = QTableWidgetItem(self._alphabet[i].upper())
            x.setBackground(QBrush(QColor(255, 215, 0)))
            self._widget.map.setItem(19, i+1, x)
            self._widget.total_sum.setStyleSheet('''QLCDNumber {background-color: black; color: red;}''')

            #self.listen()

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


    def mine_callback(self, data):
        self._mine_location = data
        self.listen()

    def listen(self):
        if self._mine_location.data[2] != 0:
            if self._mine_location.data[2] == 1:
                x = QTableWidgetItem()
                x.setBackground(self._gray_color)
                b = "{0}{1}".format(self._alphabet[self._mine_location.data[1]], self._mine_location.data[0] + 1)
                for i in range(0, self._widget.surface_list.count()):
                    if b == self._widget.surface_list.item(i).text():
                        break
                else:
                    self._widget.surface_list.addItem(b)
            elif self._mine_location.data[2] == -1:
                x = QTableWidgetItem()
                x.setBackground(self._black_color)
                b = "{0}{1}".format(self._alphabet[self._mine_location.data[1]], self._mine_location.data[0] + 1)
                for i in range(0, self._widget.buried_list.count()):
                    if b == self._widget.buried_list.item(i).text():
                        break
                else:
                    self._widget.buried_list.addItem(b)
            self._widget.map.setItem(19 - self._mine_location.data[0] - 1, self._mine_location.data[1] + 1, x)
            self._widget.total_sum.display(self._widget.buried_list.count() + self._widget.surface_list.count())
            self._widget.total_sum.setFrameStyle(2)
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._mine_location_sub.unregister()
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
