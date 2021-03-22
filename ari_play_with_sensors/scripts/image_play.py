#!/usr/bin/env python

# Copyright 2020 PAL Robotics SL. All Rights Reserved
 
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.
#
# Author:
#   * Sammy Pfeiffer

from PyQt4 import QtGui, QtCore
import rospy
from sensor_msgs.msg import CompressedImage


class Main(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(Main, self).__init__(parent)
        self.image_label = QtGui.QLabel()
        self.image_label.resize(640, 480)
        self.pixmap = QtGui.QPixmap()

        self.layout = QtGui.QHBoxLayout()
        self.layout.addWidget(self.image_label)
        self.central_widget = QtGui.QWidget()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.last_msg = None
        self.image_sub = rospy.Subscriber('/head_front_camera/color/image_raw/compressed',
                                          CompressedImage,
                                          self.image_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.image_sub.resolved_name) + "' topic.")

        # Use Qt timers to fire up the update of image
        # as the ROS callback is in another thread and we can't do it from there
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.show_image)
        self.timer.start(1.0 / 30.0)  # show 30hz image

    def image_cb(self, msg):
        self.last_msg = msg

    def show_image(self):
        if self.last_msg is not None:
            self.pixmap.loadFromData(self.last_msg.data)
            self.image_label.setPixmap(self.pixmap)
            self.image_label.adjustSize()

if __name__ == '__main__':
    rospy.init_node('image_play')
    app = QtGui.QApplication(["image_play window"])
    myWidget = Main()
    myWidget.show()
    app.exec_()
