#!/usr/bin/env python

# Copyright 2020 PAL Robotics SL. All Rights Reserved
 
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.
#
# Author:
#   * Sammy Pfeiffer

import rospy
from sensor_msgs.msg import JointState


class CurrentPlay(object):
    def __init__(self):
        self.last_msg = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                          JointState,
                                          self.joint_states_cb,
                                          queue_size=1)
        rospy.loginfo(
            "Subscribed to: '" + str(self.js_sub.resolved_name) + "' topic.")
        self.run()

    def joint_states_cb(self, msg):
        """
        :type msg: JointState
        """
        self.last_msg = msg

    def run(self):
        """Show information on what was found in the joint states current"""
        rospy.loginfo("Waiting for first JointState message...")
        while not rospy.is_shutdown() and self.last_msg is None:
            rospy.sleep(0.2)

        # Check at a 5Hz rate to not spam
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.do_stuff_with_last_msg()
            r.sleep()

    def do_stuff_with_last_msg(self):
        """Print funny sentences about what we can guess of our status thanks to the currents read"""
        currents_sum = sum(self.last_msg.effort)
        rospy.loginfo("Looks like we are consuming " + str(currents_sum) + " ampers with our motors!")


if __name__ == '__main__':
    rospy.init_node('current_play')
    cp = CurrentPlay()
    rospy.spin()
