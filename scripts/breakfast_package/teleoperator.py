#! /usr/bin/env python

""" The MIT License (MIT)

    Copyright (c) 2017 Kyle Hollins Wray, University of Massachusetts

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Teleoperator(object):
    """ Control the Kobuki remotely via a joystick controller. """

    def __init__(self):
        """ The constructor for the Teleoperator class. """

        self.started = False
        self.activated = False
        self.activatedTime = rospy.get_rostime()

        self.desiredVelocity = float(rospy.get_param(rospy.search_param('desired_velocity'), "0.2"))
        self.desiredTurnRate = float(rospy.get_param(rospy.search_param('desired_turn_rate'), "1.0"))

        self.subJoy = None
        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages and routines for the teleoperator. """

        subJoyTopic = rospy.get_param(rospy.search_param('sub_joy'), "evt_joy")
        self.subJoy = rospy.Subscriber(subJoyTopic, Joy, self.sub_joy)

        pubKobukiVelocityTopic = rospy.get_param(rospy.search_param('pub_kobuki_velocity'), "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def is_activated(self):
        """ Check if teleoperation is activated or not.

            Returns:
                True if teleoperation is active, False otherwise.
        """

        return self.activated

    def perform_teleoperation(self):
        """ Perform the teleoperation movement. """

        if not self.started:
            rospy.logwarn("Warn[Teleoperator.perform_teleoperation]: Initialization has not yet completed.")
            return

        if not self.activated:
            return

        control = Twist()
        control.linear.x = self.joySpeed * self.desiredVelocity
        control.angular.z = self.joyHeading * self.desiredTurnRate

        self.pubKobukiVelocity.publish(control)

    def sub_joy(self, msg):
        """ Receive information about the joystick from ROS.

            Parameters:
                msg     --  The joystick information.
        """

        # The "Y" button activates/deactivates teleoperation, with a 1/2 second delay between each toggle.
        if msg.buttons[3] == 1 and self.activatedTime.to_sec() + 0.5 <= rospy.get_rostime().to_sec():
            self.activated = not self.activated
            self.activatedTime = rospy.get_rostime()

            if self.activated:
                print("Teleoperation activated.")
            else:
                print("Teleoperation deactivated.")

        # The "left axes" controls the speed (forward/backward).
        self.joySpeed = msg.axes[1]

        # The "right axes" controls the heading (left/right).
        self.joyHeading = msg.axes[3]

        #print(msg.axes)
        #print(msg.buttons)

