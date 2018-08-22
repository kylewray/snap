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

import numpy as np


class Teleoperator(object):
    """ Control the Kobuki remotely via a joystick controller. """

    def __init__(self):
        """ The constructor for the Teleoperator class. """

        self.started = False
        self.activated = False
        self.activatedTime = rospy.get_rostime()

        self.maxTeleoperatorSpeed = float(rospy.get_param("~max_teleoperator_speed", "0.4"))
        self.maxTeleoperatorHeading = float(rospy.get_param("~max_teleoperator_heading", str(np.pi / 2.0)))
        self.joyDeadzone = float(rospy.get_param("~joy_deadzone", "0.1"))

        self.joyDesiredLongitudinal = 0.0
        self.joyDesiredLateral = 0.0

        self.subJoy = None
        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages and routines for the teleoperator. """

        if self.started:
            rospy.logwarn("Warn[Teleoperator.start]: Already started.")
            return

        rospy.loginfo("Info[Teleoperator.start]: Starting teleoperator sub-controller.")

        subJoyTopic = rospy.get_param("~sub_joy", "evt_joy")
        self.subJoy = rospy.Subscriber(subJoyTopic, Joy, self.sub_joy)

        pubKobukiVelocityTopic = rospy.get_param("~pub_kobuki_velocity", "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the teleoperator variables. """

        rospy.loginfo("Info[Teleoperator.reset]: Resetting teleoperator sub-controller.")

        self.activated = False
        self.activatedTime = rospy.get_rostime()

        self.joyDesiredLongitudinal = 0.0
        self.joyDesiredLateral = 0.0

    def is_activated(self):
        """ Check if teleoperation is activated or not.

            Returns:
                True if teleoperation is active, False otherwise.
        """

        return self.activated

    def perform_teleoperation(self, localization, velocity):
        """ Perform the teleoperation movement.

            Parameters:
                localization        --  The Localization object, which contains position and heading estimates.
                velocity    --  The Velocity object, which is a speed/heading PID controller.
        """

        if not self.started:
            rospy.logwarn("Warn[Teleoperator.perform_teleoperation]: Initialization has not yet completed.")
            return

        if not self.activated:
            return

        if abs(self.joyDesiredLongitudinal) >= self.joyDeadzone:
            desiredSpeed = self.joyDesiredLongitudinal * self.maxTeleoperatorSpeed
        else:
            desiredSpeed = 0.0

        if abs(self.joyDesiredLateral) >= self.joyDeadzone:
            desiredHeading = localization.get_heading_estimate() + self.joyDesiredLateral * self.maxTeleoperatorHeading
        else:
            desiredHeading = localization.get_heading_estimate()

        control = Twist()
        control.linear.x = localization.get_speed_estimate() + velocity.compute_speed(localization, desiredSpeed)
        control.angular.z = velocity.compute_heading(localization, desiredHeading)

        self.pubKobukiVelocity.publish(control)

    def sub_joy(self, msg):
        """ Receive information about the joystick from ROS.

            Parameters:
                msg     --  The joystick information.
        """

        # The "triangle" button activates/deactivates teleoperation, with a 1/2 second delay between each toggle.
        if msg.buttons[3] == 1 and self.activatedTime.to_sec() + 0.5 <= rospy.get_rostime().to_sec():
            self.activated = not self.activated
            self.activatedTime = rospy.get_rostime()

            if self.activated:
                rospy.loginfo("Info[Teleoperator.sub_joy]: Teleoperation activated.")
            else:
                rospy.loginfo("Info[Teleoperator.sub_joy]: Teleoperation deactivated.")

        # The left axis stick controls longitudinal and lateral desired set point relative to the robot.
        if self.activated:
            self.joyDesiredLongitudinal = msg.axes[1]
            self.joyDesiredLateral = msg.axes[0]

