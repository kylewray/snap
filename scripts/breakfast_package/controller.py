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

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from slam import *
from velocity import *
from recovery import *
from teleoperator import *
from path_follower import *
from visualize import *


class Controller(object):
    """ A class to control a Kobuki using 'epic' and likely 'nova'. """

    def __init__(self):
        """ The constructor for the Controller class. """

        self.started = False
        self.resetRequired = False

        self.timer = None

        self.slam = SLAM()
        self.velocity = Velocity()
        self.recovery = Recovery()
        self.teleoperator = Teleoperator()
        self.pathFollower = PathFollower()
        self.visualize = Visualize()

        self.pubKobukiVelocity = None
        self.pubKobukiResetOdometry = None

    def start(self):
        """ Start the necessary messages to operate the Kobuki. """

        if self.started:
            rospy.logwarn("Warn[Controller.start]: Already started.")
            return

        rospy.loginfo("Info[Controller.start]: Starting main controller.")

        pubKobukiVelocityTopic = rospy.get_param("~pub_kobuki_velocity", "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        pubKobukiResetOdometryTopic = rospy.get_param("~pub_kobuki_reset_odometry", "cmd_reset_odom")
        self.pubKobukiResetOdometry = rospy.Publisher(pubKobukiResetOdometryTopic, Empty, queue_size=32)

        self.slam.start()
        self.velocity.start()
        self.recovery.start()
        self.teleoperator.start()
        self.pathFollower.start()
        self.visualize.start()

        secondsPerUpdate = 1.0 / float(rospy.get_param("~update_rate", "10.0"))
        self.timer = rospy.Timer(rospy.Duration(secondsPerUpdate), self.update)

        self.started = True

    def reset(self):
        """ Reset all of the variables that change as the robot moves. """

        rospy.loginfo("Info[Controller.reset]: Resetting main controller.")

        self.slam.reset()
        self.velocity.reset()
        self.recovery.reset()
        self.teleoperator.reset()
        self.pathFollower.reset()
        self.visualize.reset()

        # Reset the robot's odometry.
        #if self.pubKobukiResetOdometry is not None:
        #    self.pubKobukiResetOdometry.publish(Empty())

        # Stop the robot's motion.
        if self.pubKobukiVelocity is not None: 
            control = Twist()
            self.pubKobukiVelocity.publish(control)

        self.resetRequired = False

    def update(self, msg):
        """ Perform an update at the rate of the timer.

            Parameters:
                msg     --  A TimerEvent object.
        """

        if not self.started:
            rospy.logwarn("Warn[Controller.update]: Initialization has not yet completed.")
            return

        if self.resetRequired:
            self.reset()

        print("UPDATE!")

        if self.recovery.is_recovering(self.slam):
            self.recovery.perform_recovery(self.slam, self.velocity)

        elif self.teleoperator.is_activated():
            self.teleoperator.perform_teleoperation(self.slam, self.velocity)

        elif self.pathFollower.has_path(self.slam):
            self.pathFollower.perform_path_following(self.slam, self.velocity)

        self.visualize.publish_path(self.slam.get_pose_estimate())

