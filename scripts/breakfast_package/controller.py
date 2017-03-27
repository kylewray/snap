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
from nav_msgs.msg import Odometry

#from breakfast.msg import *
#from breakfast.srv import *

from recovery import *
from teleoperator import *
from visualize import *


class Controller(object):
    """ A class to control a Kobuki using 'epic' and likely 'nova'. """

    def __init__(self):
        """ The constructor for the Controller class. """

        # Main controller variables used for determining initialization.
        self.started = False
        self.resetRequired = False

        # The sub-controller variables to control various aspects.
        self.recovery = Recovery()
        self.visualize = Visualize()

        self.subKobukiOdometry = None
        self.pubKobukiVelocity = None
        self.pubKobukiResetOdom = None

    def start(self):
        """ Start the necessary messages to operate the Kobuki. """

        if self.started:
            rospy.logwarn("Warn[Controller.start]: Already started.")
            return

        subKobukiOdometryTopic = rospy.get_param(rospy.search_param('sub_kobuki_odometry'))
        self.subKobukiOdometry = rospy.Subscriber(subKobukiOdometryTopic,
                                                  Odometry,
                                                  self.sub_kobuki_odometry)

        pubKobukiVelocityTopic = rospy.get_param(rospy.search_param('pub_kobuki_velocity'))
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        pubKobukiResetOdomTopic = rospy.get_param(rospy.search_param('pub_kobuki_reset_odom'))
        self.pubKobukiResetOdom = rospy.Publisher(pubKobukiResetOdomTopic, Empty, queue_size=32)

        self.visualize.start()

        self.started = True

    def reset(self):
        """ Reset all of the variables that change as the robot moves. """

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Reset the robot's odometry.
        #if self.pubKobukiResetOdometry is not None:
        #    self.pubKobukiResetOdometry.publish(Empty())

        # Stop the robot's motion.
        if self.pubKobukiVelocity is not None: 
            control = Twist()
            self.pubKobukiVelocity.publish(control)

        self.resetRequired = False

    def sub_kobuki_odometry(self, msg):
        """ Update the odometry information.

            Parameters:
                msg     --  The Odometry message data.
        """

        # TODO: Currently, we put the 'update' behavior here, but in the future
        # it should be moved to a timer perhaps.
        if self.resetRequired:
            self.reset()

        if self.recovery.check_recovery():
            self.recovery.move_recovery()

        self.visualize.publish_path(msg.pose.pose)

