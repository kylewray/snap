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
from nav_msgs.msg import Odometry


class SLAM(object):
    """ A class for creating a map and localizing within it (SLAM). """

    def __init__(self):
        """ The constructor for the SLAM class. """

        self.started = False

        self.poseEstimate = None

        self.subKobukiOdometry = None

    def start(self):
        """ Start the necessary messages to create a map and localize. """

        if self.started:
            rospy.logwarn("Warn[SLAM.start]: Already started.")
            return

        rospy.loginfo("Info[SLAM.start]: Starting SLAM sub-controller.")

        subKobukiOdometryTopic = rospy.get_param(rospy.search_param('sub_kobuki_odometry'), "odom")
        self.subKobukiOdometry = rospy.Subscriber(subKobukiOdometryTopic,
                                                  Odometry,
                                                  self.sub_kobuki_odometry)

        self.started = True

    def reset(self):
        """ Reset the SLAM variables. """

        rospy.loginfo("Info[SLAM.reset]: Resetting SLAM sub-controller.")

        self.poseEstimate = None

    def get_pose_estimate(self):
        """ Return the current pose estimate (localization).

            Returns:
                The current pose estimate as a Pose object.
        """

        return self.poseEstimate

    def sub_kobuki_odometry(self, msg):
        """ Update the odometry information.

            Parameters:
                msg     --  The Odometry message data.
        """

        if not self.started:
            rospy.logwarn("Warn[SLAM.sub_kobuki_odometry]: Initialization has not yet completed.")
            return

        self.poseEstimate = msg.pose.pose

