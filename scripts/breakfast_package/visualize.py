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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Visualize(object):
    """ Visualization of the breakfast robot's state and behavior. """

    def __init__(self):
        """ The constructor for the Visualize class. """

        self.started = False

        self.rawPath = list()
        self.lastPathPublishTime = rospy.get_rostime()

        self.publishRate = float(rospy.get_param(rospy.search_param('pub_path_rate'), "0.2"))
        self.subKobukiOdometryTopic = rospy.get_param(rospy.search_param('sub_kobuki_odometry'), "odom")

        self.pubPath = None

    def start(self):
        """ Start the necessary messages for visualization. """

        if self.started:
            rospy.logwarn("Warn[Visualize.start]: Already started.")
            return

        pubPathTopic = rospy.get_param(rospy.search_param('pub_path'), "path")
        self.pubPath = rospy.Publisher(pubPathTopic, Path, queue_size=32)

        self.started = True

    def publish_path(self, pose):
        """ Record the path taken, but only at a certain rate.

            Parameters:
                pose    --  The pose data (odom) to add to the path.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_path]: Visualize has not yet been started.")
            return

        currentTime = rospy.get_rostime()

        if self.lastPathPublishTime.to_sec() + self.publishRate <= currentTime.to_sec():
            # Add to raw path with a timestamped pose from odometers.
            poseStamped = PoseStamped()
            poseStamped.header.frame_id = self.subKobukiOdometryTopic
            poseStamped.header.stamp = currentTime
            poseStamped.pose = pose

            self.rawPath += [poseStamped]

            # Create and publish the path.
            path = Path()
            path.header.frame_id = self.subKobukiOdometryTopic
            path.header.stamp = currentTime
            path.poses = self.rawPath

            self.pubPath.publish(path)

            self.lastPathPublishTime = currentTime

