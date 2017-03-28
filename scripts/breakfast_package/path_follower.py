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

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from epic.srv import ComputePath


class PathFollower(object):
    """ A class for path following an 'epic'-generated path. """

    def __init__(self):
        """ The constructor for the PathFollower class. """

        self.started = False

        self.path = None

        self.desiredVelocity = float(rospy.get_param(rospy.search_param('desired_velocity'), "0.2"))
        self.desiredTurnRate = float(rospy.get_param(rospy.search_param('desired_turn_rate'), "1.0"))

        self.srvEpicComputePathTopic = rospy.get_param(rospy.search_param('srv_epic_compute_path'), "epic_compute_path")

        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages to get 'epic'-generated paths and follow them. """

        if self.started:
            rospy.logwarn("Warn[PathFollower.start]: Already started.")
            return

        rospy.loginfo("Info[PathFollower.start]: Starting path follower sub-controller.")

        self.subKobukiOdometryTopic = rospy.get_param(rospy.search_param('sub_kobuki_odometry'), "odom")

        pubKobukiVelocityTopic = rospy.get_param(rospy.search_param('pub_kobuki_velocity'), "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the path follower variables. """

        rospy.loginfo("Info[PathFollower.reset]: Resetting path follower sub-controller.")

        self.path = None

    def has_path(self, slam):
        """ Determine if a path is set or not.

            Parameters:
                slam    --  The SLAM object, which contains pose estimates.

            Returns:
                True if a path exists, False otherwise.
        """

        rospy.wait_for_service(self.srvEpicComputePathTopic)
        try:
            srvEpicComputePath = rospy.ServiceProxy(self.srvEpicComputePathTopic, ComputePath)

            poseStamped = PoseStamped()
            poseStamped.header.frame_id = self.subKobukiOdometryTopic
            poseStamped.header.stamp = rospy.get_rostime()
            poseStamped.pose = slam.get_pose_estimate()

            res = srvEpicComputePath(poseStamped, 0.1, 0.1, 10000)
            if res is not None:
                self.path = res.path.poses

        except rospy.ServiceException:
            rospy.logwarn("Warning[PathFollower.has_path]: Failed to execute service call ComputePath in 'epic'.")
            self.path = None

        return self.path is not None

    def perform_path_following(self, slam):
        """ Perform path following control adjustments, sending Twist messages to the Kobuki.

            Parameters:
                slam    --  The SLAM object, which contains pose estimates.
        """

        # Select the local goal position along the path that does not deviate from the current pose much,
        # and has a low change in angle (derivative), but also is as far away from the starting location
        # as possible. Store this value in 'index'.
        index = 100

        if len(self.path) < index:
            return

        poseEstimate = slam.get_pose_estimate()
        currentRoll, currentPitch, currentYaw = euler_from_quaternion([poseEstimate.orientation.x,
                                                                       poseEstimate.orientation.y,
                                                                       poseEstimate.orientation.z,
                                                                       poseEstimate.orientation.w])

        # Compute the speed based on the distance from the current pose to the 'index' pose.
        velocity = self.desiredVelocity

        # Construct the twist message which combines both the speed and the angular adjustment.
        control = Twist()

        control.linear.x = velocity

        roll, pitch, yaw = euler_from_quaternion([self.path[index].pose.orientation.x,
                                                  self.path[index].pose.orientation.y,
                                                  self.path[index].pose.orientation.z,
                                                  self.path[index].pose.orientation.w])

        control.angular.z += yaw - currentYaw

        self.pubKobukiVelocity.publish(control)

