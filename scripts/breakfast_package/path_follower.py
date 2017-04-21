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

import math

from epic.srv import ComputePath


class PathFollower(object):
    """ A class for path following an 'epic'-generated path. """

    def __init__(self):
        """ The constructor for the PathFollower class. """

        self.started = False

        self.path = None

        self.updateRate = float(rospy.get_param(rospy.search_param('update_rate'), "0.2"))
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

        if slam is None:
            return False

        rospy.wait_for_service(self.srvEpicComputePathTopic)
        try:
            srvEpicComputePath = rospy.ServiceProxy(self.srvEpicComputePathTopic, ComputePath)

            poseStamped = PoseStamped()
            poseStamped.header.frame_id = self.subKobukiOdometryTopic
            poseStamped.header.stamp = rospy.get_rostime()
            poseStamped.pose = slam.get_pose_estimate()

            res = srvEpicComputePath(poseStamped, 0.1, 0.1, 1000)
            if res is not None:
                self.path = res.path.poses

        except rospy.ServiceException:
            rospy.logwarn("Warning[PathFollower.has_path]: Failed to execute service call ComputePath in 'epic'.")
            self.path = None

        return self.path is not None

    def _compute_speed(self, poseEstimate):
        """ Compute a speed proportional to acceleration constraints and obstacle congestion.

            Parameters:
                poseEstimate    --  The current robot pose estimate as a Pose object.

            Returns:
                The desired speed in meters per second. Default is 0.0 if no path is specified.
        """

        if self.path is None or poseEstimate is None or len(self.path) <= 1:
            return 0.0

        # Iterate over the path until 1 meter has been reached.

        return self.desiredVelocity

    def _compute_heading_adjustment(self, poseEstimate):
        """ Compute a heading adjustment proportional to the next path location and a bound.

            Parameters:
                poseEstimate    --  The current robot pose estimate as a Pose object.
        
            Returns:
                The desired signed heading adjustment. Default is 0.0 if no path is specified.
        """

        if self.path is None or poseEstimate is None or len(self.path) <= 1:
            return 0.0

        # Get the current heading (yaw).
        currentRoll, currentPitch, currentYaw = euler_from_quaternion([poseEstimate.orientation.x,
                                                                       poseEstimate.orientation.y,
                                                                       poseEstimate.orientation.z,
                                                                       poseEstimate.orientation.w])

        # Use the next pose to compute the heading, but constrain it by a bound.
        roll, pitch, yaw = euler_from_quaternion([self.path[1].pose.orientation.x,
                                                  self.path[1].pose.orientation.y,
                                                  self.path[1].pose.orientation.z,
                                                  self.path[1].pose.orientation.w])

        headingAdjustment = yaw - currentYaw
        turnRate = self.updateRate * headingAdjustment

        # The parameter 'desired_turn_rate' determines the maximum degrees per second it can turn.
        if headingAdjustment > self.desiredTurnRate:
            return self.desiredTurnRate
        elif headingAdjustment < -self.desiredTurnRate:
            return -self.desiredTurnRate
        else:
            return headingAdjustment

    def perform_path_following(self, slam):
        """ Perform path following control adjustments, sending Twist messages to the Kobuki.

            Parameters:
                slam    --  The SLAM object, which contains pose estimates.
        """

        # If there is no path, then publish empty.
        if self.path is None or slam is None or len(self.path) <= 1:
            print("ASDF")
            control = Twist()
            self.pubKobukiVelocity.publish(control)
            return

        print("QERTY")
        print(len(self.path))

        poseEstimate = slam.get_pose_estimate()

        # Check if we reached the goal to within 0.1 meter. If so, then halt the path follower.
        if math.sqrt(pow(self.path[-1].pose.position.x - poseEstimate.position.x, 2)
                   + pow(self.path[-1].pose.position.y - poseEstimate.position.y, 2)) < 0.1:
            control = Twist()
            self.pubKobukiVelocity.publish(control)
            return

        # Construct and publish the twist message which combines both the speed
        # and the angular adjustment.
        control = Twist()

        control.linear.x = self._compute_speed(poseEstimate)
        control.angular.z += self._compute_heading_adjustment(poseEstimate)

        self.pubKobukiVelocity.publish(control)

