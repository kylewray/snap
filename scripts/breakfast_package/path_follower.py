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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np

from epic.srv import ComputePath


class PathFollower(object):
    """ A class for path following an 'epic'-generated path. """

    def __init__(self):
        """ The constructor for the PathFollower class. """

        self.started = False
        self.paused = False

        self.path = None
        self.lastComputePathUpdateTime = None
        self.computePathSecondsPerUpdate = 1.0 / float(rospy.get_param("~compute_path_update_rate", "2.0"))

        self.pathResolution = float(rospy.get_param("~path_resolution", 0.1))
        self.minPathListSize = int(rospy.get_param("~min_path_list_size", "5"))
        self.maxPathListSize = float(rospy.get_param("~max_path_list_size", 1000))

        self.pathFollowTimeAhead = float(rospy.get_param("~path_follow_time_ahead", 3.0))

        self.maxPathFollowerSpeed = float(rospy.get_param("~max_path_follower_speed", "0.2"))
        self.maxPathFollowerHeading = float(rospy.get_param("~max_path_follower_heading", str(np.pi / 2.0)))

        self.srvEpicComputePathTopic = rospy.get_param("~srv_epic_compute_path", "epic_compute_path")

        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages to get 'epic'-generated paths and follow them. """

        if self.started:
            rospy.logwarn("Warn[PathFollower.start]: Already started.")
            return

        rospy.loginfo("Info[PathFollower.start]: Starting path follower sub-controller.")

        self.subKobukiOdometryTopic = rospy.get_param("~sub_kobuki_odometry", "odom")

        pubKobukiVelocityTopic = rospy.get_param("~pub_kobuki_velocity", "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the path follower variables. """

        rospy.loginfo("Info[PathFollower.reset]: Resetting path follower sub-controller.")

        self.path = None
        self.lastComputePathUpdateTime = None

    def has_path(self, slam):
        """ Determine if a path is set or not.

            Parameters:
                slam    --  The SLAM object, which contains pose estimates.

            Returns:
                True if a path exists, False otherwise.
        """

        if slam is None:
            return False

        currentTime = rospy.get_rostime().to_sec()

        if (self.lastComputePathUpdateTime is None 
                or (self.lastComputePathUpdateTime + self.computePathSecondsPerUpdate <= currentTime)):
            self.lastComputePathUpdateTime = currentTime

            rospy.wait_for_service(self.srvEpicComputePathTopic)
            try:
                srvEpicComputePath = rospy.ServiceProxy(self.srvEpicComputePathTopic, ComputePath)

                poseStamped = PoseStamped()
                poseStamped.header.frame_id = self.subKobukiOdometryTopic
                poseStamped.header.stamp = rospy.get_rostime()
                poseStamped.pose = slam.get_pose_estimate()

                res = srvEpicComputePath(poseStamped, self.pathResolution, 0.1, self.maxPathListSize)
                if res is not None and len(res.path.poses) >= self.minPathListSize:
                    self.path = res.path.poses
                else:
                    raise rospy.ServiceException()

            except rospy.ServiceException:
                rospy.logwarn("Warning[PathFollower.has_path]: Failed to execute service call ComputePath in 'epic'.")
                self.path = None

        return self.path is not None


    def _compute_closest_path_index(self, pose):
        """ Compute the closest index along the path to the pose given.

            Parameters:
                pose    --  The PoseStamped object to test.

            Returns:
                The index along the path that is closest to this point, or None on an error.
        """

        if self.path is None or len(self.path) == 0 or type(pose) is not Pose:
            print(type(self.path), len(self.path), type(pose))
            return None

        pathDistances = sorted([(index, pow(element.pose.position.x - pose.position.x, 2)
                                        + pow(element.pose.position.y - pose.position.y, 2))
                                for index, element in enumerate(self.path)], key=lambda z: z[1])

        return pathDistances[0][0]

    def perform_path_following(self, slam, velocity):
        """ Perform path following control adjustments, sending Twist messages to the Kobuki.

            Parameters:
                slam        --  The SLAM object, which contains pose estimates.
                velocity    --  The Velocity object, which is a speed/heading PID controller.
        """

        # If there is no path or the path following is paused, then publish empty.
        if self.path is None or slam is None or len(self.path) <= self.minPathListSize or self.paused:
            control = Twist()
            self.pubKobukiVelocity.publish(control)
            return

        poseEstimate = slam.get_pose_estimate()
        distanceToGoal = float(np.sqrt(pow(self.path[-1].pose.position.x - poseEstimate.position.x, 2)
                                       + pow(self.path[-1].pose.position.y - poseEstimate.position.y, 2)))

        # Check if we reached the goal to within 0.1 meter. If so, then halt the path follower.
        if distanceToGoal < 0.1:
            control = Twist()
            self.pubKobukiVelocity.publish(control)
            return

        # Compute a pose that is 3 seconds away given the current speed.
        closestPathIndex = self._compute_closest_path_index(poseEstimate)
        currentSpeedEstimate = slam.get_speed_estimate()
        distanceTravelled = 0.0
        localGoalPathIndex = -1
        for localGoalPathIndex in range(closestPathIndex + 1, len(self.path)):
            newLocation = self.path[localGoalPathIndex].pose.position
            oldLocation = self.path[localGoalPathIndex - 1].pose.position
            distanceTravelled += float(np.sqrt(pow(newLocation.x - oldLocation.x, 2) + pow(newLocation.y - oldLocation.y, 2)))
            if distanceTravelled / currentSpeedEstimate >= self.pathFollowTimeAhead:
                break

        # Change the speed proportional to distance to the goal. Also change the heading path index
        # selected proportional to the curvature of the path.
        desiredSpeed = self.maxPathFollowerSpeed * min(1.0, distanceToGoal)
        desiredHeading = float(np.arctan2(self.path[localGoalPathIndex].pose.position.y - poseEstimate.position.y,
                                          self.path[localGoalPathIndex].pose.position.x - poseEstimate.position.x))

        # Construct and publish the twist message which combines both the speed
        # and the angular adjustment.
        control = Twist()

        control.linear.x = slam.get_speed_estimate() + velocity.compute_speed(slam, desiredSpeed)
        control.angular.z = velocity.compute_heading(slam, desiredHeading)

        self.pubKobukiVelocity.publish(control)

