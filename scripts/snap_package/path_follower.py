#! /usr/bin/env python

""" The MIT License (MIT)

    Copyright (c) 2018 Kyle Hollins Wray, University of Massachusetts

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

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

import math
import numpy as np

from epic.srv import ModifyGoals, ResetFreeCells, ComputePath


class PathFollower(object):
    """ A class for path following an 'epic'-generated path. """

    def __init__(self):
        """ The constructor for the PathFollower class. """

        self.started = False
        self.atGoal = False

        self.path = None
        self.goalPositions = list()
        self.lastComputePathUpdateTime = None
        self.lastTimeGoalWasSet = None
        self.initialDelayBeforeFollowingPath = float(rospy.get_param("~initial_delay_before_following_path", 5.0))
        self.computePathSecondsPerUpdate = 1.0 / float(rospy.get_param("~compute_path_update_rate", 3.0))

        self.pathResolution = float(rospy.get_param("~path_resolution", 0.1))
        self.minPathListSize = int(rospy.get_param("~min_path_list_size", "1"))
        self.maxPathListSize = int(rospy.get_param("~max_path_list_size", 5000))

        self.pathFollowTimeAhead = float(rospy.get_param("~path_follow_time_ahead", 3.0))
        self.maxPathFollowerSpeed = float(rospy.get_param("~max_path_follower_speed", 0.25))

        self.mapFrameID = rospy.get_param("~map_frame_id", "map")

        self.srvEpicAddGoalsTopic = rospy.get_param("~srv_epic_add_goals", "epic_add_goals")
        self.srvEpicRemoveGoalsTopic = rospy.get_param("~srv_epic_remove_goals", "epic_remove_goals")
        self.srvEpicResetFreeCellsTopic = rospy.get_param("~srv_epic_reset_free_cells", "epic_reset_free_cells")
        self.srvEpicComputePathTopic = rospy.get_param("~srv_epic_compute_path", "epic_compute_path")

        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages to get 'epic'-generated paths and follow them. """

        if self.started:
            rospy.logwarn("Warn[PathFollower.start]: Already started.")
            return

        rospy.loginfo("Info[PathFollower.start]: Starting path follower sub-controller.")

        pubKobukiVelocityTopic = rospy.get_param("~pub_kobuki_velocity", "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the path follower variables. """

        rospy.loginfo("Info[PathFollower.reset]: Resetting path follower sub-controller.")

        rospy.wait_for_service(self.srvEpicResetFreeCellsTopic)
        try:
            srvEpicResetFreeCells = rospy.ServiceProxy(self.srvEpicResetFreeCellsTopic, ResetFreeCells)

            res = srvEpicResetFreeCells()
            if res is None or not res.success:
                raise rospy.ServiceException()

        except rospy.ServiceException:
            rospy.logwarn("Warning[PathFollower.reset]: Failed to execute service call ResetFreeCells in 'epic'.")

        if len(self.goalPositions) > 0:
            rospy.wait_for_service(self.srvEpicRemoveGoalsTopic)
            try:
                srvEpicRemoveGoals = rospy.ServiceProxy(self.srvEpicRemoveGoalsTopic, ModifyGoals)

                res = srvEpicRemoveGoals(self._get_pose_stamped_goal_list())
                if res is None or not res.success:
                    raise rospy.ServiceException()

            except rospy.ServiceException:
                rospy.logwarn("Warning[PathFollower.reset]: Failed to execute service call ModifyGoals in 'epic'.")

        self.atGoal = False

        self.path = None
        self.goalPositions = list()
        self.lastComputePathUpdateTime = None
        self.lastTimeGoalWasSet = None

    def _get_pose_stamped_goal_list(self):
        """ Return the list of PoseStamped goal objects.

            Returns:
                A list of PoseStamped goal objects.
        """

        poseStampedList = list()

        for position in self.goalPositions:
            poseStamped = PoseStamped()
            poseStamped.header.frame_id = self.mapFrameID
            poseStamped.header.stamp = rospy.get_rostime()
            poseStamped.pose = Pose()
            poseStamped.pose.position = position
            poseStampedList += [poseStamped]

        return poseStampedList

    def set_goals(self, points):
        """ Set the goal for the path follower. This calls 'epic' services a few times.

            Parameters:
                points  --  The desired goals as a list of Point objects (x, y, z) in meters.
        """

        if type(points) is not list or len(points) == 0:
            return

        # Reset clears the path, but also importantly calls 'epic' to clear the goals
        # and reset the free space cells to default values.
        self.reset()

        # Now, assign the new goal positions and add them as goals.
        self.goalPositions = points

        rospy.wait_for_service(self.srvEpicAddGoalsTopic)
        try:
            srvEpicAddGoals = rospy.ServiceProxy(self.srvEpicAddGoalsTopic, ModifyGoals)

            res = srvEpicAddGoals(self._get_pose_stamped_goal_list())
            if res is None or not res.success:
                raise rospy.ServiceException()

        except rospy.ServiceException:
            rospy.logwarn("Warning[PathFollower.set_goals]: Failed to execute service call ModifyGoals in 'epic'.")

        # Remember when we set a goal last.
        self.lastTimeGoalWasSet = rospy.get_rostime().to_sec()

    def at_goal(self):
        """ Determine if we are at the goal or not.

            Returns:
                True if we are at the goal, False otherwise.
        """

        return self.atGoal

    def has_goal(self):
        """ Determine if goal positions have been set or not.

            Returns:
                True if goal(s) exists, False otherwise.
        """

        return self.goalPositions is not None and len(self.goalPositions) > 0

    def has_path(self):
        """ Determine if a path is set or not.

            Returns:
                True if a path exists, False otherwise.
        """

        currentTime = rospy.get_rostime().to_sec()

        # If the initial delay has not been satisfied, then there is no path yet.
        # This is to ensure that the quality of the path is high.
        if (self.lastTimeGoalWasSet is None
                or self.lastTimeGoalWasSet + self.initialDelayBeforeFollowingPath > currentTime):
            return False

        # Otherwise, if we have a path then say we have one.
        else:
            return self.path is not None

    def _compute_path(self, positionEstimate):
        """ Compute the actual path given a proper localization.

            Parameters:
                positionEstimate    --  The position estimate of the robot as a Point object.

            Returns:
                True if a path exists, False otherwise.
        """

        currentTime = rospy.get_rostime().to_sec()

        if (self.lastComputePathUpdateTime is None 
                or self.lastComputePathUpdateTime + self.computePathSecondsPerUpdate <= currentTime):
            self.lastComputePathUpdateTime = currentTime

            rospy.wait_for_service(self.srvEpicComputePathTopic)
            try:
                srvEpicComputePath = rospy.ServiceProxy(self.srvEpicComputePathTopic, ComputePath)

                poseStamped = PoseStamped()
                poseStamped.header.frame_id = self.mapFrameID
                poseStamped.header.stamp = rospy.get_rostime()
                poseStamped.pose = Pose()
                poseStamped.pose.position = Point(positionEstimate.x, positionEstimate.y, 0.0)

                res = srvEpicComputePath(poseStamped, self.pathResolution, 0.1, self.maxPathListSize)

                if res is not None and len(res.path.poses) >= self.minPathListSize:
                    self.path = res.path.poses
                else:
                    raise rospy.ServiceException()

            except rospy.ServiceException:
                rospy.logwarn("Warning[PathFollower._compute_path]: %s" %
                              ("Failed to execute service call ComputePath in 'epic'."))
                self.path = None

            if self.path is None:
                try:
                    rospy.logwarn("Warning[PathFollower._compute_path]: %s" %
                                    ("Failed first attempt. Trying a few nearby positions to get a path..."))

                    scaleOfRandomness = 0.25
                    directionOfRandomness = np.random.permutation([[-1.0, -1.0], [0.0, -1.0], [1.0, -1.0],
                                                                    [-1.0, 0.0], [0.0, 0.0], [1.0, 0.0],
                                                                    [-1.0, 1.0], [0.0, 1.0], [1.0, 1.0]])
                    for r in directionOfRandomness:
                        poseStamped.pose.position = Point()
                        poseStamped.pose.position.x = positionEstimate.x + scaleOfRandomness * r[0]
                        poseStamped.pose.position.y = positionEstimate.y + scaleOfRandomness * r[1]
                        res = srvEpicComputePath(poseStamped, self.pathResolution, 0.1, self.maxPathListSize)
                        if res is not None and len(res.path.poses) >= self.minPathListSize:
                            self.path = res.path.poses
                            break

                    if res is None or len(res.path.poses) < self.minPathListSize:
                        raise rospy.ServiceException()

                except rospy.ServiceException:
                    rospy.logwarn("Warning[PathFollower._compute_path]: %s" %
                                ("Failed to execute service call ComputePath in 'epic', even after extra attempts."))
                    self.path = None

    def _compute_closest_path_index(self, position):
        """ Compute the closest index along the path to the pose given.

            Parameters:
                position    --  The Point object to test.

            Returns:
                The index along the path that is closest to this point, or None on an error.
        """

        if self.path is None or len(self.path) == 0 or type(position) is not Point:
            return None

        pathDistances = sorted([(index, pow(element.pose.position.x - position.x, 2)
                                        + pow(element.pose.position.y - position.y, 2))
                                for index, element in enumerate(self.path)], key=lambda z: z[1])

        return pathDistances[0][0]

    def _compute_distance_to_nearest_goal(self, positionEstimate):
        """ Compute the distance to the nearest goal.

            Parameters:
                positionEstimate    --  The position estimate of the robot as a Point object.

            Returns:
                The distance to the nearest goal. This returns infinity if there are no goals.
        """

        if not self.has_goal() or type(positionEstimate) is not Point:
            return np.inf

        goalDistances = sorted([math.sqrt(pow(element.x - positionEstimate.x, 2) +
                                          pow(element.y - positionEstimate.y, 2))
                                for element in self.goalPositions])

        return math.sqrt(goalDistances[0])

    def perform_path_following(self, localization, velocity):
        """ Perform path following control adjustments, sending Twist messages to the Kobuki.

            Parameters:
                localization    --  The Localization object, which contains position and heading estimates.
                velocity        --  The Velocity object, which is a speed/heading PID controller.
        """

        # If there is no path, then publish empty.
        if localization is None or not self.has_goal():
            self.pubKobukiVelocity.publish(Twist())
            return

        positionEstimate = localization.get_position_estimate()

        # Now attempt to get a path, if the path is not defined then also publish empty.
        self._compute_path(positionEstimate)
        if not self.has_path():
            self.pubKobukiVelocity.publish(Twist())
            return

        # We have a path! Use it to check if we are at the goal.
        distanceToNearestGoal = self._compute_distance_to_nearest_goal(positionEstimate)

        # Check if we reached the goal to within 4.2 times of the path resolution (in meters). If so,
        # then halt the path follower since we will call this "arriving at the goal."
        if distanceToNearestGoal < self.pathResolution * 4.2:
            self.atGoal = True
            self.pubKobukiVelocity.publish(Twist())
            return

        # Otherwise, compute a pose that is 3 seconds away given the current speed.
        closestPathIndex = self._compute_closest_path_index(positionEstimate)
        currentSpeedEstimate = localization.get_speed_estimate()
        distanceTravelled = 0.0
        localGoalPathIndex = -1
        for localGoalPathIndex in range(closestPathIndex + 1, len(self.path)):
            newPosition = self.path[localGoalPathIndex].pose.position
            oldPosition = self.path[localGoalPathIndex - 1].pose.position
            distanceTravelled += float(np.sqrt(pow(newPosition.x - oldPosition.x, 2)
                                               + pow(newPosition.y - oldPosition.y, 2)))
            if currentSpeedEstimate == 0.0 or distanceTravelled / currentSpeedEstimate >= self.pathFollowTimeAhead:
                break

        # Change the speed proportional to distance to the goal. Also change the heading path index
        # selected proportional to the curvature of the path.
        desiredSpeed = self.maxPathFollowerSpeed * min(1.0, distanceToNearestGoal)
        desiredHeading = float(np.arctan2(self.path[localGoalPathIndex].pose.position.y - positionEstimate.y,
                                          self.path[localGoalPathIndex].pose.position.x - positionEstimate.x))

        # Slow the desired speed down based on how much we have to turn (i.e., the error in heading).
        error = desiredHeading - localization.get_heading_estimate()
        if abs(error) > np.pi:
            if error >= 0.0:
                error = desiredHeading - (localization.get_heading_estimate() + float(np.pi) * 2.0)
            elif error < 0.0:
                error = (desiredHeading + float(np.pi) * 2.0) - localization.get_heading_estimate()
        desiredSpeed /= (abs(error) + 1.0)

        # Construct and publish the twist message which combines both the speed
        # and the angular adjustment.
        control = Twist()

        control.linear.x = localization.get_speed_estimate() + velocity.compute_speed(localization, desiredSpeed)
        control.angular.z = velocity.compute_heading(localization, desiredHeading)

        # If there were invalid computations for some reason, then fix them.
        if np.isnan(control.linear.x):
            control.linear.x = 0.0
            rospy.logwarn("Warning[PathFollower.perform_path_following]: Nan detected in linear x-control message.")
        if np.isnan(control.angular.z):
            control.angular.z = 0.0
            rospy.logwarn("Warning[PathFollower.perform_path_following]: Nan detected in angular z-control message.")

        self.pubKobukiVelocity.publish(control)

