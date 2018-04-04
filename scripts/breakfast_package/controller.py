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

import random as rnd

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point

from breakfast.srv import *
from breakfast.msg import *

from localization import *
from velocity import *
from cartographer import *
from recovery import *
from teleoperator import *
from simple_mover import *
from path_follower import *
from visualize import *


class Controller(object):
    """ A class to control a Kobuki using 'epic' and likely 'nova'. """

    def __init__(self):
        """ The constructor for the Controller class. """

        self.started = False
        self.resetRequired = False

        self.timer = None

        self.currentAction = ActionType.NONE

        self.localization = Localization()
        self.velocity = Velocity()
        self.cartographer = Cartographer()
        self.recovery = Recovery()
        self.teleoperator = Teleoperator()
        self.simpleMover = SimpleMover()
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

        self.localization.start()
        self.velocity.start()
        self.cartographer.start()
        self.recovery.start()
        self.teleoperator.start()
        self.simpleMover.start()
        self.pathFollower.start()
        self.visualize.start()

        secondsPerUpdate = 1.0 / float(rospy.get_param("~update_rate", "10.0"))
        self.timer = rospy.Timer(rospy.Duration(secondsPerUpdate), self.update)

        srvActionMoveTopic = rospy.get_param("~action_move_topic", "~action_move")
        self.srvActionMove = rospy.Service(srvActionMoveTopic,
                                           ActionMove,
                                           self.srv_action_move)

        srvActionMoveInGridTopic = rospy.get_param("~action_move_in_grid_topic", "~action_move_in_grid")
        self.srvActionMoveInGrid = rospy.Service(srvActionMoveInGridTopic,
                                                 ActionMoveInGrid,
                                                 self.srv_action_move_in_grid)

        srvActionNavigateTopic = rospy.get_param("~action_navigate_topic", "~action_navigate")
        self.srvActionNavigate = rospy.Service(srvActionNavigateTopic,
                                           ActionNavigate,
                                           self.srv_action_navigate)

        srvActionNavigateToRegionTopic = rospy.get_param("~action_navigate_to_region_topic",
                                                         "~action_navigate_to_region")
        self.srvActionNavigateToRegion = rospy.Service(srvActionNavigateToRegionTopic,
                                           ActionNavigateToRegion,
                                           self.srv_action_navigate_to_region)

        srvActionPushTopic = rospy.get_param("~action_push_topic", "~action_push")
        self.srvActionPush = rospy.Service(srvActionPushTopic,
                                           ActionPush,
                                           self.srv_action_push)

        srvActionAlignTopic = rospy.get_param("~action_align_topic", "~action_align")
        self.srvActionAlign = rospy.Service(srvActionAlignTopic,
                                           ActionAlign,
                                           self.srv_action_align)

        self.started = True

    def reset(self):
        """ Reset all of the variables that change as the robot moves. """

        rospy.loginfo("Info[Controller.reset]: Resetting main controller.")

        self.currentAction = ActionType.NONE

        self.localization.reset()
        self.velocity.reset()
        self.cartographer.reset()
        self.recovery.reset()
        self.teleoperator.reset()
        self.simpleMover.reset()
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

        if self.recovery.is_recovering(self.localization):
            self.recovery.perform_recovery(self.localization, self.velocity)

            if self.simpleMover.has_goal():
                self.simpleMover.reset()
            if self.pathFollower.has_path():
                self.pathFollower.reset()
            self.currentAction = ActionType.NONE

        elif self.teleoperator.is_activated():
            self.teleoperator.perform_teleoperation(self.localization, self.velocity)

            if self.simpleMover.has_goal():
                self.simpleMover.reset()
            if self.pathFollower.has_path():
                self.pathFollower.reset()
            self.currentAction = ActionType.NONE

        elif self.currentAction in [ActionType.MOVE, ActionType.MOVE_IN_GRID, ActionType.PUSH]:
            if self.simpleMover.has_goal():
                self.simpleMover.perform_simple_moving(self.localization, self.velocity)

                if self.simpleMover.at_goal():
                    self.recovery.set_expecting_bump(False)
                    self.simpleMover.reset()
                    self.currentAction = ActionType.NONE

        elif self.currentAction in [ActionType.NAVIGATE, ActionType.NAVIGATE_TO_REGION, ActionType.ALIGN]:
            if self.pathFollower.has_goal():
                self.pathFollower.perform_path_following(self.localization, self.velocity)

                if self.pathFollower.at_goal():
                    self.pathFollower.reset()
                    self.currentAction = ActionType.NONE

        self.visualize.publish_path(self.localization)

    def srv_action_move(self, request):
        """ Handle a service request for the move action.

            Parameters:
                request     --  The ActionMoveRequest object.

            Returns:
                The ActionMoveResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveResponse(self.currentAction)

        self.simpleMover.set_goal_relative_heading(self.localization, request.heading)
        self.simpleMover.set_goal_relative_distance(self.localization, request.distance)

        self.currentAction = ActionType.MOVE

        return ActionMoveResponse(ActionType.NONE)

    def srv_action_move_in_grid(self, request):
        """ Handle a service request for the move in a grid action.

            Parameters:
                request     --  The ActionMoveInGridRequest object.

            Returns:
                The ActionMoveInGridResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveResponse(self.currentAction)

        adjust = -self.localization.get_heading_estimate()
        pi = float(np.pi)

        if request.action == ActionMoveInGridRequest.NORTH:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust + pi / 2.0)
        elif request.action == ActionMoveInGridRequest.SOUTH:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust - pi / 2.0)
        elif request.action == ActionMoveInGridRequest.EAST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust + 0.0)
        elif request.action == ActionMoveInGridRequest.WEST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust + pi)
        elif request.action == ActionMoveInGridRequest.NORTH_EAST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust + pi / 4.0)
        elif request.action == ActionMoveInGridRequest.NORTH_WEST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust + pi * 3.0 / 4.0)
        elif request.action == ActionMoveInGridRequest.SOUTH_EAST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust - pi / 4.0)
        elif request.action == ActionMoveInGridRequest.SOUTH_WEST:
            self.simpleMover.set_goal_relative_heading(self.localization, adjust - pi * 3.0 / 4.0)

        self.simpleMover.set_goal_relative_distance(self.localization, request.grid_cell_size)

        self.currentAction = ActionType.MOVE

        return ActionMoveResponse(ActionType.NONE)

    def srv_action_navigate(self, request):
        """ Handle a service request for the navigate action.

            Parameters:
                request     --  The ActionNavigateRequest object.

            Returns:
                The ActionNavigateResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveResponse(self.currentAction)

        self.pathFollower.set_goals([Point(3.0, 2.0, 0.0)])
        #self.pathFollower.set_goals(request.points)  # TODO TODO TODO TODO TODO UNCOMMENT AFTER WORKING

        self.currentAction = ActionType.NAVIGATE

        return ActionMoveResponse(ActionType.NONE)

    def srv_action_navigate_to_region(self, request):
        """ Handle a service request for the navigate to a region action.

            Parameters:
                request     --  The ActionNavigateToRegionRequest object.

            Returns:
                The ActionNavigateToRegionResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveResponse(self.currentAction)

        # Assign goals to be 42 random locations within the region. TODO: Refine this behavior.
        goals = list()
        for i in range(42):
            goals += [self.cartographer.get_random_point_in_region(request.region_uid)]

        if None in goals:
            return ActionMoveResponse(self.currentAction)

        self.pathFollower.set_goals(goals)

        self.currentAction = ActionType.NAVIGATE

        return ActionMoveResponse(ActionType.NONE)

    def srv_action_push(self, request):
        """ Handle a service request for the push action.

            Parameters:
                request     --  The ActionPushRequest object.

            Returns:
                The ActionPushResponse object.
        """

        pass

    def srv_action_align(self, request):
        """ Handle a service request for the align action.

            Parameters:
                request     --  The ActionAlignRequest object.

            Returns:
                The ActionAlignResponse object.
        """

        pass

