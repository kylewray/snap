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

import random as rnd

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from kobuki_msgs.msg import BumperEvent

from snap.srv import *
from snap.msg import *

from localization import *
from cartographer import *
from velocity import *
from snap_map import *
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

        self.timer = None

        self.currentAction = ActionType.NONE
        self.observationActionCompleteResult = ObservationActionComplete.NONE
        self.subactionQueue = list()

        self.snapMap = SnapMap()
        self.localization = Localization(self.snapMap)
        self.cartographer = Cartographer(self.snapMap, self.localization)
        self.velocity = Velocity()
        self.recovery = Recovery()
        self.teleoperator = Teleoperator()
        self.simpleMover = SimpleMover()
        self.pathFollower = PathFollower()
        self.visualize = Visualize()

        self.subKobukiBumper = None

        self.pubKobukiVelocity = None
        self.pubKobukiResetOdometry = None

        self.pubObservationActionComplete = None
        self.pubObservationRecovery = None
        self.pubObservationTeleoperator = None

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

        pubObservationActionCompleteTopic = rospy.get_param("~pub_observation_action_complete",
                                                            "~observation_action_complete")
        self.pubObservationActionComplete = rospy.Publisher(pubObservationActionCompleteTopic,
                                                            ObservationActionComplete, queue_size=32)

        pubObservationDetectedObjectsTopic = rospy.get_param("~pub_observation_detected_objects",
                                                             "~observation_detected_objects")
        self.pubObservationDetectedObjects = rospy.Publisher(pubObservationDetectedObjectsTopic,
                                                             ObservationDetectedObjects, queue_size=32)

        self.snapMap.start()
        self.cartographer.start()
        self.localization.start()
        self.velocity.start()
        self.recovery.start()
        self.teleoperator.start()
        self.simpleMover.start()
        self.pathFollower.start()
        self.visualize.start()

        subKobukiBumperTopic = rospy.get_param("~sub_kobuki_bumper", "evt_bump")
        self.subKobukiBumper = rospy.Subscriber(subKobukiBumperTopic,
                                                BumperEvent,
                                                self.sub_kobuki_bumper)

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

        secondsPerUpdate = 1.0 / float(rospy.get_param("~update_rate", "10.0"))
        self.timer = rospy.Timer(rospy.Duration(secondsPerUpdate), self.update)

        self.started = True

    def reset(self):
        """ Reset all of the variables that change as the robot moves. """

        rospy.loginfo("Info[Controller.reset]: Resetting main controller.")

        self.currentAction = ActionType.NONE
        self.observationActionCompleteResult = ObservationActionComplete.NONE
        self.subactionQueue = list()

        self.snapMap.reset()
        self.cartographer.reset()
        self.localization.reset()
        self.velocity.reset()
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

    def update(self, msg):
        """ Perform an update at the rate of the timer.

            Parameters:
                msg     --  A TimerEvent object.
        """

        if not self.started:
            rospy.logwarn("Warn[Controller.update]: Initialization has not yet completed.")
            return

        startingAction = self.currentAction

        # Before anything, save the robot! Check if it needs to recover safely.
        if self.recovery.is_recovering(self.localization):
            self.recovery.perform_recovery(self.localization, self.velocity)

            if self.currentAction != ActionType.RECOVERY:
                self.currentAction = ActionType.RECOVERY
                if startingAction != ActionType.NONE:
                    self.observationActionCompleteResult = ObservationActionComplete.INTERRUPT
                else:
                    self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
                self.subactionQueue = [{"type": "reset"}]

        # Also, if not recovering, check if a teleoperator is trying to control the robot.
        elif self.teleoperator.is_activated():
            self.teleoperator.perform_teleoperation(self.localization, self.velocity)

            if self.currentAction != ActionType.TELEOPERATOR:
                self.currentAction = ActionType.TELEOPERATOR
                if startingAction != ActionType.NONE:
                    self.observationActionCompleteResult = ObservationActionComplete.INTERRUPT
                else:
                    self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
                self.subactionQueue = [{"type": "reset"}]

        # Otherwise, if we have subactions in our queue, then execute them accordingly.
        elif len(self.subactionQueue) > 0:
            subaction = self.subactionQueue[0]

            if subaction['type'] == "reset":
                self.velocity.reset()
                if self.simpleMover.has_goal():
                    self.simpleMover.reset()
                if self.pathFollower.has_path():
                    self.pathFollower.reset()
                self.subactionQueue.pop(0)

            elif subaction['type'] == "set":
                if subaction['param'] == "expecting bump":
                    self.recovery.set_expecting_bump(subaction['value'])
                self.subactionQueue.pop(0)

            elif subaction['type'] == "simple mover":
                if self.simpleMover.has_goal():
                    self.simpleMover.perform_simple_moving(self.localization, self.velocity)

                    if self.simpleMover.at_goal():
                        self.velocity.reset()
                        self.simpleMover.reset()
                        self.subactionQueue.pop(0)
                else:
                    if subaction['relative']:
                        self.simpleMover.set_goal_relative_heading(self.localization, subaction['heading'])
                    else:
                        self.simpleMover.set_goal_absolute_heading(self.localization, subaction['heading'])
                    self.simpleMover.set_goal_relative_distance(self.localization, subaction['distance'])

            elif subaction['type'] == "path follower":
                if self.pathFollower.has_goal():
                    self.pathFollower.perform_path_following(self.localization, self.velocity)

                    if self.pathFollower.at_goal():
                        self.velocity.reset()
                        self.pathFollower.reset()
                        self.subactionQueue.pop(0)
                else:
                    self.pathFollower.set_goals(subaction['goals'])

            # If there are no actions left in the queue, then we are done our current action.
            if len(self.subactionQueue) == 0:
                self.currentAction = ActionType.NONE

        # Publish the localization of the robot.
        self.localization.publish_localization()

        # Publish action completions/terminations along with the corresponding metadata (e.g., success/failure).
        if startingAction != self.currentAction:
            observation = ObservationActionComplete(startingAction, self.currentAction,
                                                    self.observationActionCompleteResult)
            self.pubObservationActionComplete.publish(observation)
            self.observationActionCompleteResult = ObservationActionComplete.SUCCESS

        # Publish visualizations, if enabled, such as the pose estimates, map regions/objects, and observed objects.
        self.visualize.publish_robot_pose_estimate(self.localization)
        self.visualize.publish_pose_estimate_history(self.localization)
        self.visualize.publish_regions(self.snapMap)
        self.visualize.publish_objects(self.snapMap)
        self.visualize.publish_scans(self.cartographer)

    def srv_action_move(self, request):
        """ Handle a service request for the move action.

            Parameters:
                request     --  The ActionMoveRequest object.

            Returns:
                The ActionMoveResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveResponse(self.currentAction)

        self.subactionQueue += [{'type': "simple mover", 'heading': request.heading,
                                 'relative': True, 'distance': request.distance}]

        self.currentAction = ActionType.MOVE
        self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
        return ActionMoveResponse(ActionType.NONE)

    def srv_action_move_in_grid(self, request):
        """ Handle a service request for the move in a grid action.

            Parameters:
                request     --  The ActionMoveInGridRequest object.

            Returns:
                The ActionMoveInGridResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionMoveInGridResponse(self.currentAction)

        desiredHeading = 0.0
        pi = float(np.pi)

        if request.action == ActionMoveInGridRequest.NORTH:
            desiredHeading = pi / 2.0
        elif request.action == ActionMoveInGridRequest.SOUTH:
            desiredHeading = -pi / 2.0
        elif request.action == ActionMoveInGridRequest.EAST:
            desiredHeading = 0.0
        elif request.action == ActionMoveInGridRequest.WEST:
            desiredHeading = pi
        elif request.action == ActionMoveInGridRequest.NORTH_EAST:
            desiredHeading = pi / 4.0
        elif request.action == ActionMoveInGridRequest.NORTH_WEST:
            desiredHeading = pi * 3.0 / 4.0
        elif request.action == ActionMoveInGridRequest.SOUTH_EAST:
            desiredHeading = -pi / 4.0
        elif request.action == ActionMoveInGridRequest.SOUTH_WEST:
            desiredHeading = -pi * 3.0 / 4.0

        self.subactionQueue += [{'type': "simple mover", 'heading': desiredHeading,
                                 'relative': False, 'distance': request.grid_cell_size}]

        self.currentAction = ActionType.MOVE_IN_GRID
        self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
        return ActionMoveInGridResponse(ActionType.NONE)

    def srv_action_navigate(self, request):
        """ Handle a service request for the navigate action.

            Parameters:
                request     --  The ActionNavigateRequest object.

            Returns:
                The ActionNavigateResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionNavigateResponse(self.currentAction)

        self.subactionQueue += [{'type': "path follower", 'goals': [Point(3.0, 2.0, 0.0)]}]
        #self.subactionQueue += [{'type': "path follower", 'goals': request.points}]

        self.currentAction = ActionType.NAVIGATE
        self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
        return ActionNavigateResponse(ActionType.NONE)

    def srv_action_navigate_to_region(self, request):
        """ Handle a service request for the navigate to a region action.

            Parameters:
                request     --  The ActionNavigateToRegionRequest object.

            Returns:
                The ActionNavigateToRegionResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionNavigateToRegionResponse(self.currentAction)

        # Basic: Assign goals to be just the center.
        #goals = [self.snapMap.get_center_of_region(request.region_uid)]
        # Random: Assign goals to be 16 random locations within the region.
        goals = [self.snapMap.get_random_point_in_region(request.region_uid) for i in range(16)]
        if None in goals:
            return ActionNavigateToRegionResponse(self.currentAction)

        self.subactionQueue += [{'type': "path follower", 'goals': goals}]

        self.currentAction = ActionType.NAVIGATE_TO_REGION
        self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
        return ActionNavigateToRegionResponse(ActionType.NONE)

    def srv_action_push(self, request):
        """ Handle a service request for the push action.

            Parameters:
                request     --  The ActionPushRequest object.

            Returns:
                The ActionPushResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionPushResponse(self.currentAction)

        self.subactionQueue += [{'type': "set", 'param': "expecting bump", 'value': True},
                                {'type': "simple mover", 'heading': 0.0, 'relative': True,
                                 'distance': request.lead_in_distance_before_contact},
                                {'type': "simple mover", 'heading': 0.0, 'relative': True,
                                 'distance': request.push_distance},
                                {'type': "simple mover", 'heading': 0.0, 'relative': True,
                                 'distance': -request.recover_distance_after_push},
                                {'type': "set", 'param': "expecting bump", 'value': False}]

        self.currentAction = ActionType.PUSH
        self.observationActionCompleteResult = ObservationActionComplete.FAILURE
        return ActionPushResponse(ActionType.NONE)

    def srv_action_align(self, request):
        """ Handle a service request for the align action.

            Parameters:
                request     --  The ActionAlignRequest object.

            Returns:
                The ActionAlignResponse object.
        """

        if self.currentAction is not ActionType.NONE:
            return ActionAlignResponse(self.currentAction)

        objectHeading = self.snapMap.get_object_heading(request.object_uid)
        if objectHeading is None:
            return ActionAlignResponse(self.currentAction)

        goalHeading = objectHeading + float(np.pi)
        if goalHeading > np.pi:
            goalHeading -= 2.0 * float(np.pi)

        goalPosition = self.snapMap.get_object_position(request.object_uid)
        goalPosition.x += float(request.distance_from_object * np.cos(objectHeading))
        goalPosition.y += float(request.distance_from_object * np.sin(objectHeading))

        self.subactionQueue += [{'type': "path follower", 'goals': [goalPosition]},
                                {'type': "simple mover", 'heading': goalHeading, 'relative': False, 'distance': 0.0}]

        self.currentAction = ActionType.ALIGN
        self.observationActionCompleteResult = ObservationActionComplete.SUCCESS
        return ActionAlignResponse(ActionType.NONE)

    def sub_kobuki_bumper(self, msg):
        """ This method checks for sensing a bump, used for determining action success/failure.

            Parameters:
                msg     --  The BumperEvent message data.
        """

        if self.currentAction == ActionType.PUSH:
            self.observationActionCompleteResult = ObservationActionComplete.SUCCESS

