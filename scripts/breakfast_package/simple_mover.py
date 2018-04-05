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

from geometry_msgs.msg import Twist

import numpy as np


class SimpleMover(object):
    """ A class for simple heading with distance movement. """

    def __init__(self):
        """ The constructor for the SimpleMover class. """

        self.started = False
        self.moving = False
        self.atGoal = False

        self.startHeading = None
        self.startPosition = None
        self.goalHeading = None
        self.goalDistance = None

        self.maxSimpleMoverSpeed = float(rospy.get_param("~max_simple_mover_speed", "0.3"))
        self.maxSimpleMoverHeading = float(rospy.get_param("~max_simple_mover_heading", str(np.pi)))

        self.pubKobukiVelocity = None

    def start(self):
        """ Start the necessary messages for simple movement. """

        if self.started:
            rospy.logwarn("Warn[SimpleMover.start]: Already started.")
            return

        rospy.loginfo("Info[SimpleMover.start]: Starting simple mover sub-controller.")

        pubKobukiVelocityTopic = rospy.get_param("~pub_kobuki_velocity", "cmd_vel")
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the simple mover variables. """

        rospy.loginfo("Info[SimpleMover.reset]: Resetting simple mover sub-controller.")

        self.moving = False
        self.atGoal = False

        self.startHeading = None
        self.startPosition = None
        self.goalHeading = None
        self.goalDistance = None

    def set_goal_absolute_heading(self, localization, desiredAbsoluteHeading):
        """ Set the desired absolute heading. This causes the simple move to begin moving.

            Parameters:
                localization            --  The Localization object, which contains position and heading estimates.
                desiredAbsoluteHeading  --  The desired absolute heading in radians on [-pi, pi].
        """

        desiredRelativeHeading = desiredAbsoluteHeading - localization.get_heading_estimate()
        if desiredRelativeHeading > np.pi:
            desiredRelativeHeading -= 2.0 * float(np.pi)
        elif desiredRelativeHeading < -np.pi:
            desiredRelativeHeading += 2.0 * float(np.pi)

        self.set_goal_relative_heading(localization, desiredRelativeHeading)

    def set_goal_relative_heading(self, localization, desiredRelativeHeading):
        """ Set the desired relative heading. This causes the simple mover to begin moving.

            Parameters:
                localization            --  The Localization object, which contains position and heading estimates.
                desiredRelativeHeading  --  The desired relative heading in radians on [-pi, pi].
        """

        if abs(desiredRelativeHeading) > float(np.pi):
            desiredRelativeHeading = self.maxSimpleMoverHeading

        self.startHeading = localization.get_heading_estimate()

        self.goalHeading = self.startHeading + desiredRelativeHeading
        if self.goalHeading > np.pi:
            self.goalHeading -= 2.0 * float(np.pi)
        elif self.goalHeading < -np.pi:
            self.goalHeading += 2.0 * float(np.pi)

        self.moving = True

    def set_goal_relative_distance(self, localization, desiredRelativeDistance):
        """ Set the desired relative signed distance. This causes the simple mover to begin moving.

            Parameters:
                localization                --  The Localization object, which contains position and heading estimates.
                desiredRelativeDistance     --  The desired relative signed distance in meters.
        """

        self.startPosition = localization.get_position_estimate()
        self.goalDistance = desiredRelativeDistance
        self.moving = True

    def at_goal(self):
        """ Determine if we are at the goal or not.

            Returns:
                True if we are at the goal, False otherwise.
        """

        return self.atGoal

    def has_goal(self):
        """ Determine if a goal heading and/or distance is set or not.

            Returns:
                True if a goal exists, False otherwise.
        """

        return self.goalHeading is not None or self.goalDistance is not None

    def perform_simple_moving(self, localization, velocity):
        """ Perform simple moving control adjustments, sending Twist messages to the Kobuki.

            Parameters:
                localization    --  The Localization object, which contains position and heading estimates.
                velocity        --  The Velocity object, which is a speed/heading PID controller.
        """

        # If there is no goal set, then publish empty.
        if localization is None or not self.has_goal():
            self.pubKobukiVelocity.publish(Twist())
            return

        control = Twist()

        # First, get the current heading, position, and distance.
        headingEstimate = localization.get_heading_estimate()
        positionEstimate = localization.get_position_estimate()
        distanceFromStart = float(np.sqrt(pow(self.startPosition.x - positionEstimate.x, 2)
                                          + pow(self.startPosition.y - positionEstimate.y, 2)))

        desiredSpeed = self.maxSimpleMoverSpeed
        if np.sign(self.goalDistance) > 0.0:
            desiredSpeed *= float(np.clip(abs(self.goalDistance - distanceFromStart), 0.0, 1.0))
        else:
            desiredSpeed *= -float(np.clip(abs(-self.goalDistance - distanceFromStart), 0.0, 1.0))

        # First check if we need to correct the heading.
        if abs(self.goalHeading - headingEstimate) > 0.05:
            control.angular.z = velocity.compute_heading(localization, self.goalHeading)

        # Given the heading is correct, we check the distance next to move the desired distance.
        elif abs(abs(self.goalDistance) - distanceFromStart) > 0.1:
            control.linear.x = localization.get_speed_estimate() + velocity.compute_speed(localization, desiredSpeed)

        # If both heading and distance are at the goal, then we are done!
        else:
            self.atGoal = True

        # Whatever the control ends up as, including empty, we publish it.
        self.pubKobukiVelocity.publish(control)


