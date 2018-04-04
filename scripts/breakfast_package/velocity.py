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

from tf.transformations import euler_from_quaternion

import numpy as np


class Velocity(object):
    """ A class to determine the velocity (speed and heading). """

    def __init__(self):
        """ The constructor for the Velocity class. """

        self.started = False

        self.speedProportional = 0.0
        self.speedIntegral = 0.0
        self.speedDerivative = 0.0

        self.speedProportionalGain = float(rospy.get_param("~speed_proportional_gain", "0.5"))
        self.speedIntegralGain = float(rospy.get_param("~speed_integral_gain", "0.01"))
        self.speedDerivativeGain = float(rospy.get_param("~speed_derivative_gain", "0.15"))

        self.headingProportional = 0.0
        self.headingIntegral = 0.0
        self.headingDerivative = 0.0

        self.headingProportionalGain = float(rospy.get_param("~heading_proportional_gain", "0.5"))
        self.headingIntegralGain = float(rospy.get_param("~heading_integral_gain", "0.05"))
        self.headingDerivativeGain = float(rospy.get_param("~heading_derivative_gain", "0.3"))

        self.debugTuneGains = None # None, "speed", or "heading"

    def start(self):
        """ Start the necessary messages and initialize variables. """

        if self.started:
            rospy.logwarn("Warn[Velocity.start]: Already started.")
            return

        rospy.loginfo("Info[Velocity.start]: Starting Velocity sub-controller.")

        # Note: Put any message-starting code here.

        self.started = True

    def reset(self):
        """ Reset the Velocity variables. """

        rospy.loginfo("Info[Velocity.reset]: Resetting Velocity sub-controller.")

        self.speedProportional = 0.0
        self.speedIntegral = 0.0
        self.speedDerivative = 0.0

        self.headingProportional = 0.0
        self.headingIntegral = 0.0
        self.headingDerivative = 0.0

    def compute_speed(self, localization, desiredSpeed):
        """ Compute the speed from the PID controller. Update the PID as well.

            Parameters:
                localization            --  The Localization object, which contains position and heading estimates.
                desiredSpeed    --  The desired signed speed in meters per second.

            Returns:
                The speed to assign following the PID controller.
        """

        if type(desiredSpeed) is not float:
            rospy.logwarn("Warning[Velocity.compute_speed]: Desired speed is not a float!")
            desiredSpeed = localization.get_speed_estimate()

        if self.debugTuneGains == "speed":
            desiredSpeed = 1.0

        error = desiredSpeed - localization.get_speed_estimate()

        self.speedIntegral += error
        self.speedDerivative = error - self.speedProportional
        self.speedProportional = error

        integralDecay = 0.95
        self.speedProportional = np.clip(self.speedProportional, -100.0, 100.0)
        self.speedIntegral = np.clip(self.speedIntegral * integralDecay, -100.0, 100.0)
        self.speedDerivative = np.clip(self.speedDerivative, -100.0, 100.0)

        result = (self.speedProportionalGain * self.speedProportional
                  + self.speedIntegralGain * self.speedIntegral
                  + self.speedDerivativeGain * self.speedDerivative)

        if self.debugTuneGains == "speed":
            rospy.loginfo("Info[Velocity.compute_speed]: %s"
                          % ("Speed: [ Proportional = %.3f - %.3f = %.3f  Integral = %.3f  Derivative: %.3f ]"
                             % (desiredSpeed, localization.get_speed_estimate(), error,
                                self.speedIntegral, self.speedDerivative)))
        elif self.debugTuneGains == "heading":
            result = 0.4

        return result

    def compute_heading(self, localization, desiredHeading):
        """ Get the heading from the PID controller. Update the PID as well.

            Parameters:
                localization            --  The Localization object, which contains position and heading estimates.
                desiredHeading  --  The desired heading in radians on [-pi, pi].

            Returns:
                The heading to assign following the PID controller.
        """

        if type(desiredHeading) is not float:
            rospy.logwarn("Warning[Velocity.compute_heading]: Desired heading is not a float!")
            desiredHeading = localization.get_heading_estimate()

        if self.debugTuneGains == "heading":
            desiredHeading = float(np.pi) / 2.0

        error = desiredHeading - localization.get_heading_estimate()
        if abs(error) > np.pi:
            if error >= 0.0:
                error = desiredHeading - (localization.get_heading_estimate() + float(np.pi) * 2.0)
            elif error < 0.0:
                error = (desiredHeading + float(np.pi) * 2.0) - localization.get_heading_estimate()

        self.headingIntegral += error
        self.headingDerivative = error - self.headingProportional
        self.headingProportional = error

        integralDecay = 0.95
        self.headingProportional = np.clip(self.headingProportional, -100.0, 100.0)
        self.headingIntegral = np.clip(self.headingIntegral * integralDecay, -100.0, 100.0)
        self.headingDerivative = np.clip(self.headingDerivative, -100.0, 100.0)

        result = (self.headingProportionalGain * self.headingProportional
                  + self.headingIntegralGain * self.headingIntegral
                  + self.headingDerivativeGain * self.headingDerivative)

        if self.debugTuneGains == "speed":
            result = 0.0
        elif self.debugTuneGains == "heading":
            rospy.loginfo("Info[Velocity.compute_heading]: %s"
                          % ("Heading: [ Proportional = %.3f - %.3f = %.3f  Integral = %.3f  Derivative: %.3f ]"
                             % (desiredHeading, localization.get_heading_estimate(), error,
                                self.headingIntegral, self.headingDerivative)))

        return result

