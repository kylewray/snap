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
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import CliffEvent
from kobuki_msgs.msg import WheelDropEvent


class Recovery(object):
    """ A recovery mechanism to handle unexpected bumps, senses cliffs, or wheel drops. """

    def __init__(self):
        """ The constructor for the Recovery class. """

        self.started = False
        self.recovery = False
        self.recoveryStartTime = rospy.get_rostime()

        self.wheelDrop = [False, False]

        self.recoveryDuration = float(rospy.get_param(rospy.search_param("recovery_duration"), "0.5"))
        self.desiredVelocity = float(rospy.get_param(rospy.search_param('desired_velocity'), "0.2"))

        self.subKobukiBumper = None
        self.subKobukiCliff = None
        self.subKobukiWheelDrop = None

        self.pubKobukiVel = None

    def start(self):
        """ Start the necessary messages for recovery. """

        subKobukiBumperTopic = rospy.get_param(rospy.search_param('sub_kobuki_bumper'), "evt_bump")
        self.subKobukiBumper = rospy.Subscriber(subKobukiBumperTopic,
                                                BumperEvent,
                                                self.sub_kobuki_bumper)

        subKobukiCliffTopic = rospy.get_param(rospy.search_param('sub_kobuki_cliff'), "evt_cliff")
        self.subKobukiCliff = rospy.Subscriber(subKobukiCliffTopic,
                                               CliffEvent,
                                               self.sub_kobuki_cliff)

        subKobukiWheelDropTopic = rospy.get_param(rospy.search_param('sub_kobuki_wheel_drop'),
                                                  "evt_wheel_drop")
        self.subKobukiWheelDrop = rospy.Subscriber(subKobukiWheelDropTopic,
                                                   WheelDropEvent,
                                                   self.sub_kobuki_wheel_drop)

        pubKobukiVelocityTopic = rospy.get_param(rospy.search_param('pub_kobuki_velocity'))
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

        self.started = True

    def is_recovering(self):
        """ Check recovery, returning True if a recovery is necessary, and False if it is not.

            Returns:
                True if a recovery is necessary, and False if it is not.
        """

        # Special: If the wheels are dropped, then we are still "recovering", and we will not move.
        if self.are_wheels_dropped():
            return True

        if not self.recovery:
            return False

        # Now we check for wall and cliff recovery. We will only perform the recovery movement
        # as long as the recovery duration. Afterwards, we terminate recovery.
        currentTime = rospy.get_rostime()

        if self.recoveryStartTime.to_sec() + self.recoveryDuration <= currentTime.to_sec():
            self.recovery = False
            return False

        return True

    def are_wheels_dropped(self):
        """ Check if a wheel drop is occurring.

            Returns:
                True if a wheel drop is occurring, and False otherwise.
        """

        return self.wheelDrop[0] or self.wheelDrop[1]

    def perform_recovery(self):
        """ Move away from a wall or cliff backwards using the relevant Kobuki messages. """

        if not self.started:
            rospy.logwarn("Warn[Recovery.perform_recovery]: Initialization has not yet completed.")
            return

        # Special: If the wheels are dropped, then do not move.
        if self.are_wheels_dropped():
            control = Twist()
            self.pubKobukiVelocity.publish(control)
            return

        # Otherwise, we perform a basic recovery by moving backwards for a short time.
        control = Twist()
        control.linear.x = -self.desiredVelocity

        self.pubKobukiVelocity.publish(control)

    def sub_kobuki_bumper(self, msg):
        """ This method checks for sensing a bump.

            Parameters:
                msg     --  The BumperEvent message data.
        """

        if self.are_wheels_dropped():
            return

        if msg.state == BumperEvent.PRESSED:
            self.recovery = True
            self.recoveryStartTime = rospy.get_rostime()

    def sub_kobuki_cliff(self, msg):
        """ This method checks for sensing a cliff.

            Parameters:
                msg     --  The CliffEvent message data.
        """

        if self.are_wheels_dropped():
            return

        if msg.state == CliffEvent.CLIFF:
            self.recovery = True
            self.recoveryStartTime = rospy.get_rostime()

    def sub_kobuki_wheel_drop(self, msg):
        """ This method checks for sensing a wheel drop.

            Parameters:
                msg     --  The BumperEvent message data.
        """

        self.wheelDrop[int(msg.wheel)] = bool(msg.state)

        if self.are_wheels_dropped():
            self.recovery = False

