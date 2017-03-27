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


class Recovery(object)
    """ A recovery mechanism to handle unexpected bumps, senses cliffs, or wheel drops. """

    def __init__(self):
        """ The constructor for the Recovery class. """

        self.started = False
        self.recovery = False
        self.recoveryStartTime = rospy.get_rostime()

        self.wheelDrop = [False, False]

        self.desiredVelocity = rospy.get_param(rospy.search_param('desired_velocity'))

        self.subKobukiBumper = None
        self.subKobukiCliff = None
        self.subKobukiWheelDrop = None

        self.pubKobukiVel = None

    def start(self):
        """ Start the necessary messages for recovery. """

        subKobukiBumperTopic = rospy.get_param(rospy.search_param('sub_kobuki_bumper'))
        self.subKobukiBumper = rospy.Subscriber(subKobukiBumperTopic,
                                                BumperEvent,
                                                self.sub_kobuki_bumper)

        subKobukiCliffTopic = rospy.get_param(rospy.search_param('sub_kobuki_cliff'))
        self.subKobukiCliff = rospy.Subscriber(subKobukiCliffTopic,
                                               CliffEvent,
                                               self.sub_kobuki_cliff)

        subKobukiWheelDropTopic = rospy.get_param(rospy.search_param('sub_kobuki_wheel_drop'))
        self.subKobukiWheelDrop = rospy.Subscriber(subKobukiWheelDropTopic,
                                                   WheelDropEvent,
                                                   self.sub_kobuki_wheel_drop)

        pubKobukiVelocityTopic = rospy.get_param(rospy.search_param('pub_kobuki_velocity'))
        self.pubKobukiVelocity = rospy.Publisher(pubKobukiVelocityTopic, Twist, queue_size=32)

    def check_recovery(self):
        """ Check recovery, returning True if a recovery is necessary, and False if it is not.

            Returns:
                True if a recovery is necessary, and False if it is not.
        """

        if not self.recovery:
            return False

        # Now we check for wall and cliff recovery. We will only perform the recovery movement
        # as long as the recovery duration. Afterwards, we terminate recovery.
        currentTime = rospy.get_rostime()

        if self.recoveryStartTime.to_sec() + self.recoveryDuration <= currentTime.to_sec():
            self.recovery = False
            return False

        return True

    def check_wheel_drop(self):
        """ Check if a wheel drop is occurring.

            Returns:
                True if a wheel drop is occurring, and False otherwise.
        """

        return self.wheelDrop[0] or self.wheelDrop[1]

    def move_recovery(self):
        """ Move away from a wall or cliff backwards using the relevant Kobuki messages. """

        control = Twist()
        control.linear.x = -self.desiredVelocity

        self.pubKobukiVelocity.publish(control)

    def sub_kobuki_bumper(self, msg):
        """ This method checks for sensing a bump.

            Parameters:
                msg     --  The BumperEvent message data.
        """

        if self.wheel_drop():
            return

        if msg.state == BumperEvent.PRESSED:
            self.recovery = True
            self.recoveryStartTime = rospy.get_rostime()

    def sub_kobuki_cliff(self, msg):
        """ This method checks for sensing a cliff.

            Parameters:
                msg     --  The CliffEvent message data.
        """

        if self.wheel_drop():
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

        if self.wheel_drop():
            self.recovery = False

