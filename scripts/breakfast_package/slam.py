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
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

import numpy as np


class SLAM(object):
    """ A class for creating a map and localizing within it (SLAM). """

    def __init__(self):
        """ The constructor for the SLAM class. """

        self.started = False

        self.lastUpdateTime = None
        self.poseEstimate = None
        self.maxSpeedEstimates = int(rospy.get_param("~max_speed_estimates", "20"))
        self.speedEstimates = [0.0 for i in range(self.maxSpeedEstimates)]

        self.subKobukiOdometry = None
        self.subDepthPointCloud = None
        self.pubLaserScan = None

    def start(self):
        """ Start the necessary messages to create a map and localize. """

        if self.started:
            rospy.logwarn("Warn[SLAM.start]: Already started.")
            return

        rospy.loginfo("Info[SLAM.start]: Starting SLAM sub-controller.")

        subKobukiOdometryTopic = rospy.get_param("~sub_kobuki_odometry", "odom")
        self.subKobukiOdometry = rospy.Subscriber(subKobukiOdometryTopic,
                                                  Odometry,
                                                  self.sub_kobuki_odometry)

        subDepthPointCloudTopic = rospy.get_param("~sub_depth_point_cloud", "depth_point_cloud")
        self.subDepthPointCloud = rospy.Subscriber(subDepthPointCloudTopic,
                                                   PointCloud2,
                                                   self.sub_depth_point_cloud)

        pubLaserScanTopic = rospy.get_param("~pub_laser_scan", "scan")
        self.pubLaserScan = rospy.Publisher(pubLaserScanTopic, LaserScan, queue_size=8)

        self.started = True

    def reset(self):
        """ Reset the SLAM variables. """

        rospy.loginfo("Info[SLAM.reset]: Resetting SLAM sub-controller.")

        self.lastUpdateTime = None
        self.poseEstimate = None
        self.speedEstimates = [0.0 for i in range(self.maxSpeedEstimates)]

    def get_pose_estimate(self):
        """ Return the current pose estimate (localization).

            Returns:
                The current pose estimate as a Pose object.
        """

        return self.poseEstimate

    def get_speed_estimate(self):
        """ Get a speed estimate from the history of pose estimates (localization).

            Returns:
                The current speed estimate as a signed float.
        """

        if len(self.speedEstimates) == 0:
            return 0.0

        return np.average(self.speedEstimates)

    def get_heading_estimate(self):
        """ Get the heading estimate from the current pose estimate (localization).

            Returns:
                The current heading estimate as a float in radians on [-pi, pi].
        """

        roll, pitch, yaw = euler_from_quaternion([self.poseEstimate.orientation.x,
                                                  self.poseEstimate.orientation.y,
                                                  self.poseEstimate.orientation.z,
                                                  self.poseEstimate.orientation.w])

        if yaw > np.pi:
            yaw -= 2.0 * np.pi
        elif yaw < -np.pi:
            yaw += 2.0 * np.pi

        return yaw

    def _compute_speed_estimate(self, oldPoint, newPoint, deltaTime):
        """ Compute the signed speed estimate given two positions and the time passed.

            Parameters:
                oldPoint    --  The old point as a Point object (x & y in meters).
                newPoint    --  The new point as a Point object (x & y in meters).
                deltaTime   --  The time difference between these two points (in seconds).

            Returns:
                The estimate of the speed.
        """

        a = Point(oldPoint.x, oldPoint.y, oldPoint.z)
        c = Point(newPoint.x, newPoint.y, newPoint.z)

        distanceTravelled = np.sqrt(pow(a.x - c.x, 2) + pow(a.y - c.y, 2))

        #thetaOldToNew = np.arctan2(c.y - a.y, c.x - a.x)
        heading = self.get_heading_estimate()
        b = Point(a.x + np.cos(heading - np.pi / 2.0),
                  a.y + np.sin(heading - np.pi / 2.0),
                  a.z)

        # Check if this new point (c) is left of the line formed by the old point (a) and a point
        # right of it (b). If it is, then it is moving forward; otherwise, it is moving backwards.
        isLeft = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))

        if isLeft >= 0.0:
            return distanceTravelled / deltaTime
        else:
            return -distanceTravelled / deltaTime

    def sub_kobuki_odometry(self, msg):
        """ Update the odometry information.

            Parameters:
                msg     --  The Odometry message data.
        """

        if not self.started:
            rospy.logwarn("Warn[SLAM.sub_kobuki_odometry]: Initialization has not yet completed.")
            return

        if self.lastUpdateTime is not None:
            currentTime = rospy.get_rostime().to_sec()
            deltaTime = currentTime - self.lastUpdateTime
            self.lastUpdateTime = currentTime

            # Compute the speed in meters per second, and only keep the last few speed estimates.
            # Also, throw out any outliers, namely if we get a message too quickly. This is perhaps
            # caused by two things publishing on the topic, or if a few things can cause a trigger.
            if deltaTime > 0.01 and deltaTime < 1.0:
                estimate = self._compute_speed_estimate(self.poseEstimate.position, msg.pose.pose.position, deltaTime)
                self.speedEstimates += [estimate]
                self.speedEstimates.pop(0)
        else:
            self.lastUpdateTime = rospy.get_rostime().to_sec()

        self.poseEstimate = msg.pose.pose

    def sub_depth_point_cloud(self, msg):
        """ Update the raw depth point cloud information, including sub-sampling and extracting abstract data.

            Parameters:
                msg     --  The raw PointCloud2 message data.
        """

        if not self.started:
            rospy.logwarn("Warn[SLAM.sub_depth_point_cloud]: Initialization has not yet completed.")
            return

        # TODO: Take raw point cloud, find points at a height, populate a Laser whatever msg, publish on map topic...
        # Run mapping node in separate window. In rviz listen to the map topic. See how it does at mapping...

        #fakeLaserScan = LaserScan()
        #fakeLaserScan.header = msg.header
        #fakeLaserScan.angle_min = -0.994838
        #fakeLaserScan.angle_max = 0.994838
        #fakeLaserScan.angle_increment = (0.994838 * 2.0) / 
        #fakeLaserScan.time_increment = 0.0
        #fakeLaserScan.scan_time = 1.0 / 30.0
        #fakeLaserScan.
        #fakeLaserScan.
        #fakeLaserScan.

        #self.pubLaserScan.publish(fakeLaserScan)

