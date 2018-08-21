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

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan

from ar_track_alvar_msgs.msg import AlvarMarkers

from snap.msg import LocalizationEstimate

import numpy as np


class Localization(object):
    """ A class for localizing within a known map with known regions and objects (e.g., AR tags). """

    def __init__(self, snapMap):
        """ The constructor for the Localization class.

            Parameters:
                snapMap     --  The SnapMap object to get map data (e.g., AR tags).
        """

        self.started = False

        self.snapMap = snapMap

        self.lastUpdateTime = None
        self.lastOdometryPositionEstimate = None
        self.lastOdometryHeadingEstimate = None

        self.positionEstimate = Point(0.0, 0.0, 0.0)
        self.headingEstimate = 0.0

        self.maxSpeedEstimates = int(rospy.get_param("~max_speed_estimates", "20"))
        self.speedEstimates = [0.0 for i in range(self.maxSpeedEstimates)]

        self.subKobukiOdometry = None
        self.subARTags = None

        self.pubLocalization = None

        #self.subDepthPointCloud = None
        #self.pubLaserScan = None

    def start(self):
        """ Start the necessary messages to create a map and localize. """

        if self.started:
            rospy.logwarn("Warn[Localization.start]: Already started.")
            return

        rospy.loginfo("Info[Localization.start]: Starting Localization sub-controller.")

        subKobukiOdometryTopic = rospy.get_param("~sub_kobuki_odometry", "odom")
        self.subKobukiOdometry = rospy.Subscriber(subKobukiOdometryTopic,
                                                  Odometry,
                                                  self.sub_kobuki_odometry)

        subARTagsTopic = rospy.get_param("~sub_ar_tags", "ar_pose_marker")
        self.subARTags = rospy.Subscriber(subARTagsTopic, AlvarMarkers, self.sub_ar_tags)

        pubLocalizationTopic = rospy.get_param("~pub_localization", "~localization")
        self.pubLocalization = rospy.Publisher(pubLocalizationTopic, LocalizationEstimate, queue_size=8)

        #subDepthPointCloudTopic = rospy.get_param("~sub_depth_point_cloud", "depth_point_cloud")
        #self.subDepthPointCloud = rospy.Subscriber(subDepthPointCloudTopic,
        #                                           PointCloud2,
        #                                           self.sub_depth_point_cloud)

        #pubLaserScanTopic = rospy.get_param("~pub_laser_scan", "scan")
        #self.pubLaserScan = rospy.Publisher(pubLaserScanTopic, LaserScan, queue_size=8)

        self.started = True

    def reset(self):
        """ Reset the Localization variables. """

        rospy.loginfo("Info[Localization.reset]: Resetting Localization sub-controller.")

        self.lastUpdateTime = None
        self.lastOdometryPositionEstimate = None
        self.lastOdometryHeadingEstimate = None

        self.positionEstimate = Point(0.0, 0.0, 0.0)
        self.headingEstimate = 0.0

        self.speedEstimates = [0.0 for i in range(self.maxSpeedEstimates)]

    def get_position_estimate(self):
        """ Get the current position estimate.

            Returns:
                The current position estimate as a Point object (x & y in meters).
        """

        return Point(self.positionEstimate.x, self.positionEstimate.y, 0.0)

    def get_speed_estimate(self):
        """ Get a speed estimate from the history of speed estimates.

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

        return float(self.headingEstimate)

    def _compute_heading_estimate(self, poseEstimate):
        """ Compute the heading estimate (in radians) from a pose estimate.

            Parameters:
                poseEstimate    --  The pose estimate as a Pose object.

            Returns:
                The estimate of the heading.
        """

        roll, pitch, yaw = euler_from_quaternion([poseEstimate.orientation.x,
                                                  poseEstimate.orientation.y,
                                                  poseEstimate.orientation.z,
                                                  poseEstimate.orientation.w])
        if yaw > np.pi:
            yaw -= 2.0 * float(np.pi)
        elif yaw < -np.pi:
            yaw += 2.0 * float(np.pi)

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
            rospy.logwarn("Warn[Localization.sub_kobuki_odometry]: Initialization has not yet completed.")
            return

        if self.lastUpdateTime is None or self.lastOdometryPositionEstimate is None:
            self.lastUpdateTime = rospy.get_rostime().to_sec()
            self.lastOdometryPositionEstimate = msg.pose.pose.position
            self.lastOdometryHeadingEstimate = self._compute_heading_estimate(msg.pose.pose)
            return

        currentTime = rospy.get_rostime().to_sec()
        deltaTime = currentTime - self.lastUpdateTime
        self.lastUpdateTime = currentTime

        # Compute the speed in meters per second, and only keep the last few speed estimates.
        # Also, throw out any outliers, namely if we get a message too quickly. This is perhaps
        # caused by two things publishing on the topic, or if a few things can cause a trigger.
        if deltaTime > 0.01 and deltaTime < 1.0:
            estimate = self._compute_speed_estimate(self.lastOdometryPositionEstimate,
                                                    msg.pose.pose.position, deltaTime)
            self.speedEstimates += [estimate]
            self.speedEstimates.pop(0)

        # Add to the position and heading based on the difference.
        self.positionEstimate.x += msg.pose.pose.position.x - self.lastOdometryPositionEstimate.x
        self.positionEstimate.y += msg.pose.pose.position.y - self.lastOdometryPositionEstimate.y

        self.headingEstimate += self._compute_heading_estimate(msg.pose.pose) - self.lastOdometryHeadingEstimate
        if self.headingEstimate > np.pi:
            self.headingEstimate -= 2.0 * np.pi
        elif self.headingEstimate < -np.pi:
            self.headingEstimate += 2.0 * np.pi

        self.lastOdometryPositionEstimate = msg.pose.pose.position
        self.lastOdometryHeadingEstimate = self._compute_heading_estimate(msg.pose.pose)

    def sub_ar_tags(self, msg):
        """ Update the localization using the AR tags known on the map.

            Parameters:
                msg     --  The AlvarMarkers message data.
        """

        if not self.started:
            rospy.logwarn("Warn[Localization.sub_ar_tags]: Initialization has not yet completed.")
            return

        newPositionEstimate = Point()
        newHeadingEstimate = 0.0
        numARTags = 0.0

        # Check all AR tags and average their data, if in the map, to get a pose for the robot.
        for marker in msg.markers:
            # Check if the observed AR tag is in the map. If not, continue.
            obj = self.snapMap.get_object(marker.id)
            if obj is None:
                continue

            rospy.loginfo("Info[Localization.sub_ar_tags]: Detected Marker ID %i - Found a match in the map!" % (marker.id))

            # Get the location in the map and offset it by the observed pose. This is the
            # observed estimate of the robot position.
            newPositionEstimate.x = ((numARTags * newPositionEstimate.x
                                      + (obj['position'][0] - marker.pose.pose.position.x))
                                     / (numARTags + 1))
            newPositionEstimate.y = ((numARTags * newPositionEstimate.y
                                      + (obj['position'][1] - marker.pose.pose.position.y))
                                     / (numARTags + 1))

            ## TODO: Also get the orientation's yaw of the AR tag and offset that to get a heading.
            markerHeading = self._compute_heading_estimate(marker.pose.pose)
            newHeadingEstimate = ((numARTags * newHeadingEstimate
                                   + (obj['heading'] - markerHeading))
                                  / (numARTags + 1))

            numARTags += 1.0

        if numARTags > 0.0:
            self.positionEstimate.x = newPositionEstimate.x
            self.positionEstimate.y = newPositionEstimate.y
            #self.headingEstimate = newHeadingEstimate

    def sub_depth_point_cloud(self, msg):
        """ Update the raw depth point cloud information, including sub-sampling and extracting abstract data.

            Parameters:
                msg     --  The raw PointCloud2 message data.
        """

        if not self.started:
            rospy.logwarn("Warn[Localization.sub_depth_point_cloud]: Initialization has not yet completed.")
            return

        # TODO: Take raw point cloud, find points at a height, populate a Laser whatever msg,
        # publish on map topic... Run mapping node in separate window. In rviz listen to the
        # map topic. See how it does at mapping... Update: It sucks.

        #fakeLaserScan = LaserScan()
        #fakeLaserScan.header = msg.header
        #fakeLaserScan.angle_min = -0.994838
        #fakeLaserScan.angle_max = 0.994838
        #fakeLaserScan.angle_increment = (0.994838 * 2.0) / 
        #fakeLaserScan.time_increment = 0.0
        #fakeLaserScan.scan_time = 1.0 / 30.0
        #fakeLaserScan.

        #self.pubLaserScan.publish(fakeLaserScan)

    def publish_localization(self):
        """ Publish the current localization estimates for the robot. """

        if not self.started:
            rospy.logwarn("Warn[Localization.publish_localization]: Initialization has not yet completed.")
            return

        localization = LocalizationEstimate()
        localization.position = self.get_position_estimate()
        localization.heading = self.get_heading_estimate()
        localization.speed = self.get_speed_estimate()
        regionEstimate = self.snapMap.get_region_by_point(localization.position)
        if regionEstimate is not None:
            localization.region_uid = int(regionEstimate['uid'])
        else:
            localization.region_uid = int(-1)

        self.pubLocalization.publish(localization)

