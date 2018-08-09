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

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, PoseWithCovariance, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

import math
import numpy as np


class Visualize(object):
    """ Visualization of the 'snap' robot's state and behavior. """

    def __init__(self):
        """ The constructor for the Visualize class. """

        self.started = False

        self.poseEstimateHistory = list()
        self.lastPathPublishTime = rospy.get_rostime()

        self.publishRate = float(rospy.get_param("~pub_path_rate", "0.2"))

        self.mapFrameID = rospy.get_param("~map_frame_id", "map")

        self.visualizeAlpha = float(rospy.get_param("~visualize_alpha", "0.2"))

        self.pubRobotPose = None
        self.pubPath = None
        self.pubRegions = None
        self.pubObjects = None

    def start(self):
        """ Start the necessary messages for visualization. """

        if self.started:
            rospy.logwarn("Warn[Visualize.start]: Already started.")
            return

        rospy.loginfo("Info[Visualize.start]: Starting visualize sub-controller.")

        pubRobotPoseTopic = rospy.get_param("~pub_visualize_robot_pose", "/initialpose")
        self.pubRobotPose = rospy.Publisher(pubRobotPoseTopic, PoseWithCovarianceStamped, queue_size=32)

        pubPathTopic = rospy.get_param("~pub_visualize_path", "path")
        self.pubPath = rospy.Publisher(pubPathTopic, Path, queue_size=32)

        pubRegionsTopic = rospy.get_param("~pub_visualize_regions", "regions")
        self.pubRegions = rospy.Publisher(pubRegionsTopic, MarkerArray, queue_size=32)

        pubObjectsTopic = rospy.get_param("~pub_visualize_objects", "objects")
        self.pubObjects = rospy.Publisher(pubObjectsTopic, MarkerArray, queue_size=32)

        self.started = True

    def reset(self):
        """ Reset the visualize variables. """

        rospy.loginfo("Info[Visualize.reset]: Resetting visualize sub-controller.")

        self.poseEstimateHistory = list()
        self.lastPathPublishTime = rospy.get_rostime()

    def publish_robot_pose_estimate(self, localization):
        """ Publish the pose estimate for the robot using its tf.

            Parameters:
                localization    --  The Localization object, which contains position and heading estimates.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_regions]: Visualize has not yet been started.")
            return

        poseWithCovStamped = PoseWithCovarianceStamped()
        poseWithCovStamped.header.frame_id = self.mapFrameID
        poseWithCovStamped.header.stamp = rospy.get_rostime()

        poseWithCovStamped.pose = PoseWithCovariance()
        poseWithCovStamped.pose.pose.position = localization.get_position_estimate()

        v = quaternion_from_euler(0.0, 0.0, localization.get_heading_estimate())
        poseWithCovStamped.pose.pose.orientation = Quaternion(v[0], v[1], v[2], v[3])

        self.pubRobotPose.publish(poseWithCovStamped)

    def publish_pose_estimate_history(self, localization):
        """ Record the path taken, but only at a certain rate.

            Parameters:
                localization    --  The Localization object, which contains position and heading estimates.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_path]: Visualize has not yet been started.")
            return

        currentTime = rospy.get_rostime()

        if self.lastPathPublishTime.to_sec() + self.publishRate <= currentTime.to_sec():
            position = localization.get_position_estimate()

            if len(self.poseEstimateHistory) > 0:
                distanceTravelled = float(math.sqrt(pow(self.poseEstimateHistory[-1].pose.position.x - position.x, 2)
                                                    + pow(self.poseEstimateHistory[-1].pose.position.y - position.y, 2)))

            # Only consider adding the new pose if there is a large enough difference in location (>= 0.1 meters).
            if len(self.poseEstimateHistory) == 0 or distanceTravelled >= 0.1:
                poseStamped = PoseStamped()
                poseStamped.header.frame_id = self.mapFrameID
                poseStamped.header.stamp = currentTime
                poseStamped.pose = Pose()
                poseStamped.pose.position = position

                self.poseEstimateHistory += [poseStamped]

            # Create and publish the path.
            path = Path()
            path.header.frame_id = self.mapFrameID
            path.header.stamp = currentTime
            path.poses = self.poseEstimateHistory

            self.pubPath.publish(path)

            self.lastPathPublishTime = currentTime

    def publish_regions(self, cartographer):
        """ Publish the region locations in the map.

            Parameters:
                cartographer    --  The Cartographer object that contains map object data.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_regions]: Visualize has not yet been started.")
            return

        regionMarkers = MarkerArray()

        for region in cartographer.get_regions():
            if len(region['bounds']) < 3:
                continue

            regionMarker = Marker()
            regionMarker.header.frame_id = self.mapFrameID
            regionMarker.header.stamp = rospy.get_rostime()
            regionMarker.ns = "~"
            regionMarker.id = 1000 + region['uid']
            regionMarker.type = Marker.TRIANGLE_LIST
            regionMarker.action = Marker.ADD
            regionMarker.pose.position.x = 0.0
            regionMarker.pose.position.y = 0.0
            regionMarker.pose.position.z = 0.0
            regionMarker.pose.orientation.x = 0.0
            regionMarker.pose.orientation.y = 0.0
            regionMarker.pose.orientation.z = 0.0
            regionMarker.pose.orientation.w = 1.0
            regionMarker.scale.x = 1.0
            regionMarker.scale.y = 1.0
            regionMarker.scale.z = 1.0
            regionMarker.color.a = 1.0
            regionMarker.color.r = 1.0
            regionMarker.color.g = 1.0
            regionMarker.color.b = 1.0

            centerOfRegion = Point(sum([p[0] for p in region['bounds']]) / len(region['bounds']),
                                   sum([p[1] for p in region['bounds']]) / len(region['bounds']), 0.0)
            colorOfRegion = ColorRGBA(region['color'][0], region['color'][1], region['color'][2], self.visualizeAlpha)

            for i in range(len(region['bounds']) - 1):
                regionMarker.points += [Point(region['bounds'][i][0], region['bounds'][i][1], 0.0),
                                        Point(region['bounds'][i + 1][0], region['bounds'][i + 1][1], 0.0),
                                        centerOfRegion]
                regionMarker.colors += [colorOfRegion for i in range(3)]
            regionMarker.points += [Point(region['bounds'][-1][0], region['bounds'][-1][1], 0.0),
                                    Point(region['bounds'][0][0], region['bounds'][0][1], 0.0),
                                    centerOfRegion]
            regionMarker.colors += [colorOfRegion for i in range(3)]

            regionMarkers.markers += [regionMarker]

        self.pubRegions.publish(regionMarkers)

    def publish_objects(self, cartographer):
        """ Publish the object locations in the map.

            Parameters:
                cartographer    --  The Cartographer object that contains map object data.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_objects]: Visualize has not yet been started.")
            return

        objectMarkers = MarkerArray()

        for obj in cartographer.get_objects():
            objectMarker = Marker()
            objectMarker.header.frame_id = self.mapFrameID
            objectMarker.header.stamp = rospy.get_rostime()
            objectMarker.ns = "~"
            objectMarker.id = 10000 + obj['uid']
            objectMarker.type = Marker.TRIANGLE_LIST
            objectMarker.action = Marker.ADD
            objectMarker.pose.position.x = 0.0
            objectMarker.pose.position.y = 0.0
            objectMarker.pose.position.z = 0.0
            objectMarker.pose.orientation.x = 0.0
            objectMarker.pose.orientation.y = 0.0
            objectMarker.pose.orientation.z = 0.0
            objectMarker.pose.orientation.w = 1.0
            objectMarker.scale.x = 1.0
            objectMarker.scale.y = 1.0
            objectMarker.scale.z = 1.0
            objectMarker.color.a = 1.0
            objectMarker.color.r = 1.0
            objectMarker.color.g = 1.0
            objectMarker.color.b = 1.0

            colorOfObject = ColorRGBA(obj['color'][0], obj['color'][1], obj['color'][2], self.visualizeAlpha)

            objectMarker.points += [Point(obj['position'][0] + 0.2 * np.cos(obj['heading'] - np.pi / 2.0),
                                          obj['position'][1] + 0.2 * np.sin(obj['heading'] - np.pi / 2.0), 0.2)]
            objectMarker.points += [Point(obj['position'][0] + 0.2 * np.cos(obj['heading']),
                                          obj['position'][1] + 0.2 * np.sin(obj['heading']), 0.2)]
            objectMarker.points += [Point(obj['position'][0] + 0.2 * np.cos(obj['heading'] + np.pi / 2.0),
                                          obj['position'][1] + 0.2 * np.sin(obj['heading'] + np.pi / 2.0), 0.2)]
            objectMarker.colors += [colorOfObject for i in range(3)]

            objectMarkers.markers += [objectMarker]

        self.pubObjects.publish(objectMarkers)

