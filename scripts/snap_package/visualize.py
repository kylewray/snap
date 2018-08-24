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
        self.visualizeScanSubsample = int(rospy.get_param("~visualize_scan_subsample", "1"))

        self.pubRobotPose = None
        self.pubPath = None
        self.pubRegions = None
        self.pubObjects = None
        self.pubScans = None

    def start(self):
        """ Start the necessary messages for visualization. """

        if self.started:
            rospy.logwarn("Warn[Visualize.start]: Already started.")
            return

        rospy.loginfo("Info[Visualize.start]: Starting visualize sub-controller.")

        pubRobotPoseTopic = rospy.get_param("~pub_visualize_robot_pose", "/initialpose")
        self.pubRobotPose = rospy.Publisher(pubRobotPoseTopic, PoseWithCovarianceStamped, queue_size=32)

        pubPathTopic = rospy.get_param("~pub_visualize_path", "~/visualize/path")
        self.pubPath = rospy.Publisher(pubPathTopic, Path, queue_size=32)

        pubRegionsTopic = rospy.get_param("~pub_visualize_regions", "~/visualize/regions")
        self.pubRegions = rospy.Publisher(pubRegionsTopic, MarkerArray, queue_size=32)

        pubObjectsTopic = rospy.get_param("~pub_visualize_objects", "~/visualize/objects")
        self.pubObjects = rospy.Publisher(pubObjectsTopic, MarkerArray, queue_size=32)

        pubScansTopic = rospy.get_param("~pub_visualize_scans", "~/visualize/scans")
        self.pubScans = rospy.Publisher(pubScansTopic, MarkerArray, queue_size=32)

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

    def publish_regions(self, snapMap):
        """ Publish the region locations in the map.

            Parameters:
                snapMap     --  The SnapMap object that contains map object data.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_regions]: Visualize has not yet been started.")
            return

        regionMarkers = MarkerArray()

        for region in snapMap.get_regions():
            if len(region['bounds']) < 3:
                continue

            regionMarker = Marker()
            regionMarker.header.frame_id = self.mapFrameID
            regionMarker.header.stamp = rospy.get_rostime()
            regionMarker.ns = "~"
            regionMarker.id = 100 + region['uid']
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

    def publish_objects(self, snapMap):
        """ Publish the object locations in the map.

            Parameters:
                snapMap     --  The SnapMap object that contains map object data.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_objects]: Visualize has not yet been started.")
            return

        objectMarkers = MarkerArray()

        for obj in snapMap.get_objects():
            objectMarker = Marker()
            objectMarker.header.frame_id = self.mapFrameID
            objectMarker.header.stamp = rospy.get_rostime()
            objectMarker.ns = "~"
            objectMarker.id = 1000 + obj['uid']
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

    def publish_scans(self, cartographer):
        """ Publish the scans (both laser and object) for mapping and map correcting.

            Parameters:
                cartographer    --  The Cartographer object that contains new scan data.
        """

        if not self.started:
            rospy.logwarn("Warn[Visualize.publish_scans]: Visualize has not yet been started.")
            return

        scanMarkers = MarkerArray()

        for scanUID, scan in enumerate(cartographer.get_scans()):
            # Create the pose marker for the scan.
            x = scan['pose']['x']
            y = scan['pose']['y']
            heading = scan['pose']['heading']
            cosH = math.cos(heading)
            sinH = math.sin(heading)

            if scanUID == cartographer.get_correcting_current_scan_index():
                scaleOfPose = 0.2
                colorOfPose = ColorRGBA(1.0, 1.0, 0.0, self.visualizeAlpha)
            else:
                scaleOfPose = 0.1
                colorOfPose = ColorRGBA(0.0, 0.0, 0.0, self.visualizeAlpha)

            poseMarker = Marker()
            poseMarker.header.frame_id = self.mapFrameID
            poseMarker.header.stamp = rospy.get_rostime()
            poseMarker.ns = "~"
            poseMarker.id = 10000 + scanUID * 3 + 0
            poseMarker.type = Marker.TRIANGLE_LIST
            poseMarker.action = Marker.ADD
            poseMarker.pose.position.x = 0.0
            poseMarker.pose.position.y = 0.0
            poseMarker.pose.position.z = 0.0
            poseMarker.pose.orientation.x = 0.0
            poseMarker.pose.orientation.y = 0.0
            poseMarker.pose.orientation.z = 0.0
            poseMarker.pose.orientation.w = 1.0
            poseMarker.scale.x = 1.0
            poseMarker.scale.y = 1.0
            poseMarker.scale.z = 1.0
            poseMarker.color.a = 1.0
            poseMarker.color.r = 1.0
            poseMarker.color.g = 1.0
            poseMarker.color.b = 1.0

            poseMarker.points += [Point(x + scaleOfPose * np.cos(heading - np.pi / 2.0),
                                        y + scaleOfPose * np.sin(heading - np.pi / 2.0), 0.2)]
            poseMarker.points += [Point(x + scaleOfPose * np.cos(heading),
                                        y + scaleOfPose * np.sin(heading), 0.2)]
            poseMarker.points += [Point(x + scaleOfPose * np.cos(heading + np.pi / 2.0),
                                        y + scaleOfPose * np.sin(heading + np.pi / 2.0), 0.2)]
            poseMarker.colors = [colorOfPose for i in range(3)]

            scanMarkers.markers += [poseMarker]

            # Create the laser scan markers.
            if scanUID == cartographer.get_correcting_current_scan_index():
                scaleOfPose = 0.1
                colorOfLaserScan = ColorRGBA(1.0, 1.0, 0.0, self.visualizeAlpha)
            else:
                scaleOfPose = 0.05
                colorOfLaserScan = ColorRGBA(0.0, 1.0, 0.0, self.visualizeAlpha)

            laserScanMarker = Marker()
            laserScanMarker.header.frame_id = self.mapFrameID
            laserScanMarker.header.stamp = rospy.get_rostime()
            laserScanMarker.ns = "~"
            laserScanMarker.id = 10000 + scanUID * 3 + 1
            laserScanMarker.type = Marker.POINTS
            laserScanMarker.action = Marker.ADD
            laserScanMarker.pose.position.x = 0.0
            laserScanMarker.pose.position.y = 0.0
            laserScanMarker.pose.position.z = 0.0
            laserScanMarker.pose.orientation.x = 0.0
            laserScanMarker.pose.orientation.y = 0.0
            laserScanMarker.pose.orientation.z = 0.0
            laserScanMarker.pose.orientation.w = 1.0
            laserScanMarker.scale.x = scaleOfPose
            laserScanMarker.scale.y = scaleOfPose
            laserScanMarker.scale.z = scaleOfPose
            laserScanMarker.color.a = 1.0
            laserScanMarker.color.r = 1.0
            laserScanMarker.color.g = 1.0
            laserScanMarker.color.b = 1.0

            colorOfLaserScan = ColorRGBA(0.0, 1.0, 0.0, self.visualizeAlpha)

            for i in range(0, len(scan['points']), self.visualizeScanSubsample):
                ls = scan['points'][i]
                lsX = x + ls['x'] * cosH - ls['y'] * sinH
                lsY = y + ls['x'] * sinH + ls['y'] * cosH
                laserScanMarker.points += [Point(lsX, lsY, 0.2)]
            laserScanMarker.colors = [colorOfLaserScan for i in range(len(laserScanMarker.points))]

            scanMarkers.markers += [laserScanMarker]

            # Create the object scan markers.
            objectScanMarker = Marker()
            objectScanMarker.header.frame_id = self.mapFrameID
            objectScanMarker.header.stamp = rospy.get_rostime()
            objectScanMarker.ns = "~"
            objectScanMarker.id = 10000 + scanUID * 3 + 2
            objectScanMarker.type = Marker.TRIANGLE_LIST
            objectScanMarker.action = Marker.ADD
            objectScanMarker.pose.position.x = 0.0
            objectScanMarker.pose.position.y = 0.0
            objectScanMarker.pose.position.z = 0.0
            objectScanMarker.pose.orientation.x = 0.0
            objectScanMarker.pose.orientation.y = 0.0
            objectScanMarker.pose.orientation.z = 0.0
            objectScanMarker.pose.orientation.w = 1.0
            objectScanMarker.scale.x = 1.0
            objectScanMarker.scale.y = 1.0
            objectScanMarker.scale.z = 1.0
            objectScanMarker.color.a = 1.0
            objectScanMarker.color.r = 1.0
            objectScanMarker.color.g = 1.0
            objectScanMarker.color.b = 1.0

            colorOfObjectScan = ColorRGBA(0.0, 1.0, 1.0, self.visualizeAlpha)

            for i, os in enumerate(scan['objects']):
                osX = x + os['x'] * cosH - os['y'] * sinH
                osY = y + os['x'] * sinH + os['y'] * cosH
                osH = heading + os['heading'] - np.pi / 2.0
                objectScanMarker.points += [Point(osX + 0.2 * np.cos(osH - np.pi / 2.0),
                                                  osY + 0.2 * np.sin(osH - np.pi / 2.0), 0.2)]
                objectScanMarker.points += [Point(osX + 0.2 * np.cos(osH),
                                                  osY + 0.2 * np.sin(osH), 0.2)]
                objectScanMarker.points += [Point(osX + 0.2 * np.cos(osH + np.pi / 2.0),
                                                  osY + 0.2 * np.sin(osH + np.pi / 2.0), 0.2)]
            objectScanMarker.colors += [colorOfObjectScan for i in range(len(objectScanMarker.points))]

            scanMarkers.markers += [objectScanMarker]

        self.pubScans.publish(scanMarkers)

