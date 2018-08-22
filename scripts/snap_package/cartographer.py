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

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Joy
from sensor_msgs.msg import PointCloud2, LaserScan

from ar_track_alvar_msgs.msg import AlvarMarkers

from snap.msg import *
from snap.srv import *

import math
import random as rnd
import numpy as np

import json
import os.path


class Cartographer(object):
    """ A small helper class for dealing with map data. """

    def __init__(self, snapMap, localization):
        """ The constructor for the Cartographer class.

            Parameters:
                snapMap         --  The SnapMap object.
                localization    --  The Localization object.
        """

        self.started = False

        self.mapName = rospy.get_param("~map_name", "unknown")
        self.mapDirectory = rospy.get_param("~map_directory", "maps")

        self.snapMap = snapMap
        self.localization = localization

        self.scans = list()

        self.activated = "disabled"
        self.joyButtonTime = rospy.get_rostime()
        self.joyDeadzone = float(rospy.get_param("~joy_deadzone", "0.1"))

        self.secondsPerUpdate = 1.0 / float(rospy.get_param("~update_rate", "10.0"))

        self.mappingCurrentLaserScan = list()
        self.mappingCurrentObjects = list()
        self.correctingCurrentScanIndex = 0

        self.subJoy = None
        self.subARTags = None
        self.subLaserScan = None

    def start(self):
        """ Start the necessary messages for cartographer. """

        if self.started:
            rospy.logwarn("Warn[Cartographer.start]: Already started.")
            return

        rospy.loginfo("Info[Cartographer.start]: Starting cartographer sub-controller.")

        self._load_scans()

        subJoyTopic = rospy.get_param("~sub_joy", "evt_joy")
        self.subJoy = rospy.Subscriber(subJoyTopic, Joy, self.sub_joy)

        subARTagsTopic = rospy.get_param("~sub_ar_tags", "ar_pose_marker")
        self.subARTags = rospy.Subscriber(subARTagsTopic, AlvarMarkers, self.sub_ar_tags)

        subLaserScanTopic = rospy.get_param("~sub_laser_scan", "scan")
        self.subLaserScan = rospy.Subscriber(subLaserScanTopic, LaserScan, self.sub_laser_scan)

        self.started = True

    def reset(self):
        """ Reset the cartographer variables. """

        rospy.loginfo("Info[Cartographer.reset]: Resetting cartographer sub-controller.")

        self.activated = "disabled"
        self.joyButtonTime = rospy.get_rostime()

        self.mappingCurrentLaserScanPoints = list()
        self.mappingCurrentObjects = list()
        self.correctingCurrentScanIndex = 0

    def _load_scans(self):
        """ Load the map data into the variables. """

        filename = self.mapName + "_scans.json"
        mapDataFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapDataFile, 'r') as f:
                self.scans = json.load(f)
            rospy.loginfo("Info[Cartographer._load_map_data]: Loaded the raw scans of the map from 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._load_map_data]: Failed to load the raw scans data.")

    def _save_scans(self):
        """ Load the map data into the variables. """

        filename = self.mapName + "_scans.json"
        mapDataFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapDataFile, 'w') as f:
                json.dump(self.scans, f)
            rospy.loginfo("Info[Cartographer._save_map_data]: Saved the raw scans of the map to 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._save_map_data]: Failed to save the raw scans data.")

    def get_scans(self):
        """ Get the laser and object scans.

            Returns:
                A list of dictionaries with scan data.
        """

        return self.scans

    def get_correcting_current_scan_index(self):
        """ Returns the current scan index when in 'correcting' mode.

            Returns:
                This current scan index from the self.scans list.
        """

        return self.correctingCurrentScanIndex

    def sub_joy(self, msg):
        """ Receive information about the joystick from ROS.

            Parameters:
                msg     --  The joystick information.
        """

        # Handle button presses with a 1/2 second delay between each button press.
        if self.joyButtonTime.to_sec() + 0.5 <= rospy.get_rostime().to_sec():
            self.joyButtonTime = rospy.get_rostime()

            # The "square" button activates/deactivates mapping and map correcting.
            if msg.buttons[0] == 1:
                if self.activated is "disabled": 
                    self.activated = "mapping"
                elif self.activated is "mapping": 
                    self.activated = "correcting"
                else:
                    self.activated = "disabled"

                rospy.loginfo("Info[Cartographer.sub_joy]: Cartographer is currently %s." % (self.activated))

            # The "x" button adds a new scan if mapping.
            elif msg.buttons[1] == 1 and self.activated == "mapping":
                position = self.localization.get_position_estimate()
                mappingCurrentPose = {'x': position.x, 'y': position.y, 'heading': self.localization.get_heading_estimate()}
                self.scans += [{'pose': mappingCurrentPose, 'points': self.mappingCurrentLaserScan,
                                'objects': self.mappingCurrentObjects}]
                self.correctingCurrentScanIndex = len(self.scans) - 1

                rospy.loginfo("Info[Cartographer.sub_joy]: Added a scan at (%.2f, %.2f, %.2f)!"
                              % (mappingCurrentPose['x'], mappingCurrentPose['y'], mappingCurrentPose['heading']))

            # The "start/option" button saves the new map.
            elif msg.buttons[9] == 1:
                self._save_scans()

        # If the user is correcting, then we control the scans here.
        if self.activated == "correcting":
            #  The "L1" and "R1" buttons decrement and increment the selected scan to correct, respectively.
            if msg.buttons[4] == 1:
                self.correctingCurrentScanIndex = (self.correctingCurrentScanIndex - 1) % len(self.scans)
            elif msg.buttons[5] == 1:
                self.correctingCurrentScanIndex = (self.correctingCurrentScanIndex + 1) % len(self.scans)

            # The "left analog stick" moves the selected scan.
            if abs(msg.axes[0]) >= self.joyDeadzone:
                self.scans[self.correctingCurrentScanIndex]['pose']['x'] -= msg.axes[0] * self.secondsPerUpdate * 0.1
            if abs(msg.axes[1]) >= self.joyDeadzone:
                self.scans[self.correctingCurrentScanIndex]['pose']['y'] += msg.axes[1] * self.secondsPerUpdate * 0.1

            # The "right analog stick" rotates the selected scan.
            if abs(msg.axes[2]) >= self.joyDeadzone:
                self.scans[self.correctingCurrentScanIndex]['pose']['heading'] += msg.axes[2] * self.secondsPerUpdate * 0.1
                if self.scans[self.correctingCurrentScanIndex]['pose']['heading'] >= np.pi:
                    self.scans[self.correctingCurrentScanIndex]['pose']['heading'] -= 2.0 * np.pi
                elif self.scans[self.correctingCurrentScanIndex]['pose']['heading'] <= -np.pi:
                    self.scans[self.correctingCurrentScanIndex]['pose']['heading'] += 2.0 * np.pi

    def sub_ar_tags(self, msg):
        """ Update the current list of AR tag objects visible and their locations.

            Parameters:
                msg     --  The AlvarMarkers message data.
        """

        if not self.started:
            rospy.logwarn("Warn[Cartographer.sub_ar_tags]: Initialization has not yet completed.")
            return

        self.mappingCurrentObjects = list()

        for marker in msg.markers:
            #rospy.loginfo("Info[Cartographer.sub_ar_tags]: Detected Marker ID %i!" % (marker.id))

            # Note: This is assuming the camera frame transform z-axis the world frame z-axis.
            roll, pitch, yaw = euler_from_quaternion([marker.pose.pose.orientation.x,
                                                      marker.pose.pose.orientation.y,
                                                      marker.pose.pose.orientation.z,
                                                      marker.pose.pose.orientation.w])

            self.mappingCurrentObjects += [{'uid': marker.id, 'heading': yaw,
                                            'x': marker.pose.pose.position.x,
                                            'y': marker.pose.pose.position.y}]

    def sub_laser_scan(self, msg):
        """ Update the current laser scan points. """

        if not self.started:
            rospy.logwarn("Warn[Cartographer.sub_laser_scan]: Initialization has not yet completed.")
            return

        #rospy.loginfo("Info[Cartographer.sub_laser_scan]: Detected laser scan!")

        # Convert laser scan ranges to a "fan" of points.
        self.mappingCurrentLaserScan = list()
        theta = float(msg.angle_min)

        for r in msg.ranges:
            # Only add this point to the list of it is valid.
            if r >= msg.range_min and r <= msg.range_max:
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                self.mappingCurrentLaserScan += [{'x': x, 'y': y}]

            theta += float(msg.angle_increment)

