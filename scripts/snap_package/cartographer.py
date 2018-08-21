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

        self.scans = list()

        self.activated = "disabled"
        self.joyButtonTime = rospy.get_rostime()

        self.mappingCurrentPose = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.mappingCurrentLaserScan = list()
        self.mappingCurrentObjects = list()

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

        self.mappingCurrentPose = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.mappingCurrentLaserScanPoints = list()
        self.mappingCurrentObjects = list()

    def _load_scans(self):
        """ Load the map data into the variables. """

        mapDataFile = os.path.join(self.mapDirectory, self.mapName + "_scans.json")
        try:
            with open(mapDataFile, 'r') as f:
                self.scans = json.load(f)
        except:
            rospy.logwarn("Warning[Cartographer._load_map_data]: Failed to load the raw scans data.")

    def _save_scans(self):
        """ Load the map data into the variables. """

        rospy.loginfo("Info[Cartographer._save_map_data]: Attempting to save the raw scans of the map.")

        mapDataFile = os.path.join(self.mapDirectory, self.mapName + "_scans.json")
        try:
            with open(mapDataFile, 'w') as f:
                json.dump(self.scans, f)
        except:
            rospy.logwarn("Warning[Cartographer._save_map_data]: Issues encountered saving the raw scans of the map.")

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
                self.scans += [{'pose': self.mappingCurrentPose, 'points': self.mappingCurrentLaserScan, 'objects': self.mappingCurrentObjects}]

                rospy.loginfo("Info[Cartographer.sub_joy]: Added a scan at (%.2f, %.2f, %.2f)!"
                              % (self.mappingCurrentPose['x'], self.mappingCurrentPose['y'], self.mappingCurrentPose['heading']))

            # The "start/option" button saves the new map.
            elif msg.buttons[9] == 1:
                self._save_scans()

        ## If the user is correcting, then we control the scans here.
        #if self.activated == "correcting":
        #    print(msg.axes[1], msg.axes[0], msg.buttons[4], msg.buttons[5])

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

            # Get the location in the map and offset it by the observed pose. This is the
            # observed estimate of the robot position.
            self.mappingCurrentObjects += [{'uid': marker.id, 'x': marker.pose.pose.position.x, 'y': marker.pose.pose.position.y}]

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

