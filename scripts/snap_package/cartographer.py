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
from nav_msgs.msg import OccupancyGrid

from ar_track_alvar_msgs.msg import AlvarMarkers

from snap.msg import *
from snap.srv import *

import math
import random as rnd
import numpy as np

import json
import os.path

from PIL import Image


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
        self.joyDeadzone = float(rospy.get_param("~joy_deadzone", "0.1"))
        self.joyPreviousButtons = [0 for i in range(16)]

        self.secondsPerUpdate = 1.0 / float(rospy.get_param("~update_rate", "10.0"))

        self.mapFrameID = rospy.get_param("~map_frame_id", "map")
        self.mapResolution = float(rospy.get_param("~cartographer_map_resolution", "0.1"))
        self.mapWidth = int(rospy.get_param("~cartographer_map_width", 600))
        self.mapHeight = int(rospy.get_param("~cartographer_map_height", 400))
        self.mapOriginX = self.mapWidth / 2.0 * self.mapResolution
        self.mapOriginY = self.mapHeight / 2.0 * self.mapResolution

        self.mappingCurrentLaserScan = list()
        self.mappingCurrentObjects = list()
        self.correctingCurrentScanIndex = 0
        self.scanSubsample = int(rospy.get_param("~cartographer_scan_subsample", "10"))

        self.subJoy = None
        self.subARTags = None
        self.subLaserScan = None

        self.pubMap = None
        self.pubObservationDetectedObjects = None

    def start(self):
        """ Start the necessary messages for cartographer. """

        if self.started:
            rospy.logwarn("Warn[Cartographer.start]: Already started.")
            return

        rospy.loginfo("Info[Cartographer.start]: Starting cartographer sub-controller.")

        self._load_scans()

        pubObservationDetectedObjectsTopic = rospy.get_param("~pub_observation_detected_objects",
                                                             "~observation_detected_objects")
        self.pubObservationDetectedObjects = rospy.Publisher(pubObservationDetectedObjectsTopic,
                                                             ObservationDetectedObjects, queue_size=8)

        subJoyTopic = rospy.get_param("~sub_joy", "evt_joy")
        self.subJoy = rospy.Subscriber(subJoyTopic, Joy, self.sub_joy)

        subARTagsTopic = rospy.get_param("~sub_ar_tags", "ar_pose_marker")
        self.subARTags = rospy.Subscriber(subARTagsTopic, AlvarMarkers, self.sub_ar_tags)

        subLaserScanTopic = rospy.get_param("~sub_laser_scan", "scan")
        self.subLaserScan = rospy.Subscriber(subLaserScanTopic, LaserScan, self.sub_laser_scan)

        pubMapTopic = rospy.get_param("~pub_map", "map")
        self.pubMap = rospy.Publisher(pubMapTopic, OccupancyGrid, queue_size=8)

        self.started = True

    def reset(self):
        """ Reset the cartographer variables. """

        rospy.loginfo("Info[Cartographer.reset]: Resetting cartographer sub-controller.")

        self.activated = "disabled"

        self.mappingCurrentLaserScanPoints = list()
        self.mappingCurrentObjects = list()
        self.correctingCurrentScanIndex = 0

    def _load_scans(self):
        """ Load the map data into the variables. """

        filename = self.mapName + "_scans.json"
        mapScansFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapScansFile, 'r') as f:
                self.scans = json.load(f)
            rospy.loginfo("Info[Cartographer._load_map_data]: Loaded the raw scans of the map from 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._load_map_data]: Failed to load the raw scans data.")

    def _save_scans(self):
        """ Save the map data into the variables. """

        filename = self.mapName + "_scans.json"
        mapScansFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapScansFile, 'w') as f:
                json.dump(self.scans, f)
            rospy.loginfo("Info[Cartographer._save_map_data]: Saved the raw scans of the map to 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._save_map_data]: Failed to save the raw scans data.")

    def _save_map_data_as_yaml(self):
        """ Save the YAML file for the new map. """

        filename = self.mapName + "_new.yaml"
        mapYAMLFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapYAMLFile, 'w') as f:
                f.write("image: %s_new.png\n" % (self.mapName))
                f.write("resolution: %.6f\n" % (self.mapResolution))
                f.write("origin: [%.3f, %.3f, %.3f]\n" % (-self.mapOriginX, -self.mapOriginY, 0.0))
                f.write("occupied_thresh: 0.65\n")
                f.write("free_thresh: 0.196\n")
                f.write("negate: 0")
            rospy.loginfo("Info[Cartographer._save_map_data_as_yaml]: Saved the new map data to 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._save_map_data_as_yaml]: Failed to save the new map data.")

    def _save_occupancy_grid_as_image(self):
        """ Save the new occupancy grid to an image. """

        filename = self.mapName + "_new.png"
        mapImageFile = os.path.join(self.mapDirectory, filename)

        occupancyGridData = self._compute_occupancy_grid_data()
        data = [int(float(100 - occupancyGridData[i]) / 100.0) for i in range(len(occupancyGridData))]

        try:
            img = Image.new('1', (self.mapWidth, self.mapHeight))
            img.putdata(data)
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            img.save(mapImageFile)
            rospy.loginfo("Info[Cartographer._save_occupancy_grid_as_image]: Saved the new image of the map to 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._save_occupancy_grid_as_image]: Failed to save the new image of the map.")

    def _save_map_data_as_json(self):
        """ Save the JSON file for the new map. """

        filename = self.mapName + "_new.json"
        mapJSONFile = os.path.join(self.mapDirectory, filename)
        try:
            with open(mapJSONFile, 'w') as f:
                # Create a dictionary mapping object UIDs to a list of poses.
                objectDict = dict()
                for scan in self.scans:
                    x = scan['pose']['x']
                    y = scan['pose']['y']
                    heading = scan['pose']['heading']
                    cosH = math.cos(heading)
                    sinH = math.sin(heading)

                    for obj in scan['objects']:
                        objUID = obj['uid']
                        objX = x + obj['x'] * cosH - obj['y'] * sinH
                        objY = y + obj['x'] * sinH + obj['y'] * cosH
                        objH = heading + obj['heading'] - np.pi / 2.0

                        try:
                            objectDict[objUID]['x'] += [objX]
                            objectDict[objUID]['y'] += [objY]
                            objectDict[objUID]['heading'] += [objH]
                        except KeyError:
                            objectDict[objUID] = {'x': [objX], 'y': [objY], 'heading': [objH]}

                # Perform averaging over x, y, and heading observations.
                objectList = list()
                for uid, xyHeadingLists in objectDict.items():
                    objectList += [{'uid': uid,
                                    'name': "Unknown",
                                    'type': "static",
                                    'tag_uid': uid,
                                    'position': [sum(objectDict[uid]['x']) / len(objectDict[uid]['x']),
                                                    sum(objectDict[uid]['y']) / len(objectDict[uid]['y'])],
                                    'heading': sum(objectDict[uid]['heading']) / len(objectDict[uid]['heading']),
                                    'color': [rnd.random(), rnd.random(), rnd.random()]}]

                # Write them all to this file. There are no regions or connections yet, so this is empty.
                data = {'regions': list(), 'connections': list(), 'objects': objectList}
                json.dump(data, f)
            rospy.loginfo("Info[Cartographer._save_map_data_as_json]: Saved the new 'snap map' data to 'maps/%s'." % (filename))
        except:
            rospy.logwarn("Warning[Cartographer._save_map_data_as_json]: Failed to save the new 'snap map' data.")

    def _compute_occupancy_grid_data(self):
        """ Compute the raw occupancy grid data.

            Returns:
                The raw occupancy grid data as a list with 0 denoting freespace and 100 denoting an occupied space.
        """

        # Compute the data from the laser scans. Note: 0 is freespace, 100 is occupied.
        data = [100 for i in range(self.mapWidth * self.mapHeight)]
        for scan in self.scans:
            # We look over all scans starting at its pose.
            x = scan['pose']['x'] + self.mapOriginX
            y = scan['pose']['y'] + self.mapOriginY
            heading = scan['pose']['heading']
            cosH = math.cos(heading)
            sinH = math.sin(heading)

            for point in scan['points']:
                # For each point in the scan, we compute the actual world frame position of this point.
                lsX = x + point['x'] * cosH - point['y'] * sinH
                lsY = y + point['x'] * sinH + point['y'] * cosH

                # We compute the 'range' which is the distance (meters and num cells) between the pose and this position.
                distance = math.sqrt(pow(x - lsX, 2) + pow(y - lsY, 2))
                numCellsBetweenTheTwoPoints = distance / self.mapResolution

                for weight in np.arange(0.0, 1.0, 1.0 / numCellsBetweenTheTwoPoints):
                    # We compute a weighted point between the world frame scan pose and this world frame position.
                    wx = float(weight) * x + (1.0 - float(weight)) * lsX
                    wy = float(weight) * y + (1.0 - float(weight)) * lsY

                    # Lastly, compute the cell coordinate (map frame) of the world frame position of this point.
                    cwx = int(wx / self.mapResolution)
                    cwy = int(wy / self.mapResolution)

                    if cwx < 0 or cwx >= self.mapWidth or cwy < 0 or cwy >= self.mapHeight:
                        #rospy.logwarn("Warn[Cartographer._compute_occupancy_grid]: Cell out of bounds.")
                        continue

                    # This is a freespace (i.e., probability 0 of occupancy).
                    data[cwy * self.mapWidth + cwx] = 0

        return data

    def _compute_occupancy_grid(self):
        """ Compute a new map (OccupancyGrid) from the current scan data.

            Returns:
                A new OccupancyGrid made by the scan data.
        """

        # Construct the basic info about the OccupancyGrid.
        result = OccupancyGrid()
        result.header.frame_id = self.mapFrameID
        result.header.stamp = rospy.get_rostime()
        result.info.resolution = self.mapResolution
        result.info.width = self.mapWidth
        result.info.height = self.mapHeight
        result.info.origin.position.x = -self.mapOriginX
        result.info.origin.position.y = -self.mapOriginY
        result.info.origin.position.z = 0.0
        result.info.origin.orientation.x = 0.0
        result.info.origin.orientation.y = 0.0
        result.info.origin.orientation.z = 0.0
        result.info.origin.orientation.w = 1.0
        result.data = self._compute_occupancy_grid_data()

        return result

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

        # The "square" button activates/deactivates mapping and map correcting.
        if msg.buttons[0] == 1 and self.joyPreviousButtons[0] == 0:
            if self.activated is "disabled": 
                self.activated = "mapping"
            elif self.activated is "mapping": 
                self.activated = "correcting"
            else:
                self.activated = "disabled"

            rospy.loginfo("Info[Cartographer.sub_joy]: Cartographer is currently %s." % (self.activated))

        # The "start/option" button saves the new map.
        elif msg.buttons[9] == 1 and self.joyPreviousButtons[9] == 0:
            self._save_scans()
            self._save_map_data_as_yaml()
            self._save_occupancy_grid_as_image()
            self._save_map_data_as_json()

        # The "share" button loads the old map.
        elif msg.buttons[8] == 1 and self.joyPreviousButtons[8] == 0:
            self._load_scans()

        # If the user is mapping, then we add the new scans here.
        if self.activated == "mapping":
            # The "x" button adds a new scan if mapping.
            if msg.buttons[1] == 1 and self.joyPreviousButtons[1] == 0:
                position = self.localization.get_position_estimate()
                mappingCurrentPose = {'x': position.x, 'y': position.y, 'heading': self.localization.get_heading_estimate()}
                self.scans += [{'pose': mappingCurrentPose, 'points': self.mappingCurrentLaserScan,
                                'objects': self.mappingCurrentObjects}]
                self.correctingCurrentScanIndex = len(self.scans) - 1

                rospy.loginfo("Info[Cartographer.sub_joy]: Added a scan at (%.2f, %.2f, %.2f)!" %
                            (mappingCurrentPose['x'], mappingCurrentPose['y'], mappingCurrentPose['heading']))

        # If the user is correcting and there are scans to correct, then we control the scans here.
        if self.activated == "correcting" and len(self.scans) > 0:
            #  The "L1" and "R1" buttons decrement and increment the selected scan to correct, respectively.
            if msg.buttons[4] == 1 and self.joyPreviousButtons[4] == 0:
                self.correctingCurrentScanIndex = (self.correctingCurrentScanIndex - 1) % len(self.scans)
            elif msg.buttons[5] == 1 and self.joyPreviousButtons[5] == 0:
                self.correctingCurrentScanIndex = (self.correctingCurrentScanIndex + 1) % len(self.scans)

            # The "circle" button deletes the selected scan.
            elif msg.buttons[2] == 1 and self.joyPreviousButtons[2] == 0:
                rospy.loginfo("Info[Cartographer.sub_joy]: Removed scan %i at (%.2f, %.2f, %.2f). Now there are %i scans remaining." %
                            (self.correctingCurrentScanIndex,
                            self.scans[self.correctingCurrentScanIndex]['pose']['x'],
                            self.scans[self.correctingCurrentScanIndex]['pose']['y'],
                            self.scans[self.correctingCurrentScanIndex]['pose']['heading'],
                            len(self.scans) - 1))

                self.scans = self.scans[0:self.correctingCurrentScanIndex] + self.scans[(self.correctingCurrentScanIndex + 1):len(self.scans)]
                self.correctingCurrentScanIndex = max(0, min(len(self.scans) - 1, self.correctingCurrentScanIndex))

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

        # If the user is mapping or correcting, then we can publish a new map.
        if self.activated == "mapping" or self.activated == "correcting":
            # The "big center button" publishes a new map.
            if msg.buttons[13] == 1 and self.joyPreviousButtons[13] == 0:
                self.pubMap.publish(self._compute_occupancy_grid())

        joyPreviousButtons = list(msg.buttons)

    def sub_ar_tags(self, msg):
        """ Update the current list of AR tag objects visible and their locations.

            Parameters:
                msg     --  The AlvarMarkers message data.
        """

        if not self.started:
            rospy.logwarn("Warn[Cartographer.sub_ar_tags]: Initialization has not yet completed.")
            return

        # First, we handle mapping by updating the current list of objects for mapping purposes.
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

        # Second, we handle updating the poses of dynamic objects already created in the map that exists.
        detectedObjects = list()

        for marker in msg.markers:
            # Check if the observed AR tag is in the map. If not, continue.
            obj = self.snapMap.get_object(marker.id)
            if obj is None:
                continue

            # Record that we observed this object.
            detectedObjects += [marker.id]

            # Even if it is in the map, only update it if it has the "dynamic" type.
            if obj['type'] != "dynamic":
                continue

            # Get the yaw of the marker.
            roll, pitch, yaw = euler_from_quaternion([marker.pose.pose.orientation.x,
                                                      marker.pose.pose.orientation.y,
                                                      marker.pose.pose.orientation.z,
                                                      marker.pose.pose.orientation.w])
            if yaw > np.pi:
                yaw -= 2.0 * float(np.pi)
            elif yaw < -np.pi:
                yaw += 2.0 * float(np.pi)

            x = self.localization.get_position_estimate().x
            y = self.localization.get_position_estimate().y
            heading = self.localization.get_heading_estimate()
            cosH = math.cos(heading)
            sinH = math.sin(heading)

            objX = marker.pose.pose.position.x
            objY = marker.pose.pose.position.y
            objHeading = yaw

            # Update the (x, y, theta) in the list of objects.
            obj['position'][0] = x + objX * cosH - objY * sinH
            obj['position'][1] = y + objX * sinH + objY * cosH
            obj['heading'] = heading + objHeading - float(np.pi / 2.0)

        # Publish a message of all objects detected.
        observationObjects = ObservationDetectedObjects()
        observationObjects.object_uids = detectedObjects
        self.pubObservationDetectedObjects.publish(observationObjects)

    def sub_laser_scan(self, msg):
        """ Update the current laser scan points. """

        if not self.started:
            rospy.logwarn("Warn[Cartographer.sub_laser_scan]: Initialization has not yet completed.")
            return

        #rospy.loginfo("Info[Cartographer.sub_laser_scan]: Detected laser scan!")

        # Convert laser scan ranges to a "fan" of points.
        self.mappingCurrentLaserScan = list()
        theta = float(msg.angle_min)

        for i, r in enumerate(msg.ranges):
            # Only add this point to the list of it is valid, and if it is one of the subsampled ones.
            if r >= msg.range_min and r <= msg.range_max and i % self.scanSubsample == 0:
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                self.mappingCurrentLaserScan += [{'x': x, 'y': y}]

            theta += float(msg.angle_increment)

