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

import random as rnd

import json
import os.path


class Cartographer(object):
    """ A small helper class for dealing with map data. """

    def __init__(self):
        """ The constructor for the Map class. """

        self.started = False

        self.mapName = rospy.get_param("~map_name", "unknown")
        self.mapDirectory = rospy.get_param("~map_directory", "maps")

        self.regions = list()
        self.connections = list()
        self.objects = list()

    def start(self):
        """ Start the necessary messages for cartographer. """

        if self.started:
            rospy.logwarn("Warn[Cartographer.start]: Already started.")
            return

        rospy.loginfo("Info[Cartographer.start]: Starting cartographer sub-controller.")

        self._load_map_data()

        self.started = True

    def reset(self):
        """ Reset the cartographer variables. """

        rospy.loginfo("Info[Cartographer.reset]: Resetting cartographer sub-controller.")

    def _load_map_data(self):
        """ Load the map data into the variables. """

        mapDataFile = os.path.join(self.mapDirectory, self.mapName + ".json")
        with open(mapDataFile, 'r') as f:
            data = json.load(f)

        try:
            self.regions = data['regions']
            self.connections = data['connections']
            self.objects = data['objects']
        except:
            rospy.logwarn("Warning[Cartographer._load_map_data]: Issues encountered loading map data.")

    def get_region(self, regionUID):
        """ Get the region from the region UID.

            Parameters:
                regionUID   --  The region UID to get.

            Returns:
                The region corresponding to the UID, or None otherwise.
        """

        for region in self.regions:
            if region['uid'] == regionUID:
                return region

        return None

    def get_random_point_in_region(self, regionUID):
        """ Get a random location in a particular region.

            Parameters:
                regionUID   --  The UID of the region to randomly choose a point within.

            Returns:
                A random point in the region, or None if the input is invalid.
        """

        region = self.get_region(regionUID)
        if region is None:
            return None

        weight = [rnd.random() for p in region['bounds']]
        denominator = sum(weight)

        point = Point()
        point.x = sum([p[0] * weight[i] / denominator for i, p in enumerate(region['bounds'])])
        point.y = sum([p[1] * weight[i] / denominator for i, p in enumerate(region['bounds'])])

        return point

    def is_point_in_region(self, regionUID, point):
        """ Check if the point provided is within the region provided.

            Parameters:
                regionUID   --  The region UID.
                point       --  The Point object to test.

            Returns:
                True if the point is in the region, False otherwise.
        """

        region = self.get_region(regionUID)
        if region is None:
            return False

        for i in range(len(region.bounds) - 1):
            a = Point(region.bounds[i][0], region.bounds[i][1], 0.0)
            b = Point(region.bounds[i + 1][0], region.bounds[i + 1][1], 0.0)
            c = Point(point.x, point.y, 0.0)

            isLeft = ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
            if isLeft > 0.0:
                return False

        return True

    def get_region_by_point(self, point):
        """ Check if the point provided is within any of the regions. If so, return any such region.

            Parameters:
                point   --  The Point object to test.

            Returns:
                The region data if the point is in any region, otherwise None.
        """

        for region in self.regions:
            if self.is_point_in_region(region['uid'], point):
                return region

        return None

