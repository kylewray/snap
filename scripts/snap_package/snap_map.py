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

from snap.msg import *
from snap.srv import *

import random as rnd
import numpy as np

import json
import os.path


class SnapMap(object):
    """ A small helper class for dealing with map data. """

    def __init__(self):
        """ The constructor for the SnapMap class. """

        self.started = False

        self.mapName = rospy.get_param("~map_name", "unknown")
        self.mapDirectory = rospy.get_param("~map_directory", "maps")

        self.regions = list()
        self.connections = list()
        self.objects = list()

        # Used to properly publish the alpha value of a color.
        self.visualizeAlpha = float(rospy.get_param("~visualize_alpha", "0.2"))

        # TODO: Publish detected objects that match the map's objects.
        self.pubObservationObjectDetection = None

    def start(self):
        """ Start the necessary messages for map. """

        if self.started:
            rospy.logwarn("Warn[SnapMap.start]: Already started.")
            return

        rospy.loginfo("Info[SnapMap.start]: Starting map sub-controller.")

        self._load_map_data()

        srvGetRegionsTopic = rospy.get_param("~map_get_regions_topic", "~get_regions")
        self.srvGetRegions = rospy.Service(srvGetRegionsTopic,
                                           GetRegions,
                                           self.srv_get_regions)

        srvGetRegionTopic = rospy.get_param("~map_get_region_topic", "~get_region")
        self.srvGetRegion = rospy.Service(srvGetRegionTopic,
                                          GetRegion,
                                          self.srv_get_region)

        srvGetRegionByPointTopic = rospy.get_param("~map_get_region_by_point_topic", "~get_region_by_point")
        self.srvGetRegionByPoint = rospy.Service(srvGetRegionByPointTopic,
                                                 GetRegionByPoint,
                                                 self.srv_get_region_by_point)

        srvGetRegionNeighborsTopic = rospy.get_param("~map_get_region_neighbors_topic",
                                                     "~get_region_neighbors")
        self.srvGetRegionNeighbors = rospy.Service(srvGetRegionNeighborsTopic,
                                                   GetRegionNeighbors,
                                                   self.srv_get_region_neighbors)

        srvGetConnectionsTopic = rospy.get_param("~map_get_connections_topic", "~get_connections")
        self.srvGetConnections = rospy.Service(srvGetConnectionsTopic,
                                               GetConnections,
                                               self.srv_get_connections)

        srvGetConnectionTopic = rospy.get_param("~map_get_connection_topic", "~get_connection")
        self.srvGetConnection = rospy.Service(srvGetConnectionTopic,
                                              GetConnection,
                                              self.srv_get_connection)

        srvGetConnectionRegionsTopic = rospy.get_param("~map_get_connection_regions_topic",
                                                       "~get_connection_regions")
        self.srvGetConnectionRegions = rospy.Service(srvGetConnectionRegionsTopic,
                                                     GetConnectionRegions,
                                                     self.srv_get_connection_regions)

        srvGetObjectsTopic = rospy.get_param("~map_get_objects_topic", "~get_objects")
        self.srvGetObjects = rospy.Service(srvGetObjectsTopic,
                                           GetObjects,
                                           self.srv_get_objects)

        srvGetObjectTopic = rospy.get_param("~map_get_object_topic", "~get_object")
        self.srvGetObject = rospy.Service(srvGetObjectTopic,
                                          GetObject,
                                          self.srv_get_object)

        self.started = True

    def reset(self):
        """ Reset the map variables. """

        rospy.loginfo("Info[SnapMap.reset]: Resetting map sub-controller.")

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
            rospy.logwarn("Warning[SnapMap._load_map_data]: Issues encountered loading map data.")

    def get_regions(self):
        """ Get the list of regions as a dictionary of information.

            Returns:
                The list of regions as a dictionary of information.
        """

        return self.regions

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

    def get_center_of_region(self, regionUID):
        """ Get the center of a region---that is, the average of the bounds.

            Parameters:
                regionUID   --  The UID of the region to randomly choose a point within.

            Returns:
                The average of the bounds of the region, or None if the input is invalid.
        """

        region = self.get_region(regionUID)
        if region is None:
            return None

        point = Point()
        point.x = float(sum([p[0] for p in region['bounds']])) / float(len(region['bounds']))
        point.y = float(sum([p[1] for p in region['bounds']])) / float(len(region['bounds']))

        return point

    def get_random_point_in_region(self, regionUID, weightPadding=0.1):
        """ Get a random location in a particular region. Optionally, ensure it is weighted away from edges.

            Parameters:
                regionUID       --  The UID of the region to randomly choose a point within.
                weightPadding   --  Optionally, the amount of weight to 'pad' around the bounds.
                                    Default is 0.1; 0.0 is totally random; 1.0 is average center of bounds.

            Returns:
                A random point in the region, or None if the input is invalid.
        """

        region = self.get_region(regionUID)
        if region is None:
            return None

        weight = [(1.0 - weightPadding) * rnd.random() + weightPadding for p in region['bounds']]
        denominator = sum(weight)

        point = Point()
        point.x = sum([p[0] * weight[i] / denominator for i, p in enumerate(region['bounds'])])
        point.y = sum([p[1] * weight[i] / denominator for i, p in enumerate(region['bounds'])])

        return point

    def is_point_in_region(self, regionUID, point):
        """ Check if the point provided is within the clockwise region bounds provided.

            Parameters:
                regionUID   --  The region UID.
                point       --  The Point object to test.

            Returns:
                True if the point is in the region, False otherwise.
        """

        region = self.get_region(regionUID)
        if region is None:
            return False

        for i in range(len(region['bounds'])):
            iPlusOne = (i + 1) % len(region['bounds'])
            a = Point(region['bounds'][i][0], region['bounds'][i][1], 0.0)
            b = Point(region['bounds'][iPlusOne][0], region['bounds'][iPlusOne][1], 0.0)
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

    def get_region_neighbors(self, regionUID):
        """ Get the neighbors of a region, following the connections, from the connection UID.

            Parameters:
                regionUID   --  The region UID to get its neighboring connections.

            Returns:
                The neighboring regions, following the connections, of the region UID given, None on failure.
        """

        region = self.get_region(regionUID)
        if region is None:
            return None

        neighborUIDs = set()
        for connection in self.connections:
            if region['uid'] in connection['region_uids']:
                neighborUIDs |= set(connection['region_uids']) - {region['uid']}

        regions = [self.get_region(uid) for uid in neighborUIDs]
        return [r for r in regions if r is not None]

    def get_connections(self):
        """ Get the list of connections as a dictionary of information.

            Returns:
                The list of connections as a dictionary of information.
        """

        return self.connections

    def get_connection(self, connectionUID):
        """ Get the connection from the connection UID.

            Parameters:
                connectionUID   --  The connection UID to get.

            Returns:
                The connection corresponding to the UID, or None otherwise.
        """

        for con in self.connections:
            if con['uid'] == connectionUID:
                return con

        return None

    def get_connection_regions(self, connectionUID):
        """ Get the connection region information from the connection UID.

            Parameters:
                connectionUID   --  The connection UID to get.

            Returns:
                The connection region information corresponding to the UID, or None otherwise.
        """

        con = self.get_connection(connectionUID)
        if con is None:
            return None

        regions = [self.get_region(regionUID) for regionUID in con['region_uids']]

        return regions

    def get_objects(self):
        """ Get the list of objects as a dictionary of information.

            Returns:
                The list of objects as a dictionary of information.
        """

        return self.objects

    def get_object(self, objectUID):
        """ Get the object from the object UID.

            Parameters:
                objectUID   --  The object UID to get.

            Returns:
                The object corresponding to the UID, or None otherwise.
        """

        for obj in self.objects:
            if obj['uid'] == objectUID:
                return obj

        return None

    def get_object_position(self, objectUID):
        """ Get the object position of the object UID given.

            Returns:
                The object position as a Point object, or None if the input is invalid.
        """

        obj = self.get_object(objectUID)
        if obj is not None:
            return Point(obj['position'][0], obj['position'][1], 0.0)

        return None

    def get_object_heading(self, objectUID):
        """ Get the object heading of the object UID given.

            Returns:
                The object heading in [-pi, pi], or None if the input is invalid.
        """

        obj = self.get_object(objectUID)
        if obj is None:
            return None

        if obj['heading'] < -float(np.pi):
            return float(obj['heading'] + float(2.0 * np.pi))
        elif obj['heading'] > float(np.pi):
            return float(obj['heading'] - float(2.0 * np.pi))
        else:
            return float(obj['heading'])

    def _convert_region_to_msg(self, region):
        """ Convert a region to a region message for easy service definitions.

            Parameters:
                region      --  The region to convert.

            Returns:
                The region message for the region provided.
        """

        regionMsg = Region()
        regionMsg.uid = region['uid']
        regionMsg.name = region['name']
        regionMsg.bounds = [Point(bound[0], bound[1], 0.0) for bound in region['bounds']]
        regionMsg.color = ColorRGBA(region['color'][0], region['color'][1], region['color'][2], self.visualizeAlpha)
        regionMsg.object_uids = region['object_uids']
        return regionMsg

    def _convert_connection_to_msg(self, connection):
        """ Convert a connection to a connection message for easy service definitions.

            Parameters:
                connection  --  The connection to convert.

            Returns:
                The connection message for the connection provided.
        """

        connectionMsg = Connection()
        connectionMsg.uid = connection['uid']
        connectionMsg.name = connection['name']
        connectionMsg.region_uids = connection['region_uids']
        connectionMsg.weight = connection['weight']
        return connectionMsg

    def _convert_object_to_msg(self, obj):
        """ Convert an object to an object message for easy service definitions.

            Parameters:
                obj     --  The object to convert.

            Returns:
                The object message for the object provided.
        """

        objMsg = Object()
        objMsg.uid = obj['uid']
        objMsg.name = obj['name']
        objMsg.tag_uid = obj['tag_uid']
        objMsg.position = Point(obj['position'][0], obj['position'][1], 0.0)
        objMsg.heading = obj['heading']
        objMsg.color = ColorRGBA(obj['color'][0], obj['color'][1], obj['color'][2], self.visualizeAlpha)
        return objMsg

    def srv_get_regions(self, request):
        """ Service callback for getting the list of regions as a dictionary of information.

            Parameters:
                request     --  The GetRegionsRequest object.

            Returns:
                The GetRegionsResponse object.
        """

        regions = [self._convert_region_to_msg(region) for region in self.regions]
        return GetRegionsResponse(regions)

    def srv_get_region(self, request):
        """ Service callback for getting the region from the region UID.

            Parameters:
                request     --  The GetRegionRequest object.

            Returns:
                The GetRegionResponse object.
        """

        regionResponse = GetRegionResponse()
        region = self.get_region(request.uid)
        if region is None:
            regionResponse.exists = False
        else:
            regionResponse.exists = True
            regionResponse.region = self._convert_region_to_msg(region)
        return regionResponse

    def srv_get_region_by_point(self, request):
        """ Service callback for getting the region from a specified point.

            Parameters:
                request     --  The GetRegionByPointRequest object.

            Returns:
                The GetRegionByPointResponse object.
        """

        regionResponse = GetRegionByPointResponse()
        region = self.get_region_by_point(request.point)
        if region is None:
            regionResponse.exists = False
        else:
            regionResponse.exists = True
            regionResponse.region = self._convert_region_to_msg(region)
        return regionResponse

    def srv_get_region_neighbors(self, request):
        """ Service callback for getting the list of neighboring regions as a dictionary of information.

            Parameters:
                request     --  The GetRegionNeighborsRequest object.

            Returns:
                The GetRegionNeighborsResponse object.
        """

        neighborsResponse = GetRegionNeighborsResponse()
        regions = self.get_region_neighbors(request.uid)
        if regions is None:
            neighborsResponse.exists = False
        else:
            neighborsResponse.exists = True
            neighborsResponse.regions = [self._convert_region_to_msg(region) for region in regions]
        return neighborsResponse

    def srv_get_connections(self, request):
        """ Get the list of connections as a dictionary of information.

            Parameters:
                request     --  The GetConnectionsRequest object.

            Returns:
                The GetConnectionsResponse object.
        """

        connections = [self._convert_connection_to_msg(connection) for connection in self.connections]
        return GetConnectionsResponse(connection)

    def srv_get_connection(self, request):
        """ Service callback for getting the connection from the connection UID.

            Parameters:
                request     --  The GetConnectionRequest object.

            Returns:
                The GetConnectionResponse object.
        """

        connectionResponse = GetConnectionResponse()
        connection = self.get_connection(request.uid)
        if connection is None:
            connectionResponse.exists = False
        else:
            connectionResponse.exists = True
            connectionResponse.connection = self._convert_connection_to_msg(connection)
        return connectionResponse

    def srv_get_connection_regions(self, request):
        """ Service callback for getting the connection region information from the connection UID.

            Parameters:
                request     --  The GetConnectionRegionsRequest object.

            Returns:
                The GetConnectionRegionsResponse object.
        """

        connectionRegionsResponse = GetConnectionRegionsResponse()
        regions = self.get_connection_regions(request.uid)
        if regions is None:
            connectionRegionsResponse.exists = False
        else:
            connectionRegionsResponse.exists = True
            connectionRegionsResponse.regions = [self._convert_region_to_msg(r) for r in regions]
        return connectionRegionsResponse

    def srv_get_objects(self, request):
        """ Get the list of objects as a dictionary of information.

            Parameters:
                request     --  The GetObjectsRequest object.

            Returns:
                The GetObjectsResponse object.
        """

        objects = [self._convert_object_to_msg(obj) for obj in self.objects]
        return GetObjectsResponse(objects)

    def srv_get_object(self, request):
        """ Service callback for getting the object from the object UID.

            Parameters:
                request     --  The GetObjectRequest object.

            Returns:
                The GetObjectResponse object.
        """

        objectResponse = GetObjectResponse()
        obj = self.get_object(request.uid)
        if obj is None:
            objectResponse.exists = False
        else:
            objectResponse.exists = True
            objectResponse.object = self._convert_object_to_msg(obj)
        return objectResponse


