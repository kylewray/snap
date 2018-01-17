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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan


class SLAM(object):
    """ A class for creating a map and localizing within it (SLAM). """

    def __init__(self):
        """ The constructor for the SLAM class. """

        self.started = False

        self.poseEstimate = None

        self.subKobukiOdometry = None
        self.subDepthPointCloud = None
        self.pubLaserScan = None

    def start(self):
        """ Start the necessary messages to create a map and localize. """

        if self.started:
            rospy.logwarn("Warn[SLAM.start]: Already started.")
            return

        rospy.loginfo("Info[SLAM.start]: Starting SLAM sub-controller.")

        subKobukiOdometryTopic = rospy.get_param(rospy.search_param('sub_kobuki_odometry'), "odom")
        self.subKobukiOdometry = rospy.Subscriber(subKobukiOdometryTopic,
                                                  Odometry,
                                                  self.sub_kobuki_odometry)

        subDepthPointCloudTopic = rospy.get_param(rospy.search_param('sub_depth_point_cloud'), "depth_point_cloud")
        self.subDepthPointCloud = rospy.Subscriber(subDepthPointCloudTopic,
                                                   PointCloud2,
                                                   self.sub_depth_point_cloud)

        pubLaserScanTopic = rospy.get_param(rospy.search_param('pub_laser_scan'), "scan")
        self.pubLaserScan = rospy.Publisher(pubLaserScanTopic, LaserScan, queue_size=8)

        self.started = True

    def reset(self):
        """ Reset the SLAM variables. """

        rospy.loginfo("Info[SLAM.reset]: Resetting SLAM sub-controller.")

        self.poseEstimate = None

    def get_pose_estimate(self):
        """ Return the current pose estimate (localization).

            Returns:
                The current pose estimate as a Pose object.
        """

        return self.poseEstimate

    def sub_kobuki_odometry(self, msg):
        """ Update the odometry information.

            Parameters:
                msg     --  The Odometry message data.
        """

        if not self.started:
            rospy.logwarn("Warn[SLAM.sub_kobuki_odometry]: Initialization has not yet completed.")
            return

        self.poseEstimate = msg.pose.pose

    def sub_depth_point_cloud(self, msg):
        """ Update the raw depth point cloud information, including sub-sampling and extracting abstract data.

            Parameters:
                msg     --  The raw PointCloud2 message data.
        """

        if not self.started:
            rospy.logwarn("Warn[SLAM.sub_depth_point_cloud]: Initialization has not yet completed.")
            return

        # TODO: Take raw point cloud, find points at a height, populate a Laser whatever msg, publish on gmapping topic...
        # Run gmapping in separate window. In rviz listen to the map topic. See how it does at mapping...

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

