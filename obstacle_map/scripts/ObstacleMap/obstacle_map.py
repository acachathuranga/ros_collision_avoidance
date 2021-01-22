#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from DepthSource.depth_source import DepthSource
from utilities.robot_footprint_reader import FootPrint
from collections import deque
from threading import Timer
import numpy as np

class ObstacleMap():
    def __init__(self):
        averaging_window = rospy.get_param(rospy.get_name() + "/averaging_window")
        map_publish_rate = rospy.get_param(rospy.get_name() + "/map_publish_rate")
        map_topic = rospy.get_param(rospy.get_name() + "/map_topic")
        exclude_footprint = rospy.get_param(rospy.get_name() + "/exclude_footprint", True)
        sources = rospy.get_param(rospy.get_name() + "/depth_sources")
        sources = sources.replace(" ","").split(",")
        rospy.loginfo(rospy.get_name() + ": Obstacle Map Started with %d sources"%len(sources))

        self.set_map_parameters()

        self.depth_source_handler = {}
        self.depth_source_cache = {}
        self.depth_source_map = {}
        for source in sources:
            if rospy.has_param(rospy.get_name() + "/" + source):
                params = rospy.get_param(rospy.get_name() + "/" + source)
                self.depth_source_handler[source] = self.create_depth_source(source, params, self.depth_map_callback)
                self.depth_source_cache[source] = deque(maxlen=averaging_window)
                rospy.loginfo("%s : Source '%s' connected!"%(rospy.get_name(), source))
            else:
                rospy.logerr("%s : Source '%s' has incomplete parameter definition"%(rospy.get_name(), source))

        # Start Map publisher
        self.map_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=20)
        map_publish_interval = 1.0 / map_publish_rate       # Seconds
        self.timer = ClassTimer(self.publish_map, map_publish_interval)
        # Map message sequence number
        self.seq = 0

        # Footprint map listener
        self.footprint = None
        if exclude_footprint:
            footprint_receiver = FootPrint(self.map_x_range, self.map_y_range, self.resolution, self.footprint_callback)

    def create_depth_source(self, source, params, callback):
        """
            :param source: Source name
            :param params: Source parameters
            :param callback: Callback function to accept (depth_map, source_name)
        """

        # Setting Defaults
        if "rate" in params:
            rate = params["rate"]
        else:
            rate = 5

        if "obstacle_probability_threshold" in params:
            probability_threshold = params["obstacle_probability_threshold"]
        else:
            probability_threshold = 1

        try:
            return DepthSource(source, params['topic'], params['sensor_frame'], params['parent_frame'], rate, callback,
            self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max, self.resolution, probability_threshold)
        except Exception as ex:
            rospy.logerr(rospy.get_name() + "::obstacle_map : " + str(ex))

    def set_map_parameters(self):
        if rospy.has_param(rospy.get_name() + "/map_parameters"):
            params = rospy.get_param(rospy.get_name() + "/map_parameters")
        else:
            rospy.logwarn("%s::%s : Map parameters not set. Using defaults"%(rospy.get_name, "obstacle_map"))
            params = {}

        # Setting Defaults
        if "x_min" in params:
            self.x_min = params["x_min"]
        else:
            self.x_min = -1.0

        if "x_max" in params:
            self.x_max = params["x_max"]
        else:
            self.x_max = 1.0

        if "y_min" in params:
            self.y_min = params["y_min"]
        else:
            self.y_min = -1.0

        if "y_max" in params:
            self.y_max = params["y_max"]
        else:
            self.y_max = 1.0

        if "z_min" in params:
            self.z_min = params["z_min"]
        else:
            self.z_min = 0.1

        if "z_max" in params:
            self.z_max = params["z_max"]
        else:
            self.z_max = 1.1

        if "resolution" in params:
            self.resolution = params["resolution"]
        else:
            self.resolution = 0.05

        self.map_x_range = int((self.x_max - self.x_min) / self.resolution) + 2
        self.map_y_range = int((self.y_max - self.y_min) / self.resolution) +2

    def depth_map_callback(self, map, source):
        self.depth_source_cache[source].append(map)
        self.depth_source_map[source] = np.average(np.asarray(self.depth_source_cache[source]), axis=0)
        # Debug
        # print(map.shape, source, self.depth_source_map[source].shape)

    def footprint_callback(self, footprint):
        if(footprint.shape != (self.map_x_range, self.map_y_range)):
            rospy.logerr("%s::%s : Footprint size mismatch with obstacle map"%(rospy.get_name(), "ObstacleMap"))
        else:
            self.footprint = footprint
       
    def publish_map(self):
        source_count = 0
        obstacle_map = np.zeros((self.map_x_range, self.map_y_range))
        for source in self.depth_source_map:
            obstacle_map += self.depth_source_map[source]
            source_count += 1
        # Normalizing
        if (source_count>0):
            # Normalizing
            np.clip(a=obstacle_map, a_min=0, a_max=100, out=obstacle_map)

             # Filtering footprint area
            if not(self.footprint is None):
                obstacle_map = obstacle_map * np.invert(self.footprint)

            map_msg = OccupancyGrid()
            map_msg.header.frame_id = "base_link"
            map_msg.header.seq = self.seq
            self.seq += 1
            map_msg.header.stamp = rospy.get_rostime()

            map_msg.info.resolution = self.resolution
            map_msg.info.width = self.map_y_range
            map_msg.info.height = self.map_x_range
            map_msg.info.origin.position.x = -self.map_x_range * self.resolution / 2
            map_msg.info.origin.position.y = -self.map_y_range * self.resolution / 2
            map_msg.info.origin.orientation.w = 1.0

            map_msg.data = obstacle_map.astype(int).T.reshape(-1,)
            self.map_publisher.publish(map_msg)
        else:
            rospy.logwarn_throttle(1.0, "%s::%s : No obstacle map received yet from any configured sources"%(rospy.get_name(), "ObstacleMap"))

class ClassTimer():
        def __init__(self, callback, interval):
            self.callback = callback
            self.interval = interval
            self.timer = Timer(self.interval, self.cb)
            self.timer.start()

        def cb(self):
            self.callback()
            self.timer = Timer(self.interval, self.cb)
            if rospy.is_shutdown():
                self.stop()
            else:
                self.timer.start()
    
        def stop(self, *args):
            self.timer.cancel()

if __name__ == '__main__':
    rospy.init_node('obstacle_map')
    rospy.loginfo("Obstacle Map Started")
    try:
        map = ObstacleMap()
    except Exception as ex:
        rospy.logwarn("Obstacle map error error" + str(ex))

    rospy.spin()

