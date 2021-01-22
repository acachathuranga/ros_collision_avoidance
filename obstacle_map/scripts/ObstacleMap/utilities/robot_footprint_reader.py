#! /usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import numpy as np
import cv2

class FootPrint():
    def __init__(self, x_range, y_range, resolution=0.05, callback=None):
        """
            Subscribes to a footprint topic and publishes a numpy grid map
            The footprint area will be filled with '1' 
            Non footprint grid cells will be filled with '0'
            
            Footprint center will be at the center of the map (x_range/2, y_range/2)
            :warning:       The map size must be larger than footprint size

            :param x_range: X distance of the grid map, in cell count
            :param y_range: Y distance of the grid map, in cell count
            :param resolution: Grid cell size in meters
            :param callback:  Callback function to receive a grid map of the footprint

        """
        # Global attributes
        self.x_range = x_range
        self.y_range = y_range
        self.resolution = resolution
        self.callback = callback

        self.footprint_points = None

        # Setting Default footprint
        default_footprint = rospy.get_param(rospy.get_name() + "/robot_footprint", "[[0.35,0.2],[0.35,-0.2],[-0.35,-0.2],[-0.35,0.2]]")

        # Initializing with default footprint
        fp_msg = String()
        fp_msg.data = default_footprint
        self.footprint_callback(fp_msg)

        # Starting footprint subscriber
        footprint_topic = rospy.get_param(rospy.get_name() + "/robot_footprint_topic", "/robot_footprint")
        # Namespace fixing
        if (footprint_topic[0] != '/'): footprint_topic = rospy.get_name() + "/" + footprint_topic
        self.sub = rospy.Subscriber(footprint_topic, String, self.footprint_callback)
    
    def footprint_callback(self, msg):
        
        points = np.array(json.loads(msg.data))

        # Scaling and offsetting footprint vertices
        points = np.floor(points / self.resolution)
        points[:, 0] += np.floor(self.x_range/2)
        points[:, 1] += np.floor(self.y_range/2)

        # Checking if all the points are within map boundaries
        if ( (np.max(points[:, 0]) >= self.x_range) or (np.max(points[:, 1]) >= self.y_range) or (np.min(points) < 0) ):
            rospy.logerr("%s::%s : Footprint larger than obstacle map. Map size [%d, %d, %d]. Footprint[%s]"%(rospy.get_name(), "FootprintReader", self.x_range, self.y_range, self.resolution, msg.data))
            return
        else:
            rospy.loginfo("%s::%s : Footprint received [%s]"%(rospy.get_name(), "FootprintReader",msg.data))

        footprint_map = np.zeros((int(self.y_range), int(self.x_range)))
        cv2.fillPoly(footprint_map, pts=[points.astype(dtype=int)], color=1)

        # Type casting and image rotation (in accordance with opencv x, y axes)
        footprint_map = footprint_map.T.astype(dtype=bool)

        if not (self.callback is None):
            self.callback(footprint_map)

if __name__ == "__main__":
    rospy.init_node("foot_print_reader")

    footprint = FootPrint(x_range=2.0, y_range=2.0, resolution=0.05)
    rospy.spin()