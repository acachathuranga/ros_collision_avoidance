#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
publisher = None

def repub():
    global publisher
    rospy.init_node('cost_map_repub')
    publisher = rospy.Publisher('cost_map_enhanced', OccupancyGrid, queue_size=10)
    subscriber = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, callback)
    rospy.loginfo("Costmap Republisher Running")
    rospy.spin()

def callback(msg):
    map_data = np.clip(a=(np.array(msg.data) * 10), a_min=0, a_max=100, )
    msg.data = map_data.reshape(-1,)
    publisher.publish(msg)

if __name__ == '__main__':
    try:
        repub()
    except rospy.ROSInterruptException:
        pass