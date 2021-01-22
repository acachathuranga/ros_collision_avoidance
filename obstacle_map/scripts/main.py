#! /usr/bin/env python
import rospy
from ObstacleMap.obstacle_map import ObstacleMap

if __name__ == '__main__':
    rospy.init_node('obstacle_map')
    rospy.loginfo("Obstacle Map Publisher Running")
    try:
        obstacle_map = ObstacleMap()
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex))
    rospy.spin()
