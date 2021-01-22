#! /usr/bin/env python
import rospy
from CollisionDetector.collision_detector import CollisionDetector

if __name__ == '__main__':
    rospy.init_node('collision_avoidance_guard')
    rospy.loginfo("Collision avoidance guard running!")
    try:
        avoidance = CollisionDetector()
    except Exception as ex:
        rospy.logwarn(rospy.get_name() + " : " + str(ex))
    rospy.spin()
