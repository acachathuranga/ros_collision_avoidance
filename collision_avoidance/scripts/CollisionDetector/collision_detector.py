#! /usr/bin/env python

import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from utilities.robot_footprint_reader import FootPrint
from utilities.spatial_tools import MatrixExp6, VecTose3
from utilities.status_indicator import Indicator
from utilities.audio_player import AudioPlayer
from utilities.display import Display
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Joy
import numpy as np
import os

class CollisionDetector():
    DEBUG = False
    TELEOP_ENABLED = True

    def __init__(self):
        map_topic = rospy.get_param(rospy.get_name() + "/map_topic", "/obstacle_map")
        cmd_vel_input_topic = rospy.get_param(rospy.get_name() + "/cmd_vel_input", "/cmd_vel_muxed")
        cmd_vel_output_topic = rospy.get_param(rospy.get_name() + "/cmd_vel_output", "/cmd_vel_safe")
        self.slowing_look_ahead_time = rospy.get_param(rospy.get_name() + "/slowing_look_ahead_time", 2.0)
        self.stopping_look_ahead_time = rospy.get_param(rospy.get_name() + "/stopping_look_ahead_time", 0.4)
        collision_status_topic = rospy.get_param(rospy.get_name() + "/collision_status", "collision_imminent")
        collision_avoidance_enable_service = rospy.get_param(rospy.get_name() + "/avoidance_enable", "enable_avoidance")

        # Namespace fixing
        if (map_topic[0] != '/'): map_topic = rospy.get_name() + "/" + map_topic
        if (cmd_vel_input_topic[0] != '/'): cmd_vel_input_topic = rospy.get_name() + "/" + cmd_vel_input_topic
        if (cmd_vel_output_topic[0] != '/'): cmd_vel_output_topic = rospy.get_name() + "/" + cmd_vel_output_topic
        if (collision_status_topic[0] != '/'): collision_status_topic = rospy.get_name() + "/" + collision_status_topic
        if (collision_avoidance_enable_service[0] != '/'): collision_avoidance_enable_service = rospy.get_name() + "/" + collision_avoidance_enable_service

        # Indicator
        self.indicator = Indicator()
        # Display
        self.display = Display()

        # Audio
        self.audio = AudioPlayer(os.path.join(rospkg.RosPack().get_path('collision_avoidance'), 'media'))

        self.obstacle_map = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.cmd_vel_sub = rospy.Subscriber(cmd_vel_input_topic, Twist, self.cmd_vel_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_output_topic, Twist, queue_size=100)

        # Collision status publisher
        self.collision_status_publisher = rospy.Publisher(collision_status_topic, Bool, queue_size=100)

        # Enable collision avoidance
        self.enable_avoidance = True

        # Collision avoidance enable / disable service
        self.enabler = rospy.Service(collision_avoidance_enable_service, SetBool, self.enabler_callback)

        # Map size initialization
        self.map_initialized = False

        # Footprint initializaton
        self.footprint_initialized = False

        # Look ahead points and cost array
        step_size = 0.2
        self.look_ahead_steps = np.arange(start=self.stopping_look_ahead_time, stop=self.slowing_look_ahead_time+step_size, step=step_size)
        self.look_ahead_step_velocity_scale = (self.look_ahead_steps - self.stopping_look_ahead_time) / self.slowing_look_ahead_time

        # Wait for map initialization
        while not rospy.is_shutdown():
            if not self.map_initialized:
                rospy.loginfo("%s::%s : Waiting for obstacle map"%(rospy.get_name(), "CollisionDetector"))
                rospy.sleep(1.0)
            else:
                rospy.loginfo("%s::%s : Obstacle Map Received"%(rospy.get_name(), "CollisionDetector"))
                break

        # Footprint map listener
        self.footprint = None
        footprint_receiver = FootPrint(self.map_x_range, self.map_y_range, self.resolution, self.footprint_callback)

        if self.DEBUG:
            # Debug publisher
            self.debug_pub = rospy.Publisher("debug_map", OccupancyGrid, queue_size=20)
            self.debug_map = OccupancyGrid()
            self.debug_map.info.resolution = self.resolution
            self.debug_map.info.width = self.map_y_range
            self.debug_map.info.height = self.map_x_range
            self.debug_map.info.origin.position.x = -self.map_x_range * self.resolution / 2
            self.debug_map.info.origin.position.y = -self.map_y_range * self.resolution / 2
            self.debug_map.info.origin.orientation.w = 1.0

        if self.TELEOP_ENABLED:
            # Teleop Enable / Disable with  Joystick
            teleop_topic = rospy.get_param(rospy.get_name() + "/joy_topic", "/joy1/joy")
            self.teleop_button = rospy.get_param(rospy.get_name() + "/enable_joy_button", 10)
            # Namespace fixing
            if (teleop_topic[0] != '/'): teleop_topic = rospy.get_name() + "/" + teleop_topic
            self.joy_sub = rospy.Subscriber(teleop_topic, Joy, self.teleop_callback)
    
    def teleop_callback(self, msg):
        if msg.buttons[self.teleop_button]:
            self.enable_avoidance = (not self.enable_avoidance)
            if self.enable_avoidance:
                state = "Enabled"
            else:
                state = "Disabled"
            rospy.loginfo("%s : Collision Avoidance %s"%(rospy.get_name(), state))

    def enabler_callback(self, req):
        self.enable_avoidance = req.data
        if self.enable_avoidance:
            state = "Enabled"
        else:
            state = "Disabled"
        rospy.loginfo("%s : Collision Avoidance %s"%(rospy.get_name(), state))

        resp = SetBoolResponse()
        resp.success = True
        resp.message = "Collision Avoidance " + state + " successfully."
        return resp

    def map_callback(self, map_msg):
        if not (self.map_initialized):
            self.map_x_range = map_msg.info.height
            self.map_y_range = map_msg.info.width
            self.resolution = map_msg.info.resolution

            self.map_initialized = True

        self.map = np.array(map_msg.data).reshape(self.map_y_range, self.map_x_range).T
        


    def footprint_callback(self, footprint):
        # Halt other threads from reading footprint while being updated
        self.footprint_initialized = False

        # Converting grid map to a point list
        fp = np.argwhere(footprint).T
        # Converting footprint cell values to meters
        self.footprint = np.zeros(fp.shape)
        self.footprint[0, :] = (fp[0, :] - self.map_x_range / 2) * self.resolution
        self.footprint[1, :] = (fp[1, :] - self.map_y_range / 2) * self.resolution
        
        # Converting into homogenous vector array
        self.footprint = np.vstack((self.footprint, np.zeros(fp.shape[1])))
        self.footprint = np.vstack((self.footprint, np.ones(fp.shape[1])))
        self.footprint_initialized = True

    def cmd_vel_callback(self, cmd_vel):
        # If robot footprint is not initialized, ignore command velocity
        if not (self.footprint_initialized):
            return 

        # If collision avoidance is disabled, relay original command velocity
        if not self.enable_avoidance:
            # Audio Notification
            if ((np.fabs(cmd_vel.linear.x) + np.fabs(cmd_vel.angular.z)) > 0.001):
                self.audio.play("moving")
            else :
                self.audio.stop()

            self.cmd_vel_pub.publish(cmd_vel)
            self.indicator.error()
            self.display.obstacle()
            return

        # Perform obstacle avoidance

        vel_vector = np.zeros((6))
        vel_vector[2] = cmd_vel.angular.z
        vel_vector[3] = cmd_vel.linear.x
        vel_se3 = VecTose3(vel_vector)

        velocity_scale = 1

        for step, time in enumerate(self.look_ahead_steps):
            T = MatrixExp6(vel_se3 * time)

            footprint_projection = np.dot(T, self.footprint)
        
            # Coverting footprint projection to cell values
            footprint_projection[0, :] = np.round(footprint_projection[0, :] / self.resolution + self.map_x_range / 2)
            footprint_projection[1, :] = np.round(footprint_projection[1, :] / self.resolution + self.map_y_range / 2)

            mask =  (footprint_projection[0,:] >= 0) & \
                    (footprint_projection[0,:] < self.map_x_range) & \
                    (footprint_projection[1,:] >= 0) & \
                    (footprint_projection[1,:] < self.map_y_range)

            # Filtering out of boundary values and Type casting
            footprint_projection = footprint_projection[:2, mask].astype(dtype=int)

            cost = np.sum(self.map[footprint_projection[0, :], footprint_projection[1, :]])

            if (cost > 0):
                # Update velocity scale according to obstacle proximity
                velocity_scale = self.look_ahead_step_velocity_scale[step]

                if self.DEBUG:
                    # DEBUG print
                    rospy.logwarn("Collision in %.2f second"%time)
                    map_data = np.zeros((self.map_x_range, self.map_y_range))
                    map_data[footprint_projection[0, :].astype(dtype=int), footprint_projection[1, :].astype(dtype=int)] = 40
                    self.debug_map.header.frame_id = "base_link"
                    self.debug_map.header.stamp = rospy.get_rostime()
                    self.debug_map.data = map_data.astype(int).T.reshape(-1,)
                    self.debug_pub.publish(self.debug_map)

                break
        
        # Collision imminent
        if (velocity_scale > 0.9):
            self.indicator.normal()
            self.display.no_obstacle()
        elif (velocity_scale > 0.01):
            self.indicator.warning()
        else:
            status = Bool()
            status.data = True
            self.collision_status_publisher.publish(status)
            self.indicator.critical()
            self.display.obstacle()

        cmd_vel.angular.z = cmd_vel.angular.z * velocity_scale
        cmd_vel.linear.x = cmd_vel.linear.x * velocity_scale

        # Audio Notification
        if (velocity_scale > 0.01):
            if ((np.fabs(cmd_vel.linear.x) + np.fabs(cmd_vel.angular.z)) > 0.001):
                self.audio.play("moving")
            else :
                self.audio.stop()
        else:
            self.audio.play("obstacle")
        
        self.cmd_vel_pub.publish(cmd_vel)




if __name__ == '__main__':
    rospy.init_node('collision_detector')
    rospy.loginfo("Collision Detector Started")
    try:
        detector = CollisionDetector()
    except Exception as ex:
        rospy.logwarn("Collision Detector error" + str(ex))

    rospy.spin()

