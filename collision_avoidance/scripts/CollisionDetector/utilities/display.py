#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import threading

class Display():
    def __init__(self):
        """
            Communication for Displaying obstacle status on tablet display
        """

        # MQTT parameters
        address = rospy.get_param(rospy.get_name() + "/mqtt_address", "localhost")
        self.topic = rospy.get_param(rospy.get_name() + "/mqtt_topic", "obstacle_proximity")
        self.obstacle_msg = rospy.get_param(rospy.get_name() + "/mqtt_obstacle_msg", "obstacle")
        self.obstacle_clear_msg = rospy.get_param(rospy.get_name() + "/mqtt_obstacle_free_msg", "clear")
        

        # Starting Connection
        self.client = mqtt.Client()
        rospy.loginfo("Connecting to Mqtt broker")
        self.client.connect(address)
        self.client.loop_start()
        rospy.sleep(0.8)    # Publisher initialization tiom
        
        # Start display thread
        self.event = threading.Condition()
        thread = threading.Thread(target=self.display_thread)
        thread.start()

        # Initialize default indication
        self.current_indication = ""
        self.no_obstacle()
        
        rospy.loginfo("MQTT broker connected")

    def display_thread(self):
        while not(rospy.is_shutdown()):
            with self.event:
                # Thread wait
                self.event.wait(5)
                if (self.current_indication != self.indication):
                    self.display_indication(self.indication)
                    self.current_indication = self.indication

    def display_indication(self, status):
        # rospy.sleep(0.05)
        # rospy.logdebug("Display set to: %s" %status)
        self.client.publish(self.topic, status)

    def display_obstacle(self, status):
        if status:
            self.indication = self.obstacle_msg
        else:
            self.indication = self.obstacle_clear_msg
        with self.event:
            self.event.notifyAll()
            
    
    def no_obstacle(self):
        self.display_obstacle(False)
        
    
    def obstacle(self):
        self.display_obstacle(True)

    

if __name__ == "__main__":
    rospy.init_node('collision_status_display')
    rospy.loginfo("Collision Display Started")
    display = Display()
    print("No Obstacle")
    display.no_obstacle()
    rospy.sleep(5)
    print("Obstacle")
    display.obstacle()
    rospy.spin()