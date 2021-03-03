#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import threading

class Indicator():
    def __init__(self):
        """
            Emits indications on obstacle avoidance status
            :rosparam topic: Light sensor topic (std_msgs/Int32)
        """
        # Global attributes
        self.ON = {"RED":[0], "GREEN":[2], "YELLOW":[4], "BLINK":[6], "NORMAL":[2], "WARNING":[2,6], "CRITICAL":[4], "ERROR":[0]}
        self.OFF = {"RED":[1], "GREEN":[3], "YELLOW":[5], "BLINK":[5], "NORMAL":[3], "WARNING":[3,5], "CRITICAL":[5], "ERROR":[1]}

        # Indicator topic
        topic = rospy.get_param(rospy.get_name() + "/indicator_topic", "/tower_lights_cmd")
        # Namespace fixing
        if (topic[0] != '/'): topic = rospy.get_name() + "/" + topic

        # Starting publisher
        self.indicator_publisher = rospy.Publisher(topic, Int32, queue_size=100)
        rospy.sleep(0.8)    # Publisher initialization tiom

        # Turn off all indications
        for state in self.OFF:
            for cmd in self.OFF[state]:
                self.publish_cmd(cmd)
        
        # Start indicator thread
        self.event = threading.Condition()
        thread = threading.Thread(target=self.indicator_thread)
        thread.start()

        # Initialize default indication
        self.current_indication = "NORMAL"
        self.indication = "NORMAL"
        for i in self.ON[self.current_indication]:
            self.publish_cmd(i)

    def indicator_thread(self):
        while not(rospy.is_shutdown()):
            with self.event:
                # Thread wait
                self.event.wait(2)
                if (self.current_indication != self.indication):
                    for i in self.OFF[self.current_indication]:
                        self.publish_cmd(i)
                    for i in self.ON[self.indication]:
                        self.publish_cmd(i)
                    self.current_indication = self.indication

    def publish_cmd(self, cmd):
        rospy.sleep(0.01)
        self.indicator_publisher.publish(cmd)

    def send_indication(self, indication):
        self.indication = indication
        with self.event:
            self.event.notifyAll()
            
    
    def warning(self):
        self.send_indication("WARNING")
        
    
    def error(self):
        self.send_indication("ERROR")

    def critical(self):
        self.send_indication("CRITICAL")

    def normal(self):
        self.send_indication("NORMAL")

if __name__ == "__main__":
    rospy.init_node('collision_status_indicator')
    rospy.loginfo("Collision Indicator Started")
    indicator = Indicator()
    print("Normal")
    indicator.normal()
    rospy.sleep(5)
    print("Warning")
    indicator.warning()
    rospy.sleep(5)
    print("Critical")
    indicator.critical()
    rospy.sleep(5)
    print("Error")
    indicator.error()
    rospy.spin()