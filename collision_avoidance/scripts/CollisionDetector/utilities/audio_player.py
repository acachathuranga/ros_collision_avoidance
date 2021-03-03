from playsound import playsound
from multiprocessing import Process
import subprocess
import threading
import rospy
import time
import os
import sys

class AudioPlayer():
        def __init__(self, path):
            self.path = path
            self.track = ""
            self.current_track = ""
            self.player = None

            self.lock = threading.Lock()
            self.condition = threading.Condition()

            self.handler = threading.Thread(target=self.run)
            self.handler.start()

        def run(self):
            while not(rospy.is_shutdown()):
                if (self.current_track != self.track):
                    # Terminate existing players
                    if not(self.player is None):
                        try:
                            self.player.terminate()
                        except Exception as ex:
                            print(str(ex))
                    if (self.track != ""):
                     # Start New Player
                        audio_file = os.path.join(self.path, self.track + ".mp3")
                        self.player = subprocess.Popen(['mpg123', '--loop', '-1', '-q', audio_file], stdout=sys.stdout, stderr=sys.stderr)

                    self.current_track = self.track
                time.sleep(1.0)
            # Terminate existing players
            if not(self.player is None):
                try:
                    self.player.terminate()
                except Exception as ex:
                    print(str(ex))

        def stop(self):
            self.track = ""
        
        def start(self, file_name):
            audio_file = os.path.join(self.path, file_name + ".mp3")
            if (os.path.isfile(audio_file)):
                    while not(rospy.is_shutdown()):
                        playsound(audio_file)
            else:
                rospy.logerr(audio_file + " File Not found")
            
        def play(self, track):
            self.track = track

if __name__ == '__main__':
    import time
    audio = AudioPlayer("/home/ros/user_ros_ws/src/robot_obstacle_safety/collision_avoidance/media")
    audio.play("moving")
    time.sleep(5)
    audio.stop()