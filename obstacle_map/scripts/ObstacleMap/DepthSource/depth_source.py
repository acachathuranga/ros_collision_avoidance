#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy import point_cloud2 as ros_pcl
from tf import transformations, TransformListener
import numpy as np

class DepthSource():
    def __init__(   self,
                    name,
                    topic_name, 
                    sensor_frame, 
                    parent_frame, 
                    rate=5,
                    callback=None,
                    x_min=-1.0,
                    x_max=1.0,
                    y_min=-1.0,
                    y_max=1.0,
                    z_min=0.2,
                    z_max=1.1 ,
                    resolution=0.05,
                    probability_threshold=1):

        """
            Depth Source Plugin for Obstacle Map

            This class subscribes to a depth sensor topic (sensor_msgs/PointCloud2)
            and produces a 2D depth map, in robot's base frame.
            
            :param name         : Unique name for depth source
            :param topic_name   : Depth source topic (sensor_msgs/PointCloud2)
            :param sensor_frame : The frame in which point cloud data is being published
            :param parent_frame : Robot base frame name
            :param rate         : Depth cloud update rate (5Hz)
            :callback           : Callback function to receive 2D depth_map (numpy 2D) and source name.
            :param x_min        : Depth map boudary x_min
            :param x_max        : Depth map boudary x_max
            :param y_min        : Depth map boudary y_min
            :param y_max        : Depth map boudary y_max
            :param z_min        : Depth map boudary z_min
            :param z_max        : Depth map boudary z_max
            :parm resolution    : Depth map resolution
        """

        # Global attributes
        self.name = name
        self.sensor_frame = sensor_frame
        self.parent_frame = parent_frame
        self.callback = callback
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        self.resolution = resolution
        self.probability_threshold = probability_threshold

        # Map size calucation (considering boundary indexes)
        self.map_x_range = int((x_max - x_min) / resolution) + 2
        self.map_y_range = int((y_max - y_min) / resolution) + 2
        self.map_z_range = int((z_max - z_min) / resolution) + 2

        self.transform_initialized = False

        self.depth_sub = rospy.Subscriber(topic_name, PointCloud2, self.depth_cb)
        self.update_interval = 1.0 / rate
        self.update_stamp = rospy.get_time()

    def initalize_transform(self):
        # Fetch base frame to sensor frame transform
        listener = TransformListener()
        # Initialization delay for the tf listener
        rospy.sleep(0.2) 
        while not(rospy.is_shutdown()):
            try:
                (trans, rot) = listener.lookupTransform(self.parent_frame, self.sensor_frame, rospy.Time(0))
                self.transform = transformations.quaternion_matrix(rot)
                self.transform[:3, 3] = trans
                self.transform_initialized = True
                break
            except Exception as ex:
                rospy.logwarn("%s::%s : Transform Init error : %s"%(rospy.get_name(), self.name, str(ex)))
                rospy.sleep(1.0)

    def depth_cb(self, msg):
        # Initialize Transform if not initialized
        if not self.transform_initialized:
            self.initalize_transform()

        if ((rospy.get_time() - self.update_stamp) > self.update_interval):
            self.update_stamp = rospy.get_time()

            # TODO Remove debug line
            start = rospy.get_time()
        
            cloud_array = ros_pcl.pointcloud2_to_array(msg)
            depth_array = ros_pcl.get_xyz_points(cloud_array).T
            ones = np.ones((1, depth_array.shape[1]))
            depth_vector = np.vstack((depth_array, ones))

            np.set_printoptions(precision=5)

            # Transforming points to base frame
            depth_points = np.dot(self.transform, depth_vector)

            # Filtering out far region
            mask =  (depth_points[0,:] > self.x_min) & \
                    (depth_points[0,:] < self.x_max) & \
                    (depth_points[1,:] > self.y_min) & \
                    (depth_points[1,:] < self.y_max) & \
                    (depth_points[2,:] > self.z_min) & \
                    (depth_points[2,:] < self.z_max)
            
            filtered_depth_points = depth_points[:3, mask]
            
            # Scaling depth map
            scaled_depth_points = np.divide(filtered_depth_points, self.resolution)
            # Offsetting depth_map
            scaled_depth_points[0,:] += (self.map_x_range/2)
            scaled_depth_points[1,:] += (self.map_y_range/2)
            # Downsampling and typecasting
            scaled_depth_points = np.round(scaled_depth_points).astype(int)

            # Inserting depth information to map
            # Uses Multidimensional array indexing. Does not add repetitive cloud points
            # depth_map = np.zeros((self.map_x_range, self.map_y_range, self.map_z_range))
            # depth_map[scaled_depth_points[0, :], scaled_depth_points[1, :], scaled_depth_points[2, :]] += 1
            # depth_map_2d = np.sum(depth_map, axis=2)
            # Uses ND Array unravelling
            depth_map_2d = np.zeros((self.map_x_range, self.map_y_range)) 
            np.add.at(depth_map_2d.reshape(-1), np.ravel_multi_index(scaled_depth_points[:2, :], depth_map_2d.shape), 1)

            # Normalizing cost values
            np.clip(a=depth_map_2d, a_min=0, a_max=100, out=depth_map_2d)

            # Filtering low probability cells
            depth_map_2d[depth_map_2d<self.probability_threshold] = 0
            
            if (self.callback is None):
                rospy.logwarn("%s : Depth source '%s' subscriber running without callback"%(rospy.get_name(), self.name ))
            else:
                self.callback(depth_map_2d, self.name)

            # TODO Remove debug line
            # rospy.loginfo("Depth Processed: %.3f ms"%((rospy.get_time()-start)*1000))
        
if __name__ == '__main__':
    rospy.init_node('DepthSouce')
    rospy.loginfo("Depth Source Unit Test Started")
    try:
        depth_source = DepthSource( name="front_depth_cam",
                                    topic_name="/front_depth_cam/depth_registered/points", 
                                    sensor_frame="front_depth_cam_color_optical_frame", 
                                    parent_frame="base_link")
    except Exception as e:
        rospy.logwarn("DepthSource error" + str(e))

    rospy.spin()

