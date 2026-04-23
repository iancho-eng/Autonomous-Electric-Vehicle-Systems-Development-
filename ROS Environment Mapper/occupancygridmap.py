#!/usr/bin/env python

import numpy as np
import sys
import cv2
import time
import rospy
import tf2_ros
import math


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class OccupancyGridMap:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read parameters from params.yaml
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')

        self.t_prev = rospy.get_time()
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.scan_beams = rospy.get_param('~scan_beams')
        self.odom_frame = rospy.get_param('~odom_frame')
        self.lidar_offset = rospy.get_param('~scan_distance_to_base_link')

        # Read the map parameters from *.yaml file
        self.map_occ_grid_msg = OccupancyGrid()
        self.occ_map_topic = rospy.get_param('~occ_map_topic')
        self.p_occ = rospy.get_param('~p_occ')
        self.p_free = rospy.get_param('~p_free')
        self.map_width = rospy.get_param('~map_width')
        self.map_height = rospy.get_param('~map_height')
        self.map_res = rospy.get_param('~map_res')

        # Pre-compute log-odds for occupied and free
        self.log_p_occ = math.log(self.p_occ  / (1.0 - self.p_occ))
        self.log_p_free = math.log(self.p_free / (1.0 - self.p_free))

        # Current vehicle pose (x,y,theta)
        self.x_k = 0.0
        self.y_k = 0.0
        self.theta_k = 0.0

        # Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
        # ...
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_occ_grid_msg.header.frame_id = self.odom_frame
        self.map_occ_grid_msg.info.resolution = self.map_res
        self.map_occ_grid_msg.info.width = self.map_width
        self.map_occ_grid_msg.info.height = self.map_height

        # Set the lower-left corner of the map so the vehicle starts in the centre of the map
        self.map_occ_grid_msg.info.origin.position.x = -self.map_width  * self.map_res / 2
        self.map_occ_grid_msg.info.origin.position.y = -self.map_height * self.map_res / 2

        self.map_occ_grid_msg.info.origin.orientation.x = 0.0
        self.map_occ_grid_msg.info.origin.orientation.y = 0.0
        self.map_occ_grid_msg.info.origin.orientation.z = 0.0
        self.map_occ_grid_msg.info.origin.orientation.w = 1.0 # No rotation

        # Initialize the cell occuopancy probabilites to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown 
        self.log_p_map = [[0] * self.map_height for _ in range(self.map_width)] # when log-odds = 0 (event probability) --> probability = 0.5 (makes it so no division by 0)
        self.map_occ_grid_msg.data = [-1] * (self.map_width * self.map_height) # -1 in ROS = unknown

        # Subscribe to Lidar scan and odomery topics with corresponding lidar_callback() and odometry_callback() functions 
        # calls functions whenn new data arrives
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        # Create a publisher for the Occupancy Grid Map
        self.map_pub = rospy.Publisher(self.occ_map_topic, OccupancyGrid, queue_size=1)


    # lidar_callback () uses the current LiDAR scan and Wheel Odometry data to uddate and publish the Grid Occupancy map 
    def lidar_callback(self, data):

        # LiDAR frame origin location w.r.t. odom frame
        #xlidar=xrobot+dcos(theta) -->translation+rotation between frames
        lidar_odom_x = self.lidar_offset * math.cos(self.theta_k) + self.x_k 
        lidar_odom_y = self.lidar_offset * math.sin(self.theta_k) + self.y_k 

        # LiDAR frame yaw w.r.t. odom frame
        # LiDAR is mounted 180 degrees rotated from base_link
        theta_lidar_odom = (self.theta_k + math.pi) % (2 * math.pi)

        # Looping through every cell ij
        for j in range(self.map_height):
            for i in range(self.map_width):

                # Locating cell ij centre in odom coordinates (grid index --> physical position)
                cell_odom_x = (-self.map_width  * self.map_res / 2) + (i + 0.5) * self.map_res
                cell_odom_y = (-self.map_height * self.map_res / 2) + (j + 0.5) * self.map_res

                # Finding closest ray angle and length to this cell
                cell_angle  = (math.atan2(cell_odom_y - lidar_odom_y, cell_odom_x - lidar_odom_x) - theta_lidar_odom) % (2 * math.pi)
                cell_length = math.sqrt((cell_odom_y - lidar_odom_y)**2 + (cell_odom_x - lidar_odom_x)**2)

                # Find the closest ray index (poler --> cartesian)
                ray_index = int(cell_angle / data.angle_increment) % self.scan_beams

                # Closest ray in polar form
                ray_length = data.ranges[ray_index]
                ray_angle = data.angle_increment * ray_index

                # Closest ray in rectangular form (lidar frame)
                ray_lidar_x = ray_length * math.cos(ray_angle)
                ray_lidar_y = ray_length * math.sin(ray_angle)

                # Closest ray converted to odom frame (rotation matrix)
                ray_odom_x = lidar_odom_x + ray_lidar_x * math.cos(theta_lidar_odom) - ray_lidar_y * math.sin(theta_lidar_odom)
                ray_odom_y = lidar_odom_y + ray_lidar_y * math.sin(theta_lidar_odom) + ray_lidar_y * math.cos(theta_lidar_odom)

                # Check if ray hit point falls inside the cell (in odom frame) (if ray endoppint lies inside cell --> occupied)
                if (abs(ray_odom_x - cell_odom_x) <= 0.5 * self.map_res and abs(ray_odom_y - cell_odom_y) <= 0.5 * self.map_res):
                    in_cell = True
                else:
                    in_cell = False

                # Updating log probabilities (only for valid beam ranges)
                if data.range_min < ray_length < data.range_max:
                    if in_cell:
                        # Cell is occupied (hit cell)
                        self.log_p_map[i][j] += self.log_p_occ
                    elif cell_length < ray_length and not in_cell:
                        # Cell is free (between lidar and hit point) (before hit)
                        self.log_p_map[i][j] += self.log_p_free
                    else:
                        # Cell is unknown (behind hit point) (after hit)
                        self.log_p_map[i][j] += 0

                    # Clamp log probability to prevent overflow errors from repeated updates
                    self.log_p_map[i][j] = max(-20, min(20, self.log_p_map[i][j]))

                    # Convert log probability back to probability
                    p_cell = 1 - 1 / (1 + math.exp(self.log_p_map[i][j]))

                    # Construct grid message based on thresholds
                    if p_cell > self.p_occ:
                        # Cell occupied
                        self.map_occ_grid_msg.data[i + j * self.map_width] = 100
                    elif p_cell < self.p_free:
                        # Cell free
                        self.map_occ_grid_msg.data[i + j * self.map_width] = 0
                    else:
                        # Cell unknown
                        self.map_occ_grid_msg.data[i + j * self.map_width] = -1

        # Publish to map topic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_occ_grid_msg) # send to rviz


    # odom_callback() retrives the wheel odometry data from the publsihed odom_msg
    def odom_callback(self, odom_msg):

        # Obtaining base_link frame displacement from odom frame
        t = odom_msg.pose.pose.position # Both x and y
        self.x_k = t.x
        self.y_k = t.y

        # Obtaining base_link frame yaw from odom frame
        # Using quaternion yaw formula assuming no pitch or roll
        q = odom_msg.pose.pose.orientation
        self.theta_k = 2 * math.atan2(q.z, q.w)


def main(args):
    rospy.init_node("occupancygridmap", anonymous=True) # uses pose (x,y,theta), suffers from drift
    OccupancyGridMap() # uses LiDAR + pose, updates via log-odds
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)