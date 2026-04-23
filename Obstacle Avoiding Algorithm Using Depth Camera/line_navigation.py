#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry




class WallFollow:
    def __init__(self):

        # Read the Wall-Following controller paramters form params.yaml
        # ...
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')
        drive_topic = rospy.get_param('~drive_topic')

        self.max_lidar_range = rospy.get_param('~scan_range')
        self.scan_beams      = rospy.get_param('~scan_beams')

        self.k_p = rospy.get_param('~k_p') #kp corrects position error
        self.k_d = rospy.get_param('~k_d') #kd corrects rate of change of error (damping)

        self.max_steering_angle = rospy.get_param('~max_steering_angle')
        
        self.wheelbase = rospy.get_param('~wheelbase')

        self.d_lr_des = rospy.get_param('~CenterOffset')

        # --- LiDAR beam angles for wall estimation (radians)( ---
        # Right wall: beams b_r and a_r)
        self.angle_br = rospy.get_param('~angle_br')
        self.angle_ar = rospy.get_param('~angle_ar')
        # Left wall:  beams b_l and a_l
        self.angle_bl = rospy.get_param('~angle_bl')
        self.angle_al = rospy.get_param('~angle_al')

        # --- Velocity control parameters ---
        self.v_desired = rospy.get_param('~vehicle_velocity')   # nominal speed  (m/s)
        self.d_stop = rospy.get_param('~stop_distance')   # stop distance  (m)
        self.d_tau = rospy.get_param('~stop_distance_decay')  # decay param  (m)

        self.heading_beam_angle = rospy.get_param('~heading_beam_angle')

        self.tau = rospy.get_param('~tau')

        self.vel = 0.0   # actual vehicle speed from odometry

        # Subscrbie to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Create a publisher for the Drive topic
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

     # The LiDAR callback function is where you read LiDAR scan data as it becomes availble and compute the vehile veloicty and steering angle commands
    
    def lidar_callback(self, data):      

      # Exttract the parameters of two walls on the left and right side of the vehicles. Referrring to Fig. 1 in the lab instructions, these are al, bl, thethal, ... 
      # convert beam angles from rad tto integers
        b_r_index = int(self.angle_br / data.angle_increment) % 720
        a_r_index = int(self.angle_ar/ data.angle_increment) % 720
        b_l_index = int(self.angle_bl / data.angle_increment) % 720
        a_l_index = int(self.angle_al / data.angle_increment) % 720 

        # reads range at each index
        b_r = min(data.ranges[b_r_index], self.max_lidar_range)
        a_r = min(data.ranges[a_r_index], self.max_lidar_range)
        b_l = min(data.ranges[b_l_index], self.max_lidar_range)
        a_l = min(data.ranges[a_l_index], self.max_lidar_range)

        # angular seperation between two beams on each side
        theta_r = self.angle_ar - self.angle_br
        theta_l = self.angle_bl - self.angle_al

        # Wall orientation angles (Eq. 12, 13)
        beta_r = math.atan2(a_r * math.cos(theta_r) - b_r, a_r * math.sin(theta_r))
        beta_l = math.atan2(a_l * math.cos(theta_l) - b_l, a_l * math.sin(theta_l))

        # Vehicle heading angles w.r.t. walls (Eq. 16, 17)
        alpha_r = beta_r + math.pi / 2.0 - self.angle_br
        alpha_l = -beta_l + 3.0 * math.pi / 2.0 - self.angle_bl

        # Perpendicular distances to walls (Eq. 18, 19)
        d_r = b_r * math.cos(beta_r)
        d_l = b_l * math.cos(beta_l)

        # --- d_lr and its low-pass filtered derivative (Eq. 3) ---
        d_lr = d_l - d_r
        d_lr_dot = -self.vel * math.sin(alpha_l) - self.vel * math.sin(alpha_r)
      # Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
      # ...  
        # --- Steering angle command (Eq. 8) ---
        d_lr_err = d_lr - self.d_lr_des
        vs = max(abs(self.vel), 0.5)
        delta = math.atan((-self.wheelbase / (vs**2 * (math.cos(alpha_r) + math.cos(alpha_l)))) * (-self.k_p * d_lr_err - self.k_d * d_lr_dot))
        # Clamp to physical steering limit (Eq. 11)
        delta_c = max(-self.max_steering_angle, min(self.max_steering_angle, delta))
      # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
      #  ...     
        #idx_center = int(round((math.pi - data.angle_min) / data.angle_increment))
        idx_center = int(round((0.0 - data.angle_min) / data.angle_increment))
        half_width = int(round(self.heading_beam_angle / data.angle_increment))
        idx_lo = max(0, idx_center - half_width)
        idx_hi = min(len(data.ranges) - 1, idx_center + half_width)

        d_ob = self.max_lidar_range
        for i in range(idx_lo, idx_hi + 1):
            r = data.ranges[i]
            if not (math.isinf(r) or math.isnan(r)) and r > 0.0:
                d_ob = min(d_ob, r)

        # Velocity command (Eq. 20)
        v_cmd = self.v_desired * (1.0 - math.exp(-max(d_ob - self.d_stop, 0.0) / self.d_tau))

      # Publish steering angle and velocity commnads to the Drive topic
      # ...
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = delta_c
        drive_msg.drive.speed = v_cmd
        self.drive_pub.publish(drive_msg) 

    # The Odometry callback reads the actual vehicle velocity from VESC. 
    
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
