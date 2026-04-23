#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):

        #Topics & Subs, Pubs
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')
        drive_topic = rospy.get_param('~drive_topic')

        # Read the algorithm parameter paramters form params.yaml
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.scan_beams = rospy.get_param('~scan_beams')
        self.k_p = rospy.get_param('~k_p')
        self.k_d = rospy.get_param('~k_d')
        self.max_steering_angle = rospy.get_param('~max_steering_angle')
        self.wheelbase = rospy.get_param('~wheelbase')
        self.d_lr_des = rospy.get_param('~CenterOffset')
        self.angle_br = rospy.get_param('~angle_br')   # laser frame (rad)
        self.angle_ar = rospy.get_param('~angle_ar')
        self.angle_al = rospy.get_param('~angle_al')
        self.angle_bl = rospy.get_param('~angle_bl')
        self.v_desired = rospy.get_param('~vehicle_velocity')
        self.d_stop = rospy.get_param('~stop_distance')
        self.d_tau = rospy.get_param('~stop_distance_decay')
        self.heading_beam_angle = rospy.get_param('~heading_beam_angle')

        # New params for Act 6
        self.safe_dist = rospy.get_param('~safe_distance', 2.0)
        self.n_r = rospy.get_param('~n_pts_r', 100.0)
        self.n_l = rospy.get_param('~n_pts_l', 100.0)
        self.angle_fov =  math.radians(rospy.get_param('~angle_fov', 70))  # half-FOV in base_link (rad)

        self.vel = 0.0
        # Add your subscribers for LiDAR scan and Odomotery here
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        # Add your publisher for Drive topic here
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def lidar_to_base_link(self, angle):
        # laser x-axis is opposite to base_link x-axis: rotate by -pi (real world)
        return (angle) % (2 * math.pi)

    def base_link_to_lidar(self, angle):
        # inverse: rotate by +pi
        return (angle) % (2 * math.pi)

    def lidar_callback(self, data):

        # Pre-process LiDAR data as necessary
        raw =  np.array(data.ranges, dtype=float)
        raw = np.where(np.isfinite(raw), raw, self.max_lidar_range)

        # Compute every beam's angle in BASE_LINK frame up front
        lidar_angles = data.angle_min + np.arange(len(raw)) * data.angle_increment
        bl_angles = self.lidar_to_base_link(lidar_angles)  # all in base_link now

        # Masks out beams outside the FOV by zeroing them
        proc = np.copy(raw)
        near_zero = (bl_angles <= self.angle_fov) | (bl_angles >= 2*math.pi - self.angle_fov)
        proc[~near_zero] = 0.0
        proc[proc <= self.safe_dist] = 0.0

        # Find the widest gap in front of vehicle (if no gap then stop)
        non_zero = np.where(proc > 0)[0]
        if len(non_zero) == 0:
            self._publish(0.0, 0.0)
            return

        best_len, best_start, best_end = 0, 0, 0
        cur_start = non_zero[0]
        for k in range(1, len(non_zero)):
            if non_zero[k] - non_zero[k - 1] > 1:
                if non_zero[k - 1] - cur_start > best_len:
                    best_len  = non_zero[k - 1] - cur_start
                    best_start, best_end = cur_start, non_zero[k - 1]
                cur_start = non_zero[k]
        if non_zero[-1] - cur_start > best_len:
            best_start, best_end = cur_start, non_zero[-1]

        # Find the Best Direction of Travel
        best_i = best_start + int(np.argmax(proc[best_start:best_end + 1]))
        theta_des = bl_angles[best_i]  
        # Set up the QP for finding the two parallel barrier lines
        # Sector boundaries from params.yaml are in laser frame.

        wall_angle_br = (self.angle_br + theta_des + math.pi) % (2 * math.pi)
        wall_angle_al = (self.angle_al + theta_des + math.pi) % (2 * math.pi)

        # Convert back to laser frame to index into data.ranges
        br_idx = int((wall_angle_br - data.angle_min) / data.angle_increment) % self.scan_beams
        al_idx = int((wall_angle_al - data.angle_min) / data.angle_increment) % self.scan_beams

        C = np.zeros((3, self.n_r + self.n_l + 2))
        b = np.ones(self.n_r + self.n_l + 2)
        b[self.n_r + self.n_l:] = -0.99

        # Right obstacle points
        for i in range(self.n_r):
            idx = (br_idx + 3 * i) % self.scan_beams
            r = raw[idx]
            bl_angle = bl_angles[idx]          # already in base_link
            px = r * math.cos(bl_angle)        # standard x = r*cos(theta)
            py = r * math.sin(bl_angle)        # standard y = r*sin(theta)
            C[0, i] =  px
            C[1, i] =  py
            C[2, i] =  1

        # Left obstacle points
        for j in range(self.n_l):
            idx = (al_idx + 3 * j) % self.scan_beams
            r = raw[idx]
            bl_angle = bl_angles[idx]
            px = r * math.cos(bl_angle)
            py = r * math.sin(bl_angle)
            C[0, self.n_r + j] = -px
            C[1, self.n_r + j] = -py
            C[2, self.n_r + j] = -1

        # s bound columns
        C[2, self.n_r + self.n_l] =  1   # s >= -0.99
        C[2, self.n_r + self.n_l + 1] = -1   # s <=  0.99

        G = np.diag([1.0, 1.0, 1e-4])
        a = np.zeros(3)

        # Solve the QP problem to find the barrier lines parameters w,b
        #rospy.loginfo("\nG:\n{}\na:\n{}\nC:\n{}\nb:\n{}".format(G, a, C, b))\
        rospy.loginfo(
            "\n--- QP GEOMETRY DEBUG ---\n"
            "theta_des (deg): {:.2f}\n"
            "wall_angle_br (deg): {:.2f} | br_idx: {}\n"
            "wall_angle_al (deg): {:.2f} | al_idx: {}\n"
            "Right Wall Pt 0 (x,y): ({:.2f}, {:.2f})\n"
            "Left Wall Pt 0 (x,y): ({:.2f}, {:.2f})\n"
            "-------------------------".format(
                math.degrees(theta_des),
                math.degrees(wall_angle_br), br_idx,
                math.degrees(wall_angle_al), al_idx,
                C[0, 0], C[1, 0], 
                -C[0, self.n_r], -C[1, self.n_r] 
            )
        )
    
        x = solve_qp(G, a, C, b, 0)[0]

        w = x[:2]
        s = x[2]

        # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
        w_r = w / (s - 1.0)
        w_l = w / (s + 1.0)

        d_r  = 1.0 / (np.linalg.norm(w_r) + 1e-12)
        d_l  = 1.0 / (np.linalg.norm(w_l) + 1e-12)
        wh_r = d_r * w_r
        wh_l = d_l * w_l

        d_lr       = d_l - d_r
        d_lr_tilde = d_lr - self.d_lr_des

        vs = max(abs(self.vel), 0.5)

        d_r_dot  = np.dot(np.array([vs, 0.0]), wh_r)
        d_l_dot  = np.dot(np.array([vs, 0.0]), wh_l)
        d_lr_dot = d_l_dot - d_r_dot        # How fast that difference is changing

        cos_alpha_r = np.dot(np.array([0.0,  1.0]), wh_r)
        cos_alpha_l = np.dot(np.array([0.0, -1.0]), wh_l)

        # Compute the steering angle command
        den = vs**2 * (cos_alpha_r + cos_alpha_l)
        if abs(den) < 1e-6:
            delta = 0.0
        else:
            delta = math.atan((-self.wheelbase / den) * (-self.k_p * d_lr_tilde - self.k_d * d_lr_dot))
        delta_c = max(-self.max_steering_angle), min(self.max_steering_angle, delta)
        #except ValueError:
            #delta_c = 0.0

        # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
        # Forward cone is centred on 0 in base_link
        idx_center = int(round(self.base_link_to_lidar(0.0) / data.angle_increment)) % self.scan_beams
        half_width = int(round(self.heading_beam_angle / data.angle_increment))
        idx_lo = max(0, idx_center - half_width)
        idx_hi = min(len(data.ranges) - 1, idx_center + half_width)

        d_ob = self.max_lidar_range
        for i in range(idx_lo, idx_hi + 1):
            r = data.ranges[i]
            if not (math.isinf(r) or math.isnan(r)) and r > 0.0:
                d_ob = min(d_ob, r)

        v_cmd = self.v_desired * (1.0 - math.exp(-max(d_ob - self.d_stop, 0.0) / self.d_tau))

        # Publish the steering and speed commands to the drive topic
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = v_cmd
        drive_msg.drive.steering_angle = delta_c
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, odom_msg):
        self.vel = odom_msg.twist.twist.linear.x

def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)