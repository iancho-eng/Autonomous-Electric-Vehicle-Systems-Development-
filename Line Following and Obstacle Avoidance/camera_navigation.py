#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
from quadprog import solve_qp

import rospy
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class GapBarrierRGBD:
    def __init__(self):

        #Topics & Subs, Pubs
        lidarscan_topic  = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')
        drive_topic = rospy.get_param('~drive_topic')
        depth_topic = rospy.get_param('~depth_image_topic', '/camera/depth/image_rect_raw')
        caminfo_topic = rospy.get_param('~depth_info_topic','/camera/depth/camera_info')

        # Read the algorithm parameter paramters form params.yaml
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.scan_beams = rospy.get_param('~scan_beams')
        self.k_p = rospy.get_param('~k_p')
        self.k_d = rospy.get_param('~k_d')
        self.max_steering_angle = rospy.get_param('~max_steering_angle')
        self.wheelbase = rospy.get_param('~wheelbase')
        self.d_lr_des = rospy.get_param('~CenterOffset')
        self.angle_br = rospy.get_param('~angle_br')
        self.angle_ar = rospy.get_param('~angle_ar')
        self.angle_al = rospy.get_param('~angle_al')
        self.angle_bl = rospy.get_param('~angle_bl')
        self.v_desired = rospy.get_param('~vehicle_velocity')
        self.d_stop = rospy.get_param('~stop_distance')
        self.d_tau = rospy.get_param('~stop_distance_decay')
        self.heading_beam_angle = rospy.get_param('~heading_beam_angle')
        self.safe_dist = rospy.get_param('~safe_distance', 2.0)
        self.n_r = int(rospy.get_param('~n_pts_r', 33))
        self.n_l = int(rospy.get_param('~n_pts_l', 33))
        self.angle_fov = math.radians(rospy.get_param('~angle_fov', 70))

        # New params
        # D435 hard range limits from spec sheet
        self.depth_min = rospy.get_param('~depth_min',  0.3)
        self.depth_max = rospy.get_param('~depth_max',  3.0)  

        # D435 horizontal half-FOV
        # Only LiDAR beams inside this cone can ever be augmented by the camera
        self.cam_hfov_half = math.radians(rospy.get_param('~cam_hfov_half', 42.6))

        # Camera mounting in base_link frame
        self.cam_height = rospy.get_param('~cam_height', 0.15)
        self.cam_forward = rospy.get_param('~cam_forward', 0.05)  
        self.cam_tilt = rospy.get_param('~cam_tilt', 0.0) 

        # Height band in base_link that the LiDAR misses 
        self.obs_min_z = rospy.get_param('~obs_min_z', 0.05)
        self.obs_max_z = rospy.get_param('~obs_max_z', 0.30) 

        # Subsample every Nth row/col of the depth image to reduce computation
        self.depth_step = int(rospy.get_param('~depth_step', 4))

        self.vel = 0.0
        self.bridge = CvBridge()
        self.depth_image = None 

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Add your subscribers for LiDAR scan and Odomotery here
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(caminfo_topic, CameraInfo, self.caminfo_callback, queue_size=1)
        rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1)
        # Add your publisher for Drive topic here
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    # Frame helpers 
    def laser_to_base_link(self, angle):
        # laser x-axis is opposite to base_link x-axis: rotate by -pi (real world)
        return angle % (2 * math.pi) # Since simulator no shift

    def base_link_to_laser(self, angle):
        # inverse: rotate by + pi
        return angle % (2 * math.pi) # Since simulator no shift

    # Camera info callback — store intrinsics once
    def caminfo_callback(self, msg):
        if self.fx is None: 
            self.fx = msg.K[0]   # focal length x  (pixels)
            self.fy = msg.K[4]   # focal length y  (pixels)
            self.cx = msg.K[2]   # principal point x
            self.cy = msg.K[5]   # principal point y
            rospy.loginfo("D435 intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", self.fx, self.fy, self.cx, self.cy)

    # Depth callback just store the latest image.
    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
            # (uint16 millimeters) to convert to meters
            self.depth_image = np.array(raw, dtype=np.float32) * 0.001
            
            # Log the center pixel to prove it works
            h, w = self.depth_image.shape
            center_depth = self.depth_image[h//2, w//2]
            rospy.loginfo("[RAW CAMERA DATA] Received depth image: {}x{}pixels. center pixel depth: {:.3f} meters".format(w,h,center_depth))
            
        except Exception as e:
            # If it fails, log the EXACT error so we don't just get "bad callback"
            rospy.loginfo("Depth callback crashed: {}".format(e))

    # Plane fitting covariance determinant method.
    def _fit_plane(self, pts):
        # A plane requires at least 3 points to exist in 3D space.
        if len(pts) < 3:
            return None
        # Find the "center of mass" of the point cloud.
        centroid = np.mean(pts, axis=0)
        r  = pts - centroid
        # Calculate the elements of the 3x3 Covariance Matrix
        xx = np.sum(r[:,0]**2);  xy = np.sum(r[:,0]*r[:,1])
        xz = np.sum(r[:,0]*r[:,2]); yy = np.sum(r[:,1]**2)
        yz = np.sum(r[:,1]*r[:,2]); zz = np.sum(r[:,2]**2)

        # Calculate the determinants
        det_x = yy*zz - yz*yz
        det_y = xx*zz - xz*xz
        det_z = xx*yy - xy*xy
        det_max = max(det_x, det_y, det_z)
        if det_max <= 0.0:
            return None

        # Compute the unnormalized normal vector
        if det_max == det_x:
            n = np.array([det_x, xz*yz - xy*zz, xy*yz - xz*yy])
        elif det_max == det_y:
            n = np.array([xz*yz - xy*zz, det_y, xy*xz - yz*xx])
        else:
            n = np.array([xy*yz - xz*yy, xy*xz - yz*xx, det_z])
        # Normalize the vector
        n = n / (np.linalg.norm(n) + 1e-12)
        return n, centroid

    # Convert depth image - augmented LiDAR ranges
    def _get_augmented_ranges(self, raw_lidar, bl_angles):
        augmented = np.copy(raw_lidar)

        if self.depth_image is None or self.fx is None:
            return augmented   # camera not ready yet

        h, w = self.depth_image.shape

        # Subsample depth image
        rows = np.arange(0, h, self.depth_step)
        cols = np.arange(0, w, self.depth_step)
        rr, cc = np.meshgrid(rows, cols, indexing='ij')
        rr, cc = rr.ravel(), cc.ravel()
        Z = self.depth_image[rr, cc]

        # Discard pixels outside D435 reliable range
        valid = (Z >= self.depth_min) & (Z <= self.depth_max) & np.isfinite(Z)
        rr, cc, Z = rr[valid], cc[valid], Z[valid]
        if len(Z) == 0:
            return augmented

        # Pixel to camera-frame 3D
        X_cam = (cc - self.cx) * Z / self.fx # right
        Y_cam = (rr - self.cy) * Z / self.fy # down
        Z_cam = Z # forward

        # Camera frame to base_link frame
        phi = self.cam_tilt
        bl_x = Z_cam * math.cos(phi) + Y_cam * math.sin(phi) + self.cam_forward
        bl_y = -X_cam
        bl_z = -Z_cam * math.sin(phi) + Y_cam * (-math.cos(phi)) + self.cam_height

        pts3d = np.column_stack([bl_x, bl_y, bl_z])  # (N, 3)

        if len(pts3d) > 0:
            # Grab the point that is physically closest to the camera
            closest_idx = np.argmin(pts3d[:, 0]) # Find min X (forward distance)
            closest_pt = pts3d[closest_idx]
            
            rospy.loginfo_throttle(2.0, 
                "[3D PROJECTION] Successfully converted {} pixels to 3D space using camera intrinsics. "
                "Closest point is at (X:{:.2f}, Y:{:.2f}, Z:{:.2f}) meters.".format(
                    len(pts3d), closest_pt[0], closest_pt[1], closest_pt[2]
                )
            )

        # Fit ground plane using bottom rows of image (if enough points)
        ground_row_thresh = int(h * 0.7)
        ground_mask = rr >= ground_row_thresh
        if np.sum(ground_mask) >= 3:
            ground_result = self._fit_plane(pts3d[ground_mask])
        else:
            ground_result = None

        # Filter to the height band the LiDAR misses
        # Keep only points in [obs_min_z, obs_max_z] above the ground.
        height_ok = (pts3d[:, 2] >= self.obs_min_z) & (pts3d[:, 2] <= self.obs_max_z)

        if ground_result is not None:
            gn, gc = ground_result
            dist2gnd = np.abs(np.dot(pts3d - gc, gn))
            height_ok &= (dist2gnd > 0.05)

        height_ok &= (pts3d[:, 0] > 0.05)

        obs_bl_x = bl_x[height_ok]
        obs_bl_y = bl_y[height_ok]

        if len(obs_bl_x) == 0:
            return augmented

        # Project onto LiDAR horizontal plane
        range_2d  = np.sqrt(obs_bl_x**2 + obs_bl_y**2)
        angle_2d  = np.arctan2(obs_bl_y, obs_bl_x) % (2 * math.pi)  # base_link angle

        # Only augment beams within the D435 camera FOV
        # Any beam outside this window cannot have been seen by the camera so we skip it
        in_cam_fov = (angle_2d <= self.cam_hfov_half) | (angle_2d >= 2*math.pi - self.cam_hfov_half)

        angle_2d = angle_2d[in_cam_fov]
        range_2d = range_2d[in_cam_fov]

        # Inject into LiDAR scan
        for ang, rng in zip(angle_2d, range_2d):
            # Find nearest LiDAR beam to this camera obstacle angle
            diffs = np.abs(bl_angles - ang)
            diffs = np.minimum(diffs, 2*math.pi - diffs)  # handle wrap
            beam_idx = int(np.argmin(diffs))

            # Only inject if camera reports something CLOSER
            if rng < augmented[beam_idx]:
                augmented[beam_idx] = rng

        return augmented

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):
        # Pre-process LiDAR data as necessary
        raw = np.array(data.ranges, dtype=float)
        raw = np.where(np.isfinite(raw), raw, self.max_lidar_range)
        laser_angles = data.angle_min + np.arange(len(raw)) * data.angle_increment
        bl_angles = self.laser_to_base_link(laser_angles)

        # BONUS: augment with camera before any further processing.
        raw = self._get_augmented_ranges(raw, bl_angles)

        # Pre-process LiDAR data as necessary
        proc = np.copy(raw)
        near_fwd = (bl_angles <= self.angle_fov) | (bl_angles >= 2*math.pi - self.angle_fov)
        proc[~near_fwd] = 0.0
        proc[proc <= self.safe_dist] = 0.0

        # Find the widest gap in front of vehicle
        non_zero = np.where(proc > 0)[0]
        if len(non_zero) == 0:
            self._publish(0.0, 0.0)
            return
        # Find the longest contiguous sequence of non-zero beams, which corresponds to the widest gap.
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

        wall_angle_br = (self.angle_br + theta_des + math.pi) % (2 * math.pi)
        wall_angle_al = (self.angle_al + theta_des + math.pi) % (2 * math.pi)

        # Convert back to laser frame to index into data.ranges
        br_idx = int((wall_angle_br - data.angle_min) / data.angle_increment) % self.scan_beams
        al_idx = int((wall_angle_al - data.angle_min) / data.angle_increment) % self.scan_beams

        # Set up the QP for finding the two parallel barrier lines
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
            C[0, i] = px
            C[1, i] = py
            C[2, i] = 1

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
        x = solve_qp(G, a, C, b, 0)[0]

        w = x[:2]
        s = x[2]

        # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
        w_r = w / (s - 1.0)
        w_l = w / (s + 1.0)

        d_r = 1.0 / (np.linalg.norm(w_r) + 1e-12)
        d_l = 1.0 / (np.linalg.norm(w_l) + 1e-12)
        wh_r = d_r * w_r
        wh_l = d_l * w_l

        d_lr = d_l - d_r
        d_lr_tilde = d_lr - self.d_lr_des

        vs = max(abs(self.vel), 0.5)

        d_r_dot = np.dot(np.array([vs, 0.0]), wh_r)
        d_l_dot = np.dot(np.array([vs, 0.0]), wh_l)
        d_lr_dot = d_l_dot - d_r_dot

        cos_alpha_r = np.dot(np.array([0.0,  1.0]), wh_r)
        cos_alpha_l = np.dot(np.array([0.0, -1.0]), wh_l)

        # Compute the steering angle command
        den = vs**2 * (cos_alpha_r + cos_alpha_l)
        if abs(den) < 1e-6:
            delta = 0.0
        else:
            delta = math.atan((-self.wheelbase / den) * (-self.k_p * d_lr_tilde - self.k_d * d_lr_dot))
        delta_c = max(-self.max_steering_angle, min(self.max_steering_angle, delta))

        # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
        # Forward cone is centred on 0 in base_link
        idx_center = int(round(self.base_link_to_laser(0.0) / data.angle_increment)) % self.scan_beams
        half_width = int(round(self.heading_beam_angle / data.angle_increment))
        idx_lo = max(0, idx_center - half_width)
        idx_hi = min(len(raw) - 1, idx_center + half_width)

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

    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x

def main(args):
    rospy.init_node("GapBarrierRGBD_node", anonymous=True)
    GapBarrierRGBD()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)