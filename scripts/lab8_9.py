#!/usr/bin/env python3
from typing import Optional, Tuple, List, Dict
from argparse import ArgumentParser
from math import inf, sqrt, atan2, pi
from time import sleep, time
import queue
import json
import math
import random
from random import uniform
import copy

import scipy
import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, Point32, PoseStamped, Pose, Vector3, Quaternion, Point, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import scipy.stats
from numpy.random import choice

np.set_printoptions(linewidth=200)

# AABB format: (x_min, x_max, y_min, y_max)
OBS_TYPE = Tuple[float, float, float, float]
# Position format: {"x": x, "y": y, "theta": theta}
POSITION_TYPE = Dict[str, float]

# don't change this
GOAL_THRESHOLD = 0.1


def angle_to_0_to_2pi(angle: float) -> float:
    while angle < 0:
        angle += 2 * pi
    while angle > 2 * pi:
        angle -= 2 * pi
    return angle


def angle_to_neg_pi_to_pi(angle: float) -> float:
    while angle < -pi:
        angle += 2 * pi
    while angle > pi:
        angle -= 2 * pi
    return angle


# see https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
def ray_line_intersection(ray_origin, ray_direction_rad, point1, point2):
    # Convert to numpy arrays
    ray_origin = np.array(ray_origin, dtype=np.float32)
    ray_direction = np.array([math.cos(ray_direction_rad), math.sin(ray_direction_rad)])
    point1 = np.array(point1, dtype=np.float32)
    point2 = np.array(point2, dtype=np.float32)

    # Ray-Line Segment Intersection Test in 2D
    v1 = ray_origin - point1
    v2 = point2 - point1
    v3 = np.array([-ray_direction[1], ray_direction[0]])
    denominator = np.dot(v2, v3)
    if denominator == 0:
        return None
    t1 = np.cross(v2, v1) / denominator
    t2 = np.dot(v1, v3) / denominator
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return [ray_origin + t1 * ray_direction]
    return None


class Map:
    def __init__(self, obstacles: List[OBS_TYPE], map_aabb: Tuple):
        self.obstacles = obstacles
        self.map_aabb = map_aabb

    @property
    def top_right(self) -> Tuple[float, float]:
        return self.map_aabb[1], self.map_aabb[3]

    @property
    def bottom_left(self) -> Tuple[float, float]:
        return self.map_aabb[0], self.map_aabb[2]

    def draw_distances(self, origins: List[Tuple[float, float]]):
        """Example usage:
        map_ = Map(obstacles, map_aabb)
        map_.draw_distances([(0.0, 0.0), (3, 3), (1.5, 1.5)])
        """

        # Draw scene
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(figsize=(10, 10))
        fig.tight_layout()
        x_min_global, x_max_global, y_min_global, y_max_global = self.map_aabb
        for aabb in self.obstacles:
            width = aabb[1] - aabb[0]
            height = aabb[3] - aabb[2]
            rect = patches.Rectangle(
                (aabb[0], aabb[2]), width, height, linewidth=2, edgecolor="r", facecolor="r", alpha=0.4
            )
            ax.add_patch(rect)
        ax.set_xlim(x_min_global, x_max_global)
        ax.set_ylim(y_min_global, y_max_global)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("2D Plot of Obstacles")
        ax.set_aspect("equal", "box")
        plt.grid(True)

        # Draw rays
        angles = np.linspace(0, 2 * math.pi, 10, endpoint=False)
        for origin in origins:
            for angle in angles:
                closest_distance = self.closest_distance(origin, angle)
                if closest_distance is not None:
                    x = origin[0] + closest_distance * math.cos(angle)
                    y = origin[1] + closest_distance * math.sin(angle)
                    plt.plot([origin[0], x], [origin[1], y], "b-")
        plt.show()

    def closest_distance(self, origin: Tuple[float, float], angle: float) -> Optional[float]:
        """Returns the closest distance to an obstacle from the given origin in the given direction `angle`. If no
        intersection is found, returns `None`.
        """

        def lines_from_obstacle(obstacle: OBS_TYPE):
            """Returns the four lines of the given AABB format obstacle.
            Example usage: `point0, point1 = lines_from_obstacle(self.obstacles[0])`
            """
            x_min, x_max, y_min, y_max = obstacle
            return [
                [(x_min, y_min), (x_max, y_min)],
                [(x_max, y_min), (x_max, y_max)],
                [(x_max, y_max), (x_min, y_max)],
                [(x_min, y_max), (x_min, y_min)],
            ]

        # Iterate over the obstacles in the map to find the closest distance (if there is one). Remember that the
        # obstacles are represented as a list of AABBs (Axis-Aligned Bounding Boxes) with the format
        # (x_min, x_max, y_min, y_max).
        result = None
        origin = np.array(origin)

        for obstacle in self.obstacles:
            for line in lines_from_obstacle(obstacle):
                p = ray_line_intersection(origin, angle, line[0], line[1])
                if p is None:
                    continue

                dist = np.linalg.norm(np.array(p) - origin)
                if result is None:
                    result = dist
                else:
                    result = min(result, dist)
        return result

# PID controller class
######### Your code starts here #########
class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """

    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        ######### Your code starts here #########
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kS = kS
        self.err_int = 0
        self.err_dif = 0
        self.err_prev = 0
        self.err_hist = []
        self.t_prev = 0
        self.u_min = u_min
        self.u_max = u_max
        ######### Your code ends here #########

    def control(self, err, t):
        ######### Your code starts here #########
        dt = t - self.t_prev
        if dt < 1e-6:
            return 0
        self.err_hist.append(err)
        self.err_int += err
        if len(self.err_hist) > self.kS:
            self.err_int -= self.err_hist.pop(0)
        self.err_dif = err - self.err_prev
        u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
        self.err_prev = err
        self.t_prev = t
        return max(self.u_min, min(u, self.u_max))
        ######### Your code ends here #########
######### Your code ends here #########


class Particle:
    def __init__(self, x: float, y: float, theta: float, log_p: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.log_p = log_p

    def __str__(self) -> str:
        return f"Particle<pose: {self.x, self.y, self.theta}, log_p: {self.log_p}>"


class ParticleFilter:

    def __init__(
        self,
        map_: Map,
        n_particles: int,
        translation_variance: float,
        rotation_variance: float,
        measurement_variance: float,
    ):
        # Initialize uniformly-distributed particles
        ######### Your code starts here #########
        self.map_ = map_
        self._n_particles = n_particles
        self._translation_variance = translation_variance
        self._rotation_variance = rotation_variance
        self._measurement_variance = measurement_variance

        self._particles = []
        for _ in range(self._n_particles):
            x = random.uniform(self.map_.bottom_left[0], self.map_.top_right[0])
            y = random.uniform(self.map_.bottom_left[1], self.map_.top_right[1])
            theta = random.uniform(-math.pi, math.pi)
            log_p = 0.0
            self._particles.append(Particle(x, y, theta, log_p))


        self.particles_visualization_pub = rospy.Publisher("/pf_particles", PoseArray, queue_size=10)
        self.estimate_visualization_pub = rospy.Publisher("/pf_estimate", PoseStamped, queue_size=10)



        ######### Your code ends here #########

    def visualize_particles(self):
        pa = PoseArray()
        pa.header.frame_id = "odom"
        pa.header.stamp = rospy.Time.now()
        for particle in self._particles:
            pose = Pose()
            pose.position = Point(particle.x, particle.y, 0.01)
            q_np = quaternion_from_euler(0, 0, float(particle.theta))
            pose.orientation = Quaternion(*q_np.tolist())
            pa.poses.append(pose)
        self.particles_visualization_pub.publish(pa)

    def visualize_estimate(self):
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time.now()
        x, y, theta = self.get_estimate()
        pose = Pose()
        pose.position = Point(x, y, 0.01)
        q_np = quaternion_from_euler(0, 0, float(theta))
        pose.orientation = Quaternion(*q_np.tolist())
        ps.pose = pose
        self.estimate_visualization_pub.publish(ps)

    def move_by(self, delta_x, delta_y, delta_theta):
        delta_theta = angle_to_neg_pi_to_pi(delta_theta)

        # Propagate motion of each particle
        ######### Your code starts here #########
        for particle in self._particles:
            particle.x += delta_x + np.random.normal(0, self._translation_variance)
            particle.y += delta_y + np.random.normal(0, self._translation_variance)
            particle.theta += delta_theta + np.random.normal(0, self._rotation_variance)
            particle.theta = angle_to_neg_pi_to_pi(particle.theta)

        ######### Your code ends here #########

    def measure(self, z: float, scan_angle_in_rad: float):
        """Update the particles based on the measurement `z` at the given `scan_angle_in_rad`.

        Args:
            z: distance to an obstacle
            scan_angle_in_rad: Angle in the robots frame where the scan was taken
        """

        # Calculate posterior probabilities and resample
        ######### Your code starts here #########
        weights = []
        for particle in self._particles:
            expected_z = self.map_.closest_distance((particle.x, particle.y), particle.theta + scan_angle_in_rad)
            if expected_z is None:
                weights.append(0)
            else:
                weight = scipy.stats.norm(expected_z, self._measurement_variance).pdf(z)
                weights.append(weight)

        weights = np.array(weights)
        weights /= np.sum(weights)

        indices = choice(len(self._particles), size=len(self._particles), p=weights)
        self._particles = [copy.deepcopy(self._particles[i]) for i in indices]

        ######### Your code ends here #########

    def get_estimate(self) -> Tuple[float, float, float]:
        # Estimate robot's location using particle weights
        ######### Your code starts here #########
        x_estimate = sum(particle.x for particle in self._particles) / self._n_particles
        y_estimate = sum(particle.y for particle in self._particles) / self._n_particles
        theta_estimate = sum(particle.theta for particle in self._particles) / self._n_particles
        
        return x_estimate, y_estimate, angle_to_neg_pi_to_pi(theta_estimate)

        ######### Your code ends here #########


class Controller:
    def __init__(self, particle_filter: ParticleFilter):
        rospy.init_node("particle_filter_controller", anonymous=True)
        self._particle_filter = particle_filter        
        self._particle_filter.visualize_particles()

        #
        self.rate = rospy.Rate(10)  # 10 Hz

        self.current_position = None
        self.laserscan = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/scan_pointcloud", PointCloud, queue_size=10)
        self.target_position_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)

        while ((self.current_position is None) or (self.laserscan is None)) and (not rospy.is_shutdown()):
            rospy.loginfo("waiting for odom and laserscan")
            rospy.sleep(0.1)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def robot_laserscan_callback(self, msg: LaserScan):
        self.laserscan = msg

    def visualize_laserscan_ranges(self, idx_groups: List[Tuple[int, int]]):
        """Helper function to visualize ranges of sensor readings from the laserscan lidar.

        Example usage for visualizing the first 10 and last 10 degrees of the laserscan:
            `self.visualize_laserscan_ranges([(0, 10), (350, 360)])`
        """
        pcd = PointCloud()
        pcd.header.frame_id = "odom"
        pcd.header.stamp = rospy.Time.now()
        for idx_low, idx_high in idx_groups:
            for idx, d in enumerate(self.laserscan.ranges[idx_low:idx_high]):
                if d == inf:
                    continue
                angle = math.radians(idx) + self.current_position["theta"]
                x = d * math.cos(angle) + self.current_position["x"]
                y = d * math.sin(angle) + self.current_position["y"]
                z = 0.1
                pcd.points.append(Point32(x=x, y=y, z=z))
                pcd.channels.append(ChannelFloat32(name="rgb", values=(0.0, 1.0, 0.0)))
        self.pointcloud_pub.publish(pcd)

    def visualize_position(self, x: float, y: float):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = Point(x, y, 0.0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.075, 0.075, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        marker_array.markers.append(marker)
        self.target_position_pub.publish(marker_array)

    def take_measurements(self):
        if self.laserscan:
            
            angles = [0,180]
            num_ranges = len(self.laserscan.ranges)
            angle_min = self.laserscan.angle_min
            angle_increment = self.laserscan.angle_increment
    
            for angle in angles:
                index = int((math.radians(angle) - angle_min) / angle_increment)
                if 0 <= index < num_ranges:
                    distance = self.laserscan.ranges[index]
                    if distance != inf:
                        scan_angle_in_rad = math.radians(angle)
                        self._particle_filter.measure(distance, scan_angle_in_rad)
    
        ######### Your code ends here #########

    def autonomous_exploration(self):
        """Randomly explore the environment here, while making sure to call `take_measurements()` and
        `_particle_filter.move_by()`. The particle filter should converge on the robots position eventually.

        Note that the following visualizations functions are available:
            visualize_position(...)
            visualize_laserscan_ranges(...)
        """
        # Robot autonomously explores environment while it localizes itself
        ######### Your code starts here #########

        # Define a predefined sequence of actions
        actions = [
            ("forward", 0.5),
            ("rotate", math.radians(-20)),
            ("forward", 1),
            ("rotate", math.radians(20)),
            ("forward", 0.4),
            ("rotate", math.radians(45)),
            ("forward", 0.2)
        ]

        for action, value in actions:
            if rospy.is_shutdown():
                break

            if action == "forward":
                self.forward_action(value)
            elif action == "rotate":
                self.rotate_action(value)

            self.take_measurements()
            self.visualize_position(self.current_position["x"], self.current_position["y"])
            self.rate.sleep()

        ######### Your code ends here #########

    def forward_action(self, distance: float):
        # Robot moves forward or backward by a set amount during manual control
        ######### Your code starts here #########
    
        linear_controller = PIDController(0.8, 0.1, 0.1, 10, -0.1, 0.1)
        initial_x = self.current_position["x"]
        initial_y = self.current_position["y"]
        initial_t = self.current_position["theta"]
        ctrl_msg = Twist()
    
        while not rospy.is_shutdown():
            distance_error = distance - sqrt((self.current_position["x"] - initial_x) ** 2 + (self.current_position["y"] - initial_y) ** 2)
            cmd_linear_vel = linear_controller.control(distance_error, rospy.get_time())
            ctrl_msg.linear.x = cmd_linear_vel
            self.robot_ctrl_pub.publish(ctrl_msg)
    
            if abs(distance_error) < 0.01:
                ctrl_msg.linear.x = 0
                self.robot_ctrl_pub.publish(ctrl_msg)
                break
    
            self.rate.sleep()
    
        # Update particle filter with movement
        delta_x = self.current_position["x"] - initial_x
        delta_y = self.current_position["y"] - initial_y
        delta_theta = self.current_position["theta"] - initial_t
        self._particle_filter.move_by(delta_x, delta_y, delta_theta)

        self._particle_filter.visualize_particles()

    
        ######### Your code ends here #########

    def rotate_action(self, goal_theta: float):
        # Robot turns by a set amount during manual control
        ######### Your code starts here #########

        angular_controller = PIDController(0.5, 0.5, 0.5, 10, -1, 1)

        initial_theta = self.current_position["theta"]
        ctrl_msg = Twist()

        while not rospy.is_shutdown():
            angle_error = goal_theta - (self.current_position["theta"] - initial_theta)

            # Normalize angle error to -pi to pi range
            angle_error = (angle_error + pi) % (2 * pi) - pi

            cmd_angular_vel = angular_controller.control(angle_error, rospy.get_time())
            ctrl_msg.angular.z = cmd_angular_vel
            self.robot_ctrl_pub.publish(ctrl_msg)

            if abs(angle_error) < math.radians(1):
                ctrl_msg.angular.z = 0
                self.robot_ctrl_pub.publish(ctrl_msg)
                break

            self.rate.sleep()

        # Update particle filter with movement
        delta_theta = self.current_position["theta"] - initial_theta
        self._particle_filter.move_by(0, 0, delta_theta)

        self._particle_filter.visualize_particles()

        ######### Your code ends here #########


""" Example usage

rosrun development lab8_9.py --map_filepath src/csci455l/scripts/lab8_9_map.json
"""


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--map_filepath", type=str, required=True)
    args = parser.parse_args()
    with open(args.map_filepath, "r") as f:
        map_ = json.load(f)
        obstacles = map_["obstacles"]
        map_aabb = map_["map_aabb"]

    map_ = Map(obstacles, map_aabb)
    num_particles = 200
    translation_variance = 0.1
    rotation_variance = 0.05
    measurement_variance = 0.1
    particle_filter = ParticleFilter(map_, num_particles, translation_variance, rotation_variance, measurement_variance)
    controller = Controller(particle_filter)

    try:
        # Manual control
        goal_theta = 0
        # controller.take_measurements()
        # while not rospy.is_shutdown():
        #     print("\nEnter 'a', 'w', 's', 'd' to move the robot:")
        #     uinput = input("")
        #     if uinput == "w": # forward
        #         ######### Your code starts here #########
        #         controller.forward_action(0.5)
        #         ######### Your code ends here #########
        #     elif uinput == "a": # left
        #         ######### Your code starts here #########
        #         controller.rotate_action(math.radians(90))
        #         ######### Your code ends here #########
        #     elif uinput == "d": #right
        #         ######### Your code starts here #########
        #         controller.rotate_action(math.radians(-90))
        #         ######### Your code ends here #########
        #     elif uinput == "s": # backwards
        #         ######### Your code starts here #########
        #         controller.forward_action(-0.5)
        #         ######### Your code ends here #########
        #     else:
        #         print("Invalid input")
        #     ######### Your code starts here #########
        #     controller.take_measurements()
        #     ######### Your code ends here #########

        # Autonomous exploration
        ######### Your code starts here #########
        controller.autonomous_exploration()
        ######### Your code ends here #########

    except rospy.ROSInterruptException:
        print("Shutting down...")
