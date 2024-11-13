#!/usr/bin/env python3
from typing import Optional, Tuple, List, Dict
from argparse import ArgumentParser
from math import inf, sqrt, atan2, pi
from time import sleep, time
import queue
import json

import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, Point32, PoseStamped, Pose, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion

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


class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """
    
    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
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

    def control(self, err, t):
        dt = t - self.t_prev
        self.err_hist.append(err)
        self.err_int += err
        if len(self.err_hist) > self.kS:
            self.err_int -= self.err_hist.pop(0)
        self.err_dif = err - self.err_prev
        u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
        self.err_prev = err
        self.t_prev = t
        return max(self.u_min, min(u, self.u_max))


class Node:
    def __init__(self, position: POSITION_TYPE, parent: "Node"):
        self.position = position
        self.neighbors = []
        self.parent = parent

    def distance_to(self, other_node: "Node") -> float:
        return np.linalg.norm(self.position - other_node.position)

    def to_dict(self) -> Dict:
        return {"x": self.position[0], "y": self.position[1]}

    def __str__(self) -> str:
        return (
            f"Node<pos: {round(self.position[0], 4)}, {round(self.position[1], 4)}, #neighbors: {len(self.neighbors)}>"
        )


class RrtPlanner:

    def __init__(self, obstacles: List[OBS_TYPE], map_aabb: Tuple):
        self.obstacles = obstacles
        self.map_aabb = map_aabb
        self.graph_publisher = rospy.Publisher("/rrt_graph", MarkerArray, queue_size=10)
        self.plan_visualization_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)
        self.delta = 0.1   # Step size for extending tree
        self.obstacle_padding = 0.15
        self.goal_threshold = GOAL_THRESHOLD
        self.goal_bias = 0.2   # Probability of sampling the goal directly

    def _randomly_sample_q(self, goal_node: Node = None) -> Node:
        """Sample random configuration with goal bias"""
        if goal_node and np.random.random() < self.goal_bias:
            return Node(goal_node.position.copy(), None)
            
        x_min, x_max, y_min, y_max = self.map_aabb
        max_attempts = 100   # Prevent infinite loop
        
        for _ in range(max_attempts):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            position = np.array([x, y])
            
            # Check collision
            if not self._is_in_collision(Node(position, None)):
                return Node(position, None)
                
        # If we failed to find valid sample, return None
        return None

    def _extend(self, graph: List[Node], q_rand: Node):
        """Extend tree toward random sample"""
        if q_rand is None:
            return None

        # Find nearest node
        q_nearest = self._nearest_vertex(graph, q_rand)
        if q_nearest is None:
            return None

        # Compute new position
        direction = q_rand.position - q_nearest.position
        distance = np.linalg.norm(direction)
        
        if distance < self.delta:
            q_new_position = q_rand.position.copy()
        else:
            direction = direction / distance   # Normalize
            q_new_position = q_nearest.position + self.delta * direction

        # Create new node and check collision
        q_new = Node(q_new_position, q_nearest)
        if self._is_in_collision(q_new):
            return None

        # Add to graph
        q_nearest.neighbors.append(q_new)
        graph.append(q_new)
        return q_new

    def generate_plan(self, start, goal):
        """Generate RRT plan with improved robustness"""
        # Convert start and goal to numpy arrays
        start_pos = np.array([start["x"], start["y"]])
        goal_pos = np.array([goal["x"], goal["y"]])
        
        # Check if start or goal is in collision
        start_node = Node(start_pos, None)
        goal_node = Node(goal_pos, None)
        
        if self._is_in_collision(start_node) or self._is_in_collision(goal_node):
            rospy.logwarn("Start or goal position is in collision!")
            return [], []

        # Initialize graph
        graph = [start_node]
        plan = []
        
        # RRT parameters
        max_iterations = 1000   # Increased from 1000
        goal_reached = False
        
        # Main RRT loop
        for i in range(max_iterations):
            # Sample with goal bias
            q_rand = self._randomly_sample_q(goal_node)
            
            # Extend tree
            q_new = self._extend(graph, q_rand)
            
            # Check if we reached the goal
            if q_new is not None:
                distance_to_goal = np.linalg.norm(q_new.position - goal_pos)
                
                if distance_to_goal < self.goal_threshold:
                    # Try to connect directly to goal
                    goal_node.parent = q_new
                    if not self._is_in_collision(goal_node):
                        graph.append(goal_node)
                        goal_reached = True
                        break
            
            # Visualize progress periodically
            if i % 100 == 0:
                self.visualize_graph(graph)
                rospy.sleep(0.1)   # Allow visualization to update

        # Generate plan if goal was reached
        if goal_reached:
            # Backtrack from goal to start
            current_node = goal_node
            while current_node is not None:
                plan.append({"x": current_node.position[0], "y": current_node.position[1]})
                current_node = current_node.parent
            plan.reverse()
            
            rospy.loginfo(f"Plan found with {len(plan)} waypoints")
            self.visualize_plan(plan)
        else:
            rospy.logwarn("Failed to find a valid plan!")
            
        return plan, graph

    # Rest of the methods remain unchanged
    def visualize_plan(self, path: List[Dict]):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(waypoint["x"], waypoint["y"], 0.0)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.scale = Vector3(0.075, 0.075, 0.1)
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
            marker_array.markers.append(marker)
        self.plan_visualization_pub.publish(marker_array)

    def visualize_graph(self, graph: List[Node]):
        marker_array = MarkerArray()
        for i, node in enumerate(graph):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(0.05, 0.05, 0.05)
            marker.pose.position = Point(node.position[0], node.position[1], 0.01)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
            marker_array.markers.append(marker)
        self.graph_publisher.publish(marker_array)

    def _nearest_vertex(self, graph: List[Node], q: Node) -> Node:
        nearest_node = None
        min_distance = float('inf')
        for node in graph:
            distance = np.linalg.norm(node.position - q.position)
            if distance < min_distance:
                nearest_node = node
                min_distance = distance
        return nearest_node

    def _is_in_collision(self, q_rand: Node):
        x = q_rand.position[0]
        y = q_rand.position[1]
        for obs in self.obstacles:
            x_min, x_max, y_min, y_max = obs
            x_min -= self.obstacle_padding
            y_min -= self.obstacle_padding
            x_max += self.obstacle_padding
            y_max += self.obstacle_padding
            if (x_min < x < x_max) and (y_min < y < y_max):
                return True
        return False



# Protip: copy the ObstacleFreeWaypointController class from lab5.py here
######### Your code starts here #########

class ObstacleFreeWaypointController:
    def __init__(self, waypoints: List[Dict]):
        self.waypoints = waypoints
        # Subscriber to the robot's current position (assuming you have Odometry data)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.waypoint_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)
        sleep(0.5)   # sleep to give time for rviz to subscribe to /waypoints

        self.current_position = None

        # Add PID controllers here
        ######### Your code starts here #########
        self.linear_controller = PIDController(0.3, 0.0, 0.1, 10, -0.22, 0.22)
        self.angular_controller = PIDController(0.5, 0.0, 0.2, 10, -2.84, 2.84)
        ######### Your code ends here #########

    def odom_callback(self, msg):
        # Extracting current position from Odometry message
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def calculate_error(self, goal_position: Dict) -> Optional[Tuple]:
        """Return distance and angle error between the current position and the provided goal_position. Returns None if
        the current position is not available.
        """
        if self.current_position is None:
            return None

        ######### Your code starts here #########
        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]
        distance_error = sqrt(dx**2 + dy**2)

        desired_theta = atan2(dy, dx)
        angle_error = desired_theta - self.current_position["theta"]

        # Ensure angle error is within -pi to pi range
        if angle_error > pi:
            angle_error -= 2 * pi
        elif angle_error < -pi:
            angle_error += 2 * pi
        ######### Your code ends here #########

        return distance_error, angle_error

    def control_robot(self):
        rate = rospy.Rate(20) # 20 Hz
        ctrl_msg = Twist()

        ######### Your code starts here #########
        current_waypoint_idx = 0
        ######### Your code ends here #########

        while not rospy.is_shutdown():

            ######### Your code starts here #########
            current_waypoint = self.waypoints[current_waypoint_idx]

            error = self.calculate_error(current_waypoint)
            if error is None:
                continue
            distance_error, angle_error = error

            # Check if close enough to the goal
            if distance_error < 0.05:
                if current_waypoint_idx < len(self.waypoints) - 1:
                    rospy.loginfo("waypoint reached")
                    current_waypoint_idx += 1
                    continue
                rospy.loginfo("final waypoint reached")
                break

            # Calculate control commands using PID controllers
            cmd_linear_vel = self.linear_controller.control(distance_error, rospy.get_time())
            cmd_angular_vel = self.angular_controller.control(angle_error, rospy.get_time())

            # Publish control commands to /cmd_vel topic
            ctrl_msg.linear.x = cmd_linear_vel
            ctrl_msg.angular.z = cmd_angular_vel
            self.robot_ctrl_pub.publish(ctrl_msg)

            rospy.loginfo(
                f"dist to target: {distance_error:.2f}\tangle error: {angle_error:.2f}\tcommanded linear vel: {cmd_linear_vel:.2f}\tcommanded angular vel: {cmd_angular_vel:.2f}"
            )
            ######### Your code ends here #########
            rate.sleep()

######### Your code ends here #########


""" Example usage

rosrun development lab10.py --map_filepath src/csci445l/scripts/lab10_map.json
"""


if __name__ == "__main__":

    # Initialize the ROS node here
    rospy.init_node("rrt_planner")

    # Argument parsing and map loading
    parser = ArgumentParser()
    parser.add_argument("--map_filepath", type=str, required=True)
    args = parser.parse_args()
    with open(args.map_filepath, "r") as f:
        map_ = json.load(f)
        goal_position = map_["goal_position"]
        obstacles = map_["obstacles"]
        map_aabb = map_["map_aabb"]
        start_position = {"x": 0.0, "y": 0.0}

    # Initialize the RRT planner and generate the plan
    planner = RrtPlanner(obstacles, map_aabb)
    plan, graph = planner.generate_plan(start_position, goal_position)
    print("Generated plan:", plan)
    planner.visualize_plan(plan)
    planner.visualize_graph(graph)

    # Pass the plan to the ObstacleFreeWaypointController (no need to call init_node here)
    controller = ObstacleFreeWaypointController(plan)

    # Control robot using the generated waypoints
    try:
        while not rospy.is_shutdown():
            controller.control_robot()
    except rospy.ROSInterruptException:
        print("Shutting down...")
        