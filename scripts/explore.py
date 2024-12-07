#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib
import numpy as np
import random
import math
import argparse

class MazeExplorer:
    def __init__(self, boundary_length=6.0, 
                 boundary_width=6.0, 
                 bound_start_x=-1.0, 
                 bound_start_y=-1.0, 
                 num_random_points=100, 
                 tolerance=3,
                 robot_name='tb3_0'
                 ):
        
        rospy.init_node('maze_explorer')

        self.current_position = None
        self.current_goal = None
        self.explored = False

        # Parameters
        self.boundary_length = rospy.get_param('~boundary_length', boundary_length)
        self.boundary_width = rospy.get_param('~boundary_width', boundary_width)
        self.bound_start_x = rospy.get_param('~bound_start_x', bound_start_x)
        self.bound_start_y = rospy.get_param('~bound_start_y', bound_start_y)
        self.num_random_points = rospy.get_param('~num_random_points', num_random_points)
        self.tolerance = rospy.get_param('~tolerance', tolerance)
        self.robot_name = rospy.get_param('~robot_name', robot_name)

        self.boundary_min_x = self.bound_start_x
        self.boundary_max_x = self.bound_start_x + self.boundary_length
        self.boundary_min_y = self.bound_start_y
        self.boundary_max_y = self.bound_start_y + self.boundary_width

        self.map_data = None
        self.resolution = 0
        self.origin = (0, 0)

        # Random points for exploration
        self.frontiers = []
        self.remaining_frontiers = []

        # Subscribers
        self.odom_sub = rospy.Subscriber(f'/{self.robot_name}/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber(f'/{self.robot_name}/map', OccupancyGrid, self.map_callback)

        # Publishers
        self.marker_pub = rospy.Publisher('/exploration_markers', MarkerArray, queue_size=10)
        self.polygon_pub = rospy.Publisher('/search_boundary', PolygonStamped, queue_size=10)
        self.map_pub = rospy.Publisher(f'{self.robot_name}/map', OccupancyGrid, queue_size=10)

        self.init_boundary()
        self.generate_random_points()

        rospy.loginfo(f"Attempt subscribe to /{self.robot_name}/move_base")

        self.move_base_client = actionlib.SimpleActionClient(f'/{self.robot_name}/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


        self.robot_ctrl_pub = rospy.Publisher(f'{self.robot_name}/cmd_vel', Twist, queue_size=10)
        self.move_base_ctrl_sub = rospy.Subscriber("/cmd_vel", Twist, self.remap_cmd_vel)


    def odom_callback(self, msg):
        '''Callback to process the odometry data.'''
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def remap_cmd_vel(self,msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.linear.x
        twist_msg.linear.y = msg.linear.y
        twist_msg.linear.z = msg.linear.z
        twist_msg.angular.x = msg.angular.x
        twist_msg.angular.y = msg.angular.y
        twist_msg.angular.z = msg.angular.z
        self.robot_ctrl_pub.publish(twist_msg)

    def map_callback(self, map_msg):

        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.resolution = map_msg.info.resolution
        self.origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.update_frontiers()
        self.publish_markers()

    def init_boundary(self):
        """Initialize the boundary polygon."""
        self.polygon = PolygonStamped()
        self.polygon.header.frame_id = "map"

        points = [Point32(x=self.boundary_min_x, y=self.boundary_min_y),
                  Point32(x=self.boundary_min_x, y=self.boundary_max_y),
                  Point32(x=self.boundary_max_x, y=self.boundary_max_y),
                  Point32(x=self.boundary_max_x, y=self.boundary_min_y)]

        self.polygon.polygon.points = points

    def mark_boundary_walls_as_obstacles(self):
            
            """Mark the boundary walls as obstacles in the map."""
            if self.map_data is None:
                return
            
            # Convert boundary coordinates (in meters) to map grid indices
            min_x_idx = int((self.boundary_min_x - self.origin[0]) / self.resolution)
            max_x_idx = int((self.boundary_max_x - self.origin[0]) / self.resolution)
            min_y_idx = int((self.boundary_min_y - self.origin[1]) / self.resolution)
            max_y_idx = int((self.boundary_max_y - self.origin[1]) / self.resolution)

            # Mark the walls (edges) of the boundary as obstacles
            # Top and bottom walls
            self.map_data[min_y_idx, min_x_idx:max_x_idx] = 100
            self.map_data[max_y_idx, min_x_idx:max_x_idx] = 100

            # Left and right walls
            self.map_data[min_y_idx:max_y_idx, min_x_idx] = 100
            self.map_data[min_y_idx:max_y_idx, max_x_idx] = 100

            # Create and publish an updated map to /custom_map
            updated_map = OccupancyGrid()
            updated_map.header.stamp = rospy.Time.now()
            updated_map.header.frame_id = "map"
            updated_map.info.resolution = self.resolution
            updated_map.info.width = self.map_data.shape[1]
            updated_map.info.height = self.map_data.shape[0]
            updated_map.info.origin.position.x = self.origin[0]
            updated_map.info.origin.position.y = self.origin[1]
            updated_map.info.origin.position.z = 0
            updated_map.data = self.map_data.flatten().tolist()

            # Publish the updated map to the /custom_map topic
            self.map_pub.publish(updated_map)
    
    def publish_boundary(self):
        self.polygon_pub.publish(self.polygon)

    def generate_random_points(self):
        """Generate random exploration points within the boundary."""
        self.frontiers = [(random.uniform(self.boundary_min_x, self.boundary_max_x),
                           random.uniform(self.boundary_min_y, self.boundary_max_y)) for _ in range(self.num_random_points)]
        self.remaining_frontiers = self.frontiers[:]

    def publish_markers(self):
        """Publish exploration points as markers."""
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
    
            if self.current_goal and self.current_goal == (x, y):
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif (x, y) in self.remaining_frontiers:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
    
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def update_frontiers(self):
        """Update the status of frontiers based on the map with a given tolerance."""
        new_remaining_frontiers = []
        resolution_inv = 1 / self.resolution

        goal_reached = False

        for x, y in self.remaining_frontiers:
            # Convert coordinates to map indices
            map_x = int((x - self.origin[0]) * resolution_inv)
            map_y = int((y - self.origin[1]) * resolution_inv)

            # Define the tolerance range in map coordinates
            min_x = max(0, map_x - self.tolerance)
            max_x = min(self.map_data.shape[1], map_x + self.tolerance + 1)
            min_y = max(0, map_y - self.tolerance)
            max_y = min(self.map_data.shape[0], map_y + self.tolerance + 1)

            # Check the region around the frontier point
            region = self.map_data[min_y:max_y, min_x:max_x]
            if np.any(region == 0):  
                if self.current_goal and self.current_goal == (x, y):
                    goal_reached = True
                continue  
            else:
                new_remaining_frontiers.append((x, y)) 

        self.remaining_frontiers = new_remaining_frontiers

        if self.current_goal and goal_reached:
            self.move_to_next_frontier()


    def send_goal(self, x, y):
        """Send a navigation goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: ({x:.2f}, {y:.2f})")
        self.current_goal = (x, y)
        self.move_base_client.send_goal(goal)
        # rospy.loginfo(goal)

    def move_to_next_frontier(self):
        """Move to the next unexplored frontier."""
        if self.remaining_frontiers:
            next_frontier = self.find_nearest_frontier()
            if next_frontier:
                self.send_goal(*next_frontier)
        else:
            rospy.loginfo("All frontiers explored.")
            self.explored = True


    def find_nearest_frontier(self):
        """Find the nearest unexplored frontier using Manhattan distance."""
        nearest = None
        min_distance = float('inf')
        for x, y in self.remaining_frontiers:
            distance = abs(x - self.current_position["x"]) + abs(y - self.current_position["y"])
            if distance < min_distance:
                min_distance = distance
                nearest = (x, y)
        return nearest


    def explore(self):
        """Main exploration loop."""
        rate = rospy.Rate(10)
        self.mark_boundary_walls_as_obstacles()
        self.publish_boundary()

        self.move_to_next_frontier()

        while not rospy.is_shutdown():

            self.mark_boundary_walls_as_obstacles()
            self.publish_boundary()

            if (self.explored):
                rospy.loginfo("Exploration completed.")
                break

            rate.sleep()

        rospy.loginfo("Exploration finished. Shutting down.")


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Maze Explorer Parameters')
        parser.add_argument('--boundary_length', type=float, default=6.0, help='Length of the boundary')
        parser.add_argument('--boundary_width', type=float, default=6.0, help='Width of the boundary')
        parser.add_argument('--bound_start_x', type=float, default=-1.0, help='Starting x-coordinate of the boundary')
        parser.add_argument('--bound_start_y', type=float, default=-1.0, help='Starting y-coordinate of the boundary')
        parser.add_argument('--num_random_points', type=int, default=400, help='Number of random points for exploration')
        parser.add_argument('--tolerance', type=int, default=3, help='Tolerance for frontier exploration')
        parser.add_argument('--robot_name', type=str, default='', help='Name of the robot')

        args = parser.parse_args()

        explorer = MazeExplorer(boundary_length=args.boundary_length,
                    boundary_width=args.boundary_width,
                    bound_start_x=args.bound_start_x,
                    bound_start_y=args.bound_start_y,
                    num_random_points=args.num_random_points,
                    tolerance=args.tolerance,
                    robot_name=args.robot_name)
        
        explorer.explore()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted.")
