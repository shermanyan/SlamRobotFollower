#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import actionlib
import numpy as np
from visualization_msgs.msg import Marker
import json
import os
import subprocess
import xml.etree.ElementTree as ET

class MazeExplorer:
    
    def __init__(self):
        rospy.init_node('maze_explorer')

        # Get boundary dimensions and starting position from launch file arguments
        self.boundary_length = rospy.get_param('~boundary_length', 10.0)
        self.boundary_width = rospy.get_param('~boundary_width', 10.0)
        self.start_x = rospy.get_param('~start_x', 0.0)
        self.start_y = rospy.get_param('~start_y', 0.0)

        # Define exploration boundaries (in meters)
        self.boundary_min_x = self.start_x - self.boundary_length / 2
        self.boundary_max_x = self.start_x + self.boundary_length / 2
        self.boundary_min_y = self.start_y - self.boundary_width / 2
        self.boundary_max_y = self.start_y + self.boundary_width / 2

        rospy.loginfo(f"Exploration boundaries set: "
                      f"X[{self.boundary_min_x}, {self.boundary_max_x}], "
                      f"Y[{self.boundary_min_y}, {self.boundary_max_y}]")

        # Initialize subscribers and action clients
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Variables to store the map and exploration state
        self.map_data = None
        self.resolution = 0
        self.origin = (0, 0)
        self.explored_goals = set()
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # Initialize the SLAM data dictionary
        self.slam_data = {
            "goal_position": {},
            "map_aabb": [self.boundary_min_x, self.boundary_max_x, self.boundary_min_y, self.boundary_max_y],
            "obstacles": []
        }

        # Ensure the directory for SLAM data exists
        self.slam_data_dir = os.path.join(os.path.expanduser("~"), "catkin_ws/src/SlamRobotFollower/slam_data")
        os.makedirs(self.slam_data_dir, exist_ok=True)

    def map_callback(self, map_msg):
        """Callback to process the map data from SLAM."""
        rospy.loginfo("map_callback triggered.")
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.resolution = map_msg.info.resolution
        self.origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        rospy.loginfo("Map updated. Extracting obstacles...")
        self.extract_obstacles()
        self.save_slam_data()  # Save data each time the map updates

    def extract_obstacles(self):
        """Extract obstacle data from the map and save it."""
        if self.map_data is None:
            rospy.logwarn("Map data is None. Obstacles cannot be extracted.")
            return

        rospy.loginfo("Extracting obstacles from map data...")
        self.slam_data["obstacles"] = []  # Clear existing obstacles
        for i in range(self.map_data.shape[0]):
            for j in range(self.map_data.shape[1]):
                if self.map_data[i, j] > 50:  # Assuming 50 is the threshold for obstacles
                    obstacle_x = self.origin[0] + j * self.resolution
                    obstacle_y = self.origin[1] + i * self.resolution
                    self.slam_data["obstacles"].append([
                        obstacle_x - self.resolution / 2,
                        obstacle_x + self.resolution / 2,
                        obstacle_y - self.resolution / 2,
                        obstacle_y + self.resolution / 2
                    ])
        rospy.loginfo(f"Extracted {len(self.slam_data['obstacles'])} obstacles.")

    def send_goal(self, x, y):
        """Send a navigation goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Face forward

        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.slam_data["goal_position"] = {"x": x, "y": y}
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        self.save_slam_data()  # Save data after sending a goal

    def publish_boundary(self):
        """Publish the boundary as a Marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "boundary"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Define the boundary points
        points = [
            (self.start_x, self.start_y),
            (self.start_x + self.boundary_length, self.start_y),
            (self.start_x + self.boundary_length, self.start_y + self.boundary_width),
            (self.start_x, self.start_y + self.boundary_width),
            (self.start_x, self.start_y)
        ]

        for x, y in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def save_slam_data(self):
        """Save the SLAM data to a JSON file."""
        json_path = os.path.join(self.slam_data_dir, 'slam_data.json')
        rospy.loginfo("Saving SLAM data...")
        with open(json_path, 'w') as json_file:
            json.dump(self.slam_data, json_file, indent=4)
        rospy.loginfo(f"SLAM data saved to {json_path}")

        # Append changes to a log for debugging
        log_path = os.path.join(self.slam_data_dir, 'slam_data_log.json')
        with open(log_path, 'a') as log_file:
            log_file.write(json.dumps(self.slam_data) + "\n")
        rospy.loginfo(f"SLAM data changes appended to {log_path}")

    def save_map_and_data(self, map_name="map"):
        """Save the map using map_saver and the SLAM data."""
        rospy.loginfo("Saving map and associated SLAM data...")
        # Save map using map_saver
        save_path = os.path.join(self.slam_data_dir, map_name)
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", save_path])
        rospy.loginfo(f"Map saved to {save_path}.pgm and {save_path}.yaml")
        
        # Save SLAM data
        self.save_slam_data()

    def generate_world_file(self, world_name="slam_map"):
        """Generate a Gazebo .world file based on SLAM data."""
        rospy.loginfo("Generating Gazebo world file...")
        
        # Create the root element
        world = ET.Element('sdf', {'version': '1.6'})
        world_elem = ET.SubElement(world, 'world', {'name': world_name})
        
        # Add a basic ground plane
        ground_plane = ET.SubElement(world_elem, 'include')
        ground_uri = ET.SubElement(ground_plane, 'uri')
        ground_uri.text = 'model://ground_plane'
        
        # Add sun
        sun = ET.SubElement(world_elem, 'include')
        sun_uri = ET.SubElement(sun, 'uri')
        sun_uri.text = 'model://sun'
        
        # Add obstacles from SLAM data
        for i, obstacle in enumerate(self.slam_data.get("obstacles", [])):
            # Calculate obstacle dimensions
            x_min, x_max, y_min, y_max = obstacle
            width = x_max - x_min
            height = y_max - y_min
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            
            # Create model for each obstacle
            model = ET.SubElement(world_elem, 'model', {'name': f'obstacle_{i}'})
            pose = ET.SubElement(model, 'pose')
            pose.text = f'{x_center} {y_center} 0 0 0 0'
            
            # Add static link
            link = ET.SubElement(model, 'link', {'name': 'link'})
            collision = ET.SubElement(link, 'collision', {'name': 'collision'})
            geometry = ET.SubElement(collision, 'geometry')
            box = ET.SubElement(geometry, 'box')
            size = ET.SubElement(box, 'size')
            size.text = f'{width} {height} 0.5'
            
            visual = ET.SubElement(link, 'visual', {'name': 'visual'})
            visual_geometry = ET.SubElement(visual, 'geometry')
            visual_box = ET.SubElement(visual_geometry, 'box')
            visual_size = ET.SubElement(visual_box, 'size')
            visual_size.text = f'{width} {height} 0.5'
            
            # Make model static
            static = ET.SubElement(model, 'static')
            static.text = 'true'
        
        # Write to .world file
        world_path = os.path.join(self.slam_data_dir, f'{world_name}.world')
        tree = ET.ElementTree(world)
        tree.write(world_path)
        
        rospy.loginfo(f"World file saved to {world_path}")
        return world_path

    def explore(self):
        """Main exploration loop."""
        self.publish_boundary()  # Publish the boundary

        self.send_goal(5, 5)  # Example goal

        # Save map and SLAM data
        self.save_map_and_data("my_exploration_map")
        
        # Generate world file
        self.generate_world_file("my_exploration_world")

        rospy.loginfo("Exploration finished. Map, SLAM data, and world file saved. Shutting down.")

if __name__ == '__main__':
    try:
        explorer = MazeExplorer()
        explorer.explore()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted.")