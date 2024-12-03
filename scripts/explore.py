#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import actionlib
import numpy as np
from visualization_msgs.msg import Marker

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

    def map_callback(self, map_msg):
        """Callback to process the map data from SLAM."""
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.resolution = map_msg.info.resolution
        self.origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        rospy.loginfo("Map updated.")

    def send_goal(self, x, y):
        """Send a navigation goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Face forward

        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

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

    def explore(self):
        """Main exploration loop."""
        self.publish_boundary()  # Publish the boundary

        # while not rospy.is_shutdown():
        #     frontiers = self.find_frontiers()

        #     if not frontiers:
        #         rospy.loginfo("Exploration complete!")
        #         break

        #     # Pick the closest frontier
        #     frontiers.sort(key=lambda p: np.hypot(p[0], p[1]))
        #     next_goal = frontiers[0]

        #     # Mark the goal as explored and send it
        #     self.explored_goals.add(next_goal)
        #     self.send_goal(next_goal[0], next_goal[1])
        self.send_goal(5, 5)

        rospy.loginfo("Exploration finished. Shutting down.")

if __name__ == '__main__':
    try:
        explorer = MazeExplorer()
        explorer.explore()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted.")