#!/usr/bin/env python3
"""
MazeExplorer: Autonomous exploration of a maze with boundary constraints.
SUBSCRIBERS:
  - /map (nav_msgs/OccupancyGrid): Represents the occupancy grid map of the environment.
PUBLISHERS:
  - /move_base_simple/goal (geometry_msgs/PoseStamped): Sends navigation goals to the robot.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
import random
from math import sqrt

class MazeExplorer:

    def __init__(self):
        """Initialize MazeExplorer"""
        rospy.init_node('maze_explorer', log_level=rospy.DEBUG)
        
        # Map parameters
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.map_width = None
        self.map_height = None
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Robot starting position (home)
        self.home_position = (0, 0)

        # Move Base Action Client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        rospy.sleep(2)  # Wait for initial map updates

    def map_callback(self, data):
        """Callback to update map data."""
        self.map_data = np.array(data.data).reshape((data.info.height, data.info.width))
        self.resolution = data.info.resolution
        self.origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.map_width = data.info.width
        self.map_height = data.info.height

    def generate_random_points(self, num_points):
        """Generate a grid of random points within the map boundary."""
        points = []
        for _ in range(num_points):
            row = random.randint(0, self.map_height - 1)
            col = random.randint(0, self.map_width - 1)
            x = col * self.resolution + self.origin[0]
            y = row * self.resolution + self.origin[1]
            points.append((x, y))
        return points

    def find_nearest_frontier(self, points, robot_position):
        """Find the nearest unexplored point from the robot's current position."""
        unexplored_points = [
            point for point in points if self.is_unexplored(point)
        ]
        if not unexplored_points:
            return None

        nearest_point = min(
            unexplored_points,
            key=lambda p: sqrt((p[0] - robot_position[0])**2 + (p[1] - robot_position[1])**2)
        )
        return nearest_point

    def is_unexplored(self, point):
        """Check if a point is unexplored on the map."""
        col = int((point[0] - self.origin[0]) / self.resolution)
        row = int((point[1] - self.origin[1]) / self.resolution)
        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            return self.map_data[row, col] == -1  # -1 indicates unexplored
        return False

    def set_goal(self, point):
        """Send a goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0  # Facing forward
        self.move_base.send_goal(goal)

    def return_home(self):
        """Send the robot back to the starting position."""
        rospy.loginfo("Returning to home position.")
        self.set_goal(self.home_position)

    def explore(self):
        """Main exploration logic."""
        robot_position = self.home_position
        points = self.generate_random_points(1000)

        while True:
            # Find the nearest unexplored frontier
            frontier = self.find_nearest_frontier(points, robot_position)
            if frontier is None:
                rospy.loginfo("Exploration complete!")
                break

            # Move to the frontier
            rospy.loginfo(f"Moving to frontier: {frontier}")
            self.set_goal(frontier)
            self.move_base.wait_for_result()

            # Update robot position after reaching goal
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached goal successfully.")
                robot_position = frontier
            else:
                rospy.logwarn("Failed to reach goal. Trying next frontier.")
                points.remove(frontier)

        # Return to the home position
        self.return_home()
        self.move_base.wait_for_result()

def main():
    """Run the MazeExplorer node."""
    explorer = MazeExplorer()
    rospy.sleep(2)  # Ensure the map is populated
    explorer.explore()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("MazeExplorer interrupted.")
