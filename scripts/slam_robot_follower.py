#!/usr/bin/env python3

import rospy
import argparse
from explore import MazeExplorer




if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Maze Explorer Parameters')
        parser.add_argument('--boundary_length', type=float, default=6.0, help='Length of the boundary')
        parser.add_argument('--boundary_width', type=float, default=6.0, help='Width of the boundary')
        parser.add_argument('--bound_start_x', type=float, default=-1.0, help='Starting x-coordinate of the boundary')
        parser.add_argument('--bound_start_y', type=float, default=-1.0, help='Starting y-coordinate of the boundary')
        parser.add_argument('--num_random_points', type=int, default=200, help='Number of random points for exploration')
        parser.add_argument('--tolerance', type=int, default=4, help='Tolerance for frontier exploration')
        parser.add_argument('--robot_name', type=str, default='tb3_0', help='Name of the robot')

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
