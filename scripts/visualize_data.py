#!/usr/bin/env python3

import json
import os

def generate_gazebo_world(json_path, output_world_path):
    """Generate a Gazebo .world file from SLAM data."""
    # Load the JSON data
    with open(json_path, 'r') as f:
        slam_data = json.load(f)

    map_aabb = slam_data["map_aabb"]  # Boundary [min_x, max_x, min_y, max_y]
    obstacles = slam_data["obstacles"]  # List of obstacle bounding boxes

    # Begin Gazebo world file content
    world_template = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Boundaries -->
    <model name="boundary">
      <static>true</static>
      <link name="boundary_link">
        <collision name="boundary_collision">
          <geometry>
            <box>
              <size>{map_aabb[1] - map_aabb[0]} {map_aabb[3] - map_aabb[2]} 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="boundary_visual">
          <geometry>
            <box>
              <size>{map_aabb[1] - map_aabb[0]} {map_aabb[3] - map_aabb[2]} 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
          </material>
        </visual>
      </link>
      <pose>{(map_aabb[0] + map_aabb[1]) / 2} {(map_aabb[2] + map_aabb[3]) / 2} 0 0 0 0</pose>
    </model>

    <!-- Obstacles -->
"""
    # Add obstacles as models
    for i, obs in enumerate(obstacles):
        obs_width = obs[1] - obs[0]
        obs_height = obs[3] - obs[2]
        obs_x = (obs[0] + obs[1]) / 2
        obs_y = (obs[2] + obs[3]) / 2
        world_template += f"""
    <model name="obstacle_{i}">
      <static>true</static>
      <link name="obstacle_{i}_link">
        <collision name="obstacle_{i}_collision">
          <geometry>
            <box>
              <size>{obs_width} {obs_height} 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_{i}_visual">
          <geometry>
            <box>
              <size>{obs_width} {obs_height} 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
      <pose>{obs_x} {obs_y} 0.5 0 0 0</pose>
    </model>
"""
    # Close world file content
    world_template += """
  </world>
</sdf>
"""

    # Save the .world file
    with open(output_world_path, 'w') as f:
        f.write(world_template)

    print(f"Gazebo world file generated: {output_world_path}")

# Path to JSON file
json_path = "/home/catkin_ws/src/SlamRobotFollower/slam_data/slam_data.json"  # Replace <username> with your actual username
output_world_path = "/home/catkin_ws/src/SlamRobotFollower/slam_data/slam_world.world"

# Generate world file
generate_gazebo_world(json_path, output_world_path)
