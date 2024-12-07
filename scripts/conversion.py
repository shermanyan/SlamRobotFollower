import xml.etree.ElementTree as ET
import json
import math
import os

def parse_world_file(world_file_path):
    """
    Parse a .world file and extract obstacle and map information.
    
    Args:
        world_file_path (str): Path to the .world file
    
    Returns:
        dict: Parsed world information in JSON-compatible format
    """
    # Parse the XML
    tree = ET.parse(world_file_path)
    root = tree.getroot()
    
    # Initialize data structures
    obstacles = []
    map_boundaries = [-50.0, 50.0, -50.0, 50.0]  # Default large boundary
    goal_position = {"x": 0, "y": 1.5}  # Default goal position
    
    # Function to extract numeric coordinates from a string
    def parse_coordinates(coord_str):
        try:
            return [float(x) for x in coord_str.split()]
        except:
            return []
    
    # Iterate through world elements to find obstacles
    for world in root.findall('.//world'):
        # Look for model or obstacle elements
        for model in world.findall('.//model'):
            # Extract pose information
            pose = model.find('pose')
            if pose is not None:
                coords = parse_coordinates(pose.text)
                if len(coords) >= 3:
                    x, y, z = coords[:3]
                    
                    # Look for link with collision geometry
                    link = model.find('.//link')
                    if link is not None:
                        collision = link.find('.//collision')
                        if collision is not None:
                            geometry = collision.find('.//geometry')
                            if geometry is not None:
                                box = geometry.find('.//box')
                                if box is not None:
                                    size = box.find('size')
                                    if size is not None:
                                        dims = parse_coordinates(size.text)
                                        if len(dims) >= 2:
                                            # Create obstacle representation
                                            half_width = dims[0] / 2
                                            half_height = dims[1] / 2
                                            
                                            obstacle = [
                                                x - half_width, 
                                                x + half_width, 
                                                y - half_height, 
                                                y + half_height
                                            ]
                                            obstacles.append(obstacle)
    
    # If no obstacles found, add some default or placeholder obstacles
    if not obstacles:
        obstacles = [
            [0.2, 0.3, -1.0, -0.9],
            [0.7, 0.8, -1.0, -0.9]
        ]
    
    # Construct the final JSON-like dictionary
    world_data = {
        "goal_position": goal_position,
        "map_aabb": map_boundaries,
        "obstacles": obstacles
    }
    
    return world_data

def convert_world_to_json(world_file_path, output_json_path=None):
    """
    Convert a .world file to a JSON file.
    
    Args:
        world_file_path (str): Path to the input .world file
        output_json_path (str, optional): Path to save the output JSON file
    
    Returns:
        dict: Parsed world information
    """
    # Parse the world file
    world_data = parse_world_file(world_file_path)
    
    # If output path is provided, write to file
    if output_json_path:
        with open(output_json_path, 'w') as f:
            json.dump(world_data, f, indent=4)
    
    return world_data

# Example usage
if __name__ == "__main__":
    # Replace with your actual .world file path
    input_world_file = os.path.join(os.path.expanduser("~"), "catkin_ws/src/SlamRobotFollower/slam_data/my_exploration_world.world")
    output_json_file = os.path.join(os.path.expanduser("~"), "catkin_ws/src/SlamRobotFollower/slam_data/my_world.json")
    
    result = convert_world_to_json(input_world_file, output_json_file)
    print(json.dumps(result, indent=4))