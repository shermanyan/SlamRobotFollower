import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Function to visualize the map
def visualize_robotics_map(json_file_path):
    # Load the JSON file
    with open(json_file_path, 'r') as file:
        map_data = json.load(file)
    
    # Extract goal position
    goal_x = map_data['goal_position']['x']
    goal_y = map_data['goal_position']['y']
    
    # Extract map boundaries (axis-aligned bounding box)
    map_aabb = map_data['map_aabb']
    x_min, x_max, y_min, y_max = map_aabb
    
    # Extract obstacles
    obstacles = map_data['obstacles']
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Robotics Map Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    
    # Plot goal position
    ax.plot(goal_x, goal_y, 'go', markersize=10, label="Goal Position")  # Green circle
    
    # Plot obstacles
    for obstacle in obstacles:
        x0, x1, y0, y1 = obstacle
        rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0, color='red', alpha=0.7)
        ax.add_patch(rect)
    
    # Add legend
    ax.legend()
    
    # Show the plot
    plt.grid(True)
    plt.show()

# Path to the JSON file (replace with your file path)
json_file_path = "/home/cgorey/catkin_ws/src/csci445l/scripts/lab10_map.json"

# Visualize the map
visualize_robotics_map(json_file_path)
