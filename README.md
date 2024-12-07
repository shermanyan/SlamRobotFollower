## About Project
The aim of this project was to create a multi robot system with two robots in which they could use data sharing to optimize their behavior. In this case, one robot performed a SLAM mapping of the environment and saved the data collected from that as a 3D world file, which was converted to a 2D JSON map. This data was then used by the second robot to optimize a path out of the maze that it was in using an RTT algorithm without having to do any active object detection. The components used in this scenario were the robots, and then the SLAM and RTT algorithm implementations and finally the data transferring mechanism.


## System Architecture Description
Our project’s system architecture involves two TurtleBot robots, each performing specialized tasks. The architecture is divided into two primary components: SLAM and Exploration and Map Processing and Path Planning.
#### SLAM and Exploration:
This component leverages the TurtleBot slam_toolbox to perform Simultaneous Localization and Mapping (SLAM) while using navigation nodes to explore the environment.
A Python script, ```explore.py```, implements random dispersion sampling to determine navigation points. These points along with a boundary obstacle are dynamically published as goals to the ```/map``` topic. The script uses Manhattan distance to prioritize unexplored points and updates goals accordingly with tolerance as the robot navigates. This ensures complete map coverage.
#### Map Processing and Path Planning:
After completing the exploration phase, the SLAM-generated map is exported and a Python script converts the map data into a ```.json``` format. Using a Rapidly-Exploring Random Tree (RRT) algorithm, the shortest path to the target destination is computed.


## How to Run
#### Step 1: Launch sim world 
```$ roslaunch SlamRobotFollower slam_sim.launch```

#### Step 2: Run explore script
```$ rosrun SlamRobotFollower explore.py ```

#### Step 3: Save world

#### Step 4: Process map
```$ python3 conversion.py```


```$ python3 visualize.py```

#### Step 5: Launch w
```$ roslaunch turtlebot3_bringup turtlebot3_robot.launch ```

### Step 6: Execute RRT and cmd_vel to robot
```$ rosrun SlamRobotFollower lab10_map.py```


## Authors
* Kip Gorey | [cgorey@usc.edu](mailto:cgorey@usc.edu) | [iotsak1r](https://github.com/kipgorey)
* Sherman Yan | [shermany@usc.edu](mailto:shermany@usc.edu) | [shermanyan](https://github.com/shermanyan)
* Mikael Yikum | [yikum@usc.edu](mailto:yikum@usc.edu) | [YikumMikael](https://github.com/YikumMikael)





