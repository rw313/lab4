## Lab 3
**Rachel Wu (rww2115)**  
**David Lee (jl4397)**

*COMSW4733 Computational Aspects of Robotics*  
*Peter Allen*

#### Dependencies
```
matplotlib==2.2.2
```

#### To Run
Unzip tarball and `cd` into this repository
```
$ cd <path_to_this_repo>
```

Install the dependencies, preferably in a virtualenv
```
$ pip install -r requirements.txt
```

Run the script. Use the `python` command if your default Python version is 3.x
```
$ python3 main.py <distance> <obstacle_path> <start_goal_path> <attempts>
```

The default values for the command line arguments are:

```
- distance (the distance to draw the line between q_near and q_new): 15
- obstacle_path (path to the text file defining the obstacles): ./world_obstacles.txt
- start_goal_path (path to the text file defining the start and goal points): ./start_goal.txt
- attempts (max number of attempts to find points): 50000
```

#### Files and Methods
**Part A**
1. `main.py`: The main driver function adapted from the script given to drawn the world. Instantiates a `UnidirectionalRRT` and calls its methods to create the RRT tree and draw it  
  - `build_obstacle_course()`: Builds and draws the obstacle course
  - `add_start_and_goal()`: Draws the start and end points
  - `build_shapes()`: Builds a representation of obstacles as a list of list of points corresponding to the vertices of the obstacles
2. `rrt.py`: Defines the `UnidirectionalRRT` class and its methods  
  - `UnidirectionalRRT`: A class that builds and draws the RRT. Has the following attributes:
    - Finds
  - `find_and_draw_path()`: Builds and draws the RRT. Main function.
  - `build_rrt()`: Builds the RRT by sampling points for a max of `self.attempts` times
  - `extend_rrt()`: Called by `build_rrt()` to try and extend the tree
  - `draw_shortest_path()`: Draws the shortest path
  - `draw_point()`: Draws a point
  - `draw_line()`: Draws a line between two points
  - `_get_rand_config()`: Gets a random point on the plot
3. `utils.py`: Defines utility functions to determine collisions, get measurements, and find the shortest path  
  - `point_collides()`: Determines if a point collides with the obstacles
  - `line_collides()`: Determine if a line collides with the obstacles
  - `get_distance()`: Gets the euclidean distance between two points
  - `get_point_on_line()`: Gets a point on a line between `start` and `end` that is `distance` away from start
  - `get_unit_vector()`: Gets the unit vector of the vector between `start` and `end`
  - `get_nearest_point()`: Gets the nearest point to `curr_point` among `other_points`
  - `dijkstra()`: Finds the shortest path between `start` and `end` on `graph`


#### Video
