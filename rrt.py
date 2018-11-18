import matplotlib.pyplot as plt
import matplotlib.patches as patches

import utils
import random

class UnidirectionalRRT():
    def __init__(self, start, goal, num_attempts, distance, obstacles, max_x, max_y, ax, is_bi_dir=False):
        self.start = start
        self.goal = goal
        self.num_attempts = num_attempts
        self.distance = distance
        self.obstacles = obstacles
        self.adj_matrix = {start: []}
        self.max_x = max_x
        self.max_y = max_y
        self.ax = ax
        self.bias_every = int(self.num_attempts*.1) # 10% bias towards the goal
        self.sleep = 0.00001
	self.is_bi_dir = is_bi_dir
	self.adj_matrix_goal = {goal: []}

    def find_and_draw_path(self): 
	if self.is_bi_dir:
		self.build_bi_rrt()
	else:
	        self.build_rrt()

        shortest_path = utils.dijkstra(self.adj_matrix, self.start, self.goal)
        if shortest_path is None:
            print("Path not found")
            return

        self.draw_shortest_path(shortest_path)
        plt.show()
        return

    def build_bi_rrt(self):
	for i in range(self.num_attempts):
	    rand_point_goal = self._get_rand_config() if i%self.bias_every != 0 else self.start
	    q_new = self.extend_rrt_bi(rand_point_goal)
	    #find the closest point from the other tree 	
	    if q_new:
	        start_tree_near = utils.get_nearest_point(q_new, self.adj_matrix.keys()) 
	    	if utils.get_distance(start_tree_near, q_new) < self.distance and not utils.line_collides(q_new, start_tree_near, self.obstacles, self.distance):
		    self.adj_matrix[q_new] = [start_tree_near]
	   	    self.adj_matrix[start_tree_near].append(q_new)
		    self.draw_line(q_new, start_tree_near)
		    self.adj_matrix = dict(self.adj_matrix.items() + self.adj_matrix_goal.items()) 
		    #self.adj_matrix.update(self.adj_matrix_goal)
		    print("Found path after generation {} random configs".format(i))
		    return 
	
	    rand_point_start = self._get_rand_config() if i%self.bias_every != 0 else self.goal 
	    q_new = self.extend_rrt(rand_point_start)
	    if q_new:
		goal_tree_near = utils.get_nearest_point(q_new, self.adj_matrix_goal.keys())
		if utils.get_distance(goal_tree_near, q_new) < self.distance and not utils.line_collides(q_new, goal_tree_near, self.obstacles, self.distance):
		    if q_new not in self.adj_matrix:
			self.adj_matrix[q_new] = []
		    if goal_tree_near not in self.adj_matrix:
			self.adj_matrix[goal_tree_near] = []

		    self.adj_matrix[q_new] = [goal_tree_near]
		    self.adj_matrix[goal_tree_near].append(q_new)
		    self.draw_line(q_new, goal_tree_near) 
		    self.adj_matrix = dict(self.adj_matrix.items() + self.adj_matrix_goal.items()) 
		    print("Found path after generation {} random configs".format(i))
		    return
		

	return True
    def extend_rrt_bi(self, rand_point):
	q_near = utils.get_nearest_point(rand_point, self.adj_matrix_goal.keys())
	q_new = utils.get_point_on_line(q_near, rand_point, self.distance)

	if utils.point_collides(q_new, self.obstacles) or utils.line_collides(q_near, q_new, self.obstacles, self.distance):
		return None

	if q_near not in self.adj_matrix_goal:
		self.adj_matrix_goal[q_near] = []
	if q_new not in self.adj_matrix_goal:
		self.adj_matrix_goal[q_new] = []
	
	self.adj_matrix_goal[q_near].append(q_new)
	self.adj_matrix_goal[q_new].append(q_near)

	self.draw_point(q_new, color='orange')
	self.draw_line(q_near, q_new, color='red')
	return q_new

    def build_rrt(self):
        for i in range(self.num_attempts):
            # bias towards goal every self.bias_every iterations
            rand_point = self._get_rand_config() if i%self.bias_every != 0 else self.goal
            q_new = self.extend_rrt(rand_point)

            if q_new and utils.get_distance(self.goal, q_new) < self.distance and not utils.line_collides(q_new, self.goal, self.obstacles, self.distance):
                self.adj_matrix[self.goal] = [q_new]
                self.adj_matrix[q_new].append(self.goal)
                print("Found path after generating {} random configs".format(i))
                return

        return True

    def extend_rrt(self, rand_point):
        q_near = utils.get_nearest_point(rand_point, self.adj_matrix.keys())
        q_new = utils.get_point_on_line(q_near, rand_point, self.distance)

        if utils.point_collides(q_new, self.obstacles) or utils.line_collides(q_near, q_new, self.obstacles, self.distance):
            return None

        if q_near not in self.adj_matrix:
            self.adj_matrix[q_near] = []
        if q_new not in self.adj_matrix:
            self.adj_matrix[q_new] = []

        self.adj_matrix[q_near].append(q_new)
        self.adj_matrix[q_new].append(q_near)

        self.draw_point(q_new)
        self.draw_line(q_near, q_new)
        return q_new

    def draw_shortest_path(self, edges):
        if len(edges) < 2:
            return
        prev = edges[0]
        for curr in edges[1:]:
            self.draw_line(prev, curr, color='yellow')
            prev = curr

    def draw_point(self, point, color='blue'):
        self.ax.plot(point[0], point[1], marker='o', color=color, markersize=2)
        plt.pause(self.sleep)

    def draw_line(self, start, end, color='green'):
        self.ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linewidth=1)
        plt.pause(self.sleep)

    def _get_rand_config(self):
        return random.uniform(0, self.max_x), random.uniform(0, self.max_y)
