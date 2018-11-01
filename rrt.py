import matplotlib.pyplot as plt
import matplotlib.patches as patches

import utils
import random

class UnidirectionalRRT():
    def __init__(self, start, goal, num_attempts, distance, obstacles, max_x, max_y, ax):
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

    def find_and_draw_path(self):
        self.build_rrt()
        shortest_path = utils.dijkstra(self.adj_matrix, self.start, self.goal)
        if shortest_path is None:
            print("Path not found")
            return

        self.draw_nodes()
        self.draw_edges()

        self.draw_shortest_path(shortest_path)
        plt.show()
        return

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
        return q_new


    def draw_shortest_path(self, edges):
        if len(edges) < 2:
            return
        prev = edges[0]
        for curr in edges[1:]:
            self.draw_line(prev, curr, color='yellow')
            prev = curr

    def draw_nodes(self):
        for node in self.adj_matrix:
            self.draw_point(node)

    def draw_edges(self):
        drawable_edges = set()
        for node, edges in self.adj_matrix.items():
            for edge in edges:
                if (node, edge) not in drawable_edges and (edge, node) not in drawable_edges:
                    drawable_edges.add((node, edge))
        for edge in drawable_edges:
            self.draw_line(*edge)

    def draw_point(self, point, color='blue'):
        self.ax.plot(point[0], point[1], marker='o', color=color, markersize=2)

    def draw_line(self, start, end, color='green'):
        self.ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linewidth=1)

    def _get_rand_config(self):
        return random.uniform(0, self.max_x), random.uniform(0, self.max_y)
