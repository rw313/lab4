import utils

class RRT():
    def __init__(self, start, goal, num_attempts, distance, obstacles):
        self.start = start
        self.goal = goal
        self.num_attempts = num_attempts
        self.distance = distance
        self.obstacles = obstacles
        self.adj_matrix = {start: []}

    def build_rrt(self, root=self.start):
        for _ in range(self.num_attempts):
            rand_point = self._get_rand_config(root)
            self.extend_rrt(self, rand_point)

        return True

    def extend_rrt(self, new_point):
        nearest_point = utils.get_nearest_point(new_point, self.adj_matrix.keys())

    def _get_rand_config(self, root):
        pass
