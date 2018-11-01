import math

def point_collides(point, shapes):
    # adapted from https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
    x, y = point

    for shape in shapes:
        num_ray_intersections = 0
        prev_x, prev_y = shape[0]

        for curr_x, curr_y in shape[1:] + [shape[0]]:
            if ((curr_y > y) != (prev_y > y) and \
                (x < (prev_x - curr_x)* (y-curr_y)/(prev_y-curr_y) + curr_x)):
                    num_ray_intersections += 1
            prev_x, prev_y = curr_x, curr_y

        if num_ray_intersections%2 == 1:
            return True

    return False

def line_collides(start, end, obstacles, distance, step_size=0.01):
    # sample points on the line between q_near and q_new
    # i corresponds to a distance between 1 and self.distance, inclusive
    for i in range(1, distance* int(1/step_size)+1):
        q_sample = get_point_on_line(start, end, i*step_size)
        if point_collides(q_sample, obstacles):
            return True
    return False


def get_distance(point1, point2):
    # Euclidean distance between 2 points represented as tuples
    x1, y1 = point1
    x2, y2 = point2

    return math.sqrt((y2-y1)**2 + (x2-x1)**2)


def get_point_on_line(start, end, distance):
    start_x, start_y = start
    end_x, end_y = end
    unit_vector = get_unit_vector(start, end)
    return start_x + unit_vector[0]*distance, start_y + unit_vector[1]*distance

def get_unit_vector(start, end):
    magnitude = get_distance(start, end)
    if magnitude == 0:
        return start

    start_x, start_y = start
    end_x, end_y = end
    unit_vector = ((end_x-start_x)/magnitude, (end_y-start_y)/magnitude)
    return unit_vector

def get_nearest_point(curr_point, other_points):
    if not other_points:
        return curr_point

    min_distance = float('inf')
    nearest_point = None

    for other_point in other_points:
        distance = get_distance(curr_point, other_point)
        if distance < min_distance:
            min_distance = distance
            nearest_point = other_point
    return nearest_point

def dijkstra(graph, start, end):
	dp = {start: (0, None)}
	curr = start
	seen = set()

	while curr != end:
		seen.add(curr)
		neighbors = graph[curr]

		for neighbor in neighbors:
			dist_to_neighbor = get_distance(curr, neighbor) + dp[curr][0]
			if neighbor not in dp or dp[neighbor][0] > dist_to_neighbor:
				dp[neighbor] = (dist_to_neighbor, curr)
		curr = None
		min_dist = float('inf')
		for node in dp:
			if node not in seen and dp[node][0] < min_dist:
				min_dist = dp[node][0]
				curr = node
		if not curr:
			return None

	path = []
	while curr:
		path.append(curr)
		curr = dp[curr][1]
	return path[::-1]
