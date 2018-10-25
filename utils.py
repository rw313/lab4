import math

def get_distance(point1, point2):
    # Euclidean distance between 2 points represented as tuples
    x1, y1 = point1
    x2, y2 = point2

    return math.sqrt((y2-y1)**2 + (x2-x1)**2)

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
