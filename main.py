# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
import rrt

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path


def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

def build_shapes(obstacle_path):
    shapes = []
    with open(obstacle_path) as f:
        f.readline()
        f.readline()
        buff = []
        for line in f:
            arr = line.split()
            if len(arr) == 1:
                shapes.append(buff)
                buff = []
            else:
                buff.append(tuple(map(int, arr)))
    shapes.append(buff)
    return shapes


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set",
                        nargs='?',
                        default='./world_obstacles.txt',
                        type=str)
    parser.add_argument('start_goal_path',
                        help="File path for start and goal set",
                        nargs='?',
                        default='./start_goal.txt',
                        type=str)
    parser.add_argument('distance',
                        help="Distance to grow tree branches",
                        nargs='?',
                        default=10,
                        type=int)
    parser.add_argument('attempts',
                        help="Number of attempts to expand the tree",
                        nargs='?',
                        default=500,
                        type=int)
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)
    shapes = build_shapes(args.obstacle_path)

    print("Initializing RRT")
    rrt = rrt.UnidirectionalRRT(start, goal, args.attempts, args.distance, shapes, 600, 600, ax)
    print("Building tree")
    rrt.build_rrt()
    rrt.display_rrt()
