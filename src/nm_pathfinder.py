from collections import deque
from queue import PriorityQueue
#from Dijkstra import Dijkstra_forward_search
import math
import functools

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    #variables
    start_box = next((box for box in mesh["boxes"] if in_bounds(source_point, box)), None)
    destination_box = next((box for box in mesh["boxes"] if in_bounds(destination_point, box)), None)
    path = []
    boxes = []
    
    #check if both source and destination are in the same box, or if there is no path
    if start_box == destination_box:
        return [source_point, destination_point], [start_box]
    if not start_box or not destination_box:
        print ("No Path Found!")
        return [], []

    #BFS to get boxes
    boxes = BFS(mesh["adj"], start_box, destination_box)

    # center = lambda box: ((box[0] + box[1]) / 2.0, (box[2] + box[3]) / 2.0)
    # path = [center(start_box), center(destination_box)]

    path = [source_point]
    for i in range(1, len(boxes)):
        start_point = None
        end_point = None
        if path:
            start_point = path[-1]
        if boxes[i] == destination_box:
            end_point = destination_point
        path.append(get_point(boxes[i-1], boxes[i], start_point, end_point))
    path.append(destination_point)
    print("Boxes: ", boxes, " Path: ", path) #print statement for testing
    return path, boxes
#find_path end========================================

def in_bounds(point, box):
    x1, x2, y1, y2 = box
    if point[0] >= x1 and point[0] < x2:
        if point[1] >= y1 and point[1] < y2:
            return True
    return False

def BFS(boxes, starting_box, destination_box):
    #to convert this to A*, we need to track movement costs
    frontier = deque()
    frontier.append(starting_box)
    reached = set()
    previous = dict()
    previous[starting_box] = []

    while frontier:
        current = frontier.popleft()
        reached.add(current)
        for box in boxes[current]:
            if not box in reached:
                previous[box] = current
                frontier.append(box)
            if box == destination_box:
                return get_path(previous, destination_box)
    return [], []

#def a_star(boxes, starting_box, destination_box):
    #frontier = PriorityQueue()
    #frontier.put(starting_box, 0)

def get_path(history, box):
    if box == []:
        return []
    return get_path(history, history[box]) + [box]

def get_point(a_box, b_box, start_point=None, end_point=None):
    # lambda functions to find center of box and to find midline
    center = lambda box: ((box[0] + box[1]) / 2.0, (box[2] + box[3]) / 2.0) #(x, y)
    mid_line = lambda x: mid_line_slope * x + mid_line_offset

    #check if boxes are missing, and define start / end points
    if not a_box:
        return None
    if not b_box:
        return center(a_box)
    if not start_point:
        start_point = center(a_box)
    if not end_point:
        end_point = center(b_box)

    #computes slope and offset of line passing through start and end points
    mid_line_slope = (start_point[1] - end_point[1]) / (start_point[0] - end_point[0]) #y / x 
    mid_line_offset = start_point[1] - mid_line_slope * start_point[0]
    
    overlap_point_a = max(a_box[0], b_box[0]), max(a_box[2], b_box[2]) #(x1, y1)
    overlap_point_b = min(a_box[1], b_box[1]), min(a_box[3], b_box[3]) #(x2, y2)
    
    if overlap_point_a[0] == overlap_point_b[0]:
        return overlap_point_a[0], clamp(mid_line(overlap_point_a[0]), overlap_point_a[1], overlap_point_b[1])
    else:
        x = (overlap_point_a[1] - mid_line_offset) / mid_line_slope
        return clamp(x, overlap_point_a[0], overlap_point_b[0]), overlap_point_a[1]
    

def heuristic(point_a, point_b):
    (x1, y1) = point_a
    (x2, y2) = point_b
    return abs(x1[0] - x2[0]) + abs(y1[2] - y2[2]) 
    #return math.sqrt(pow(point_a[0] - point_b[0], 2) + pow(point_a[1] - point_b[1], 2))
    
def clamp(value, start, end):
    if start > end:
        temp = start
        start = end
        end = temp
    return max(min(value, end), start)