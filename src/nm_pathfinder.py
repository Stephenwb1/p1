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
    boxes = search(mesh["adj"], start_box, destination_box)

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

def search(boxes, starting_box, destination_box):
    #to convert this to A*, we need to track movement costs
    frontier = PriorityQueue()
    frontier.put(((0, 0), starting_box, 'f'))
    frontier.put(((0, 0), destination_box, 'b'))
    forward_reached = dict()
    forward_previous = dict()
    backward_previous = dict()
    backward_reached = dict()
    forward_previous[starting_box] = []
    backward_previous[destination_box] = []
    forward_reached[starting_box] = (0, 0)
    backward_reached[destination_box] = (0, 0)
    starting_point = get_point(starting_box)
    destination_point = get_point(destination_box)
    while frontier:
        current_priority, current, dir = frontier.get()
        dest_reached = (dir == 'f' and current == destination_box) or (dir == 'b' and current == starting_box)
        if dest_reached or (current in forward_reached and current in backward_reached):
            # CHANGE
            # Base Cases: Boxes are touching
            # Boxes are seperated by one box
            # Steps: Traverse from dest to midpoint
            # Traverse from midpoint to start
            # append dest path to start path
            # get_path(forward_previous, destination_box)

            path = []
            #if boxes are touching
            if destination_box in boxes[starting_box]: 
                path.append(starting_box)
                path.append(destination_box)
                return path

            path = get_path(forward_previous, current)
            path += get_path(backward_previous, current)[::-1]
            if path.count(current) > 1:
                path.remove(current)
            return path
        for adj_box in boxes[current]:
            entry_point = get_point(current, adj_box) # Fallback estimate is entry point from center of prev box
            prev_entry_point = get_point(current) # Fallback estimate is center of prev box
            if dir == 'f':
                if forward_previous.get(current, None):
                    prev_entry_point = get_point(forward_previous[current], current)
                    entry_point = get_point(current, adj_box, prev_entry_point)
                distance = current_priority[0] + heuristic(prev_entry_point, entry_point)
                heuristic_value = heuristic(entry_point, destination_point)
                if not adj_box in forward_reached or forward_reached[adj_box][0] > distance:
                    forward_previous[adj_box] = current
                    forward_reached[adj_box] = (distance, heuristic_value)
                    frontier.put(((distance, heuristic_value), adj_box, 'f'))
            else:
                if backward_previous.get(current, None):
                    prev_entry_point = get_point(backward_previous[current], current)
                    entry_point = get_point(current, adj_box, prev_entry_point)
                distance = current_priority[0] + heuristic(prev_entry_point, entry_point)
                heuristic_value = heuristic(entry_point, starting_point)
                if not adj_box in backward_reached or backward_reached[adj_box][0] > distance:
                    backward_previous[adj_box] = current
                    backward_reached[adj_box] = (distance, heuristic_value)
                    frontier.put(((distance, heuristic_value), adj_box, 'b'))

    return [], []

#def a_star(boxes, starting_box, destination_box):
    #frontier = PriorityQueue()
    #frontier.put(starting_box, 0)

def get_path(history: dict, box) -> list:
    if box == None or box == []:
        return []
    return get_path(history, history.get(box, None)) + [box]

def get_point(a_box, b_box=None, start_point=None, end_point=None):
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
    mid_line_slope = (start_point[1] - end_point[1])
    if start_point[0] - end_point[0] == 0:
        mid_line_slope /= (0.001) #y / x
    else:
        mid_line_slope /= (start_point[0] - end_point[0]) #y / x  
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
    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
    
def clamp(value, start, end):
    if start > end:
        temp = start
        start = end
        end = temp
    return max(min(value, end), start)