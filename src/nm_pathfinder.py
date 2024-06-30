from collections import deque

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
    
    start_box = next((box for box in mesh["boxes"] if in_bounds(source_point, box)), None)
    destination_box = next((box for box in mesh["boxes"] if in_bounds(destination_point, box)), None)
    
    
    if start_box == destination_box:
        print ("first")
        return [], [start_box]
    if not start_box or not destination_box:
        print ("second")
        return [], []
    
    path = []
    boxes = []

    boxes = BFS(mesh["adj"], start_box, destination_box)
    path = [((box[0] + box[1]) / 2.0, (box[2] + box[3]) / 2.0) for box in boxes]
    print("Boxes: ", boxes, " Path: ", path)
    return path, boxes

def in_bounds(point, box):
    x1, x2, y1, y2 = box
    if point[0] >= x1 and point[0] < x2:
        if point[1] >= y1 and point[1] < y2:
            return True
    return False

def BFS(boxes, starting_box, destination_box):
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
    return []

def get_path(history, box):
    if box == []:
        return []
    return get_path(history, history[box]) + [get_point(box, history.get(None))]

def get_point(a_box, b_box):
    if not a_box:
        return None
    if not b_box:
        return (a_box[0] + a_box[1]) / 2.0
    x1 = max(a_box[0], b_box[0])
    x2 = max(a_box[1], b_box[1])
    