from math import inf, sqrt
from heapq import heappop, heappush

blocks = []
blocks_adj = {}
min_adj_box = None
max_adj_box = None
max_adj = -inf
min_adj = inf


def find_path(source_point, destination_point, mesh):
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

    # shortcut and reset global
    global blocks, blocks_adj, min_adj, max_adj, min_adj_box, max_adj_box
    blocks = mesh["boxes"]  # blocks is a list
    blocks_adj = mesh["adj"]  # blocks_adj is a dictionary
    for key in blocks_adj.keys():
        num_of_adj = len(blocks_adj[key])
        if num_of_adj > max_adj:
            max_adj = num_of_adj
            max_adj_box = key
        if num_of_adj < min_adj:
            min_adj = num_of_adj
            min_adj_box = key

    # return true if point is in the box
    def in_box(point, box):
        if (point[0] >= box[0] and point[0] <= box[1]) and (
            point[1] >= box[2] and point[1] <= box[3]
        ):
            return True
        # point is not in the box, return false
        return False

    # return the box coordinate if the point is in the mesh
    def get_box(point):
        for block in blocks:
            if in_box(point, block):
                return block
        # else return a empty tuple
        return ()

    # get box center point
    get_center = lambda box: ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)
    # get the Euclidean distance between two points.
    get_distance = lambda a, b: sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    # heuristic
    heuristic = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])

    # The first step is to obtain the grid where the path is located.

    source_point_box = get_box(source_point)
    destination_point_box = get_box(destination_point)

    # storage the boxes instead of points first.

    path = [source_point]
    boxes = {source_point_box: None}  # a dict to storage the points came from

    queue = []
    heappush(queue, (0, source_point_box))  # push the source box in the priority queue
    cost_so_far = {source_point_box: 0}  # a dict to storage the cost
    came_from = {source_point_box: None}

    while queue:
        priority, curr_box = heappop(queue)

        if curr_box == destination_point_box:

            def get_path(block):
                if came_from[block] == None:
                    if block == destination_point_box:
                        path.append(destination_point)
                    return
                else:
                    boxes[block] = came_from[block]
                    get_path(came_from[block])
                    if block != destination_point_box:
                        if block != source_point_box:
                            path.append(get_center(block))
                    else:
                        path.append(destination_point)

            get_path(curr_box)
            print(path)
            break

        for next_box in blocks_adj[curr_box]:
            # calc new cost =  old cost + distance
            new_cost = cost_so_far[curr_box] + get_distance(
                get_center(curr_box), get_center(next_box)
            )
            # if it has smaller cost or its new path, store it
            if next_box not in cost_so_far or new_cost < cost_so_far[next_box]:
                # updata the cost
                cost_so_far[next_box] = new_cost
                priority = new_cost + get_distance(
                    destination_point, get_center(next_box)
                )
                heappush(queue, (priority, next_box))
                came_from[next_box] = curr_box

    return path, boxes.keys()
