from math import inf, sqrt
from heapq import heappop, heappush
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

    path = []
    boxes = {}

    path.push(source_point)
    queue = []

    heappush(queue,(0,source_point))

    while queue:
        break

    return path, boxes.keys()

def heuristic(a, b):
     # Manhattan distance on a square grid
   return abs(a.x - b.x) + abs(a.y - b.y)

def calc_cost(box1,box2):
    return 