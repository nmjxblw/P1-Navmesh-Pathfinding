from math import inf, sqrt
from heapq import heappop, heappush

blocks = []
blocks_adj = {}


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
    global blocks, blocks_adj
    blocks = mesh["boxes"]  # blocks is a list
    blocks_adj = mesh["adj"]  # blocks_adj is a dictionary

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

    def the_closest_point_on_box(point, box):
        closest_x = max(box[0], min(point[0], box[1]))
        closest_y = max(box[2], min(point[1], box[3]))
        return (closest_x, closest_y)

    def distance_point_to_box(a, b):
        point = the_closest_point_on_box(a, b)
        return get_distance_point_to_point(a, point)

    # some lamnbda functions shortcut
    # get the Euclidean distance between two points.
    get_distance_point_to_point = lambda a, b: sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    )

    # The first step is to obtain the grid where the path is located.

    source_point_box = get_box(source_point)
    destination_point_box = get_box(destination_point)

    # storage the boxes instead of points first.

    path = []
    boxes = {}

    queue = []
    heappush(
        queue, (0, source_point_box, destination_point_box)
    )  # push the source box in the priority queue

    forward = {source_point_box: [source_point_box, 0, source_point, source_point]}
    # { box : [ prev_box , distance , start_point , leave_point ] }
    # forward[box][0] = prev_point (x1,x2,y1,y2)
    # forward[box][1] = distance (float:123.5)
    # forward[box][2] = start_point (x,y) this point is inside the prev_box
    # forward[box][3] = leave_point (x,y) this point is on the edge of both prev_box and current_box
    backward = {
        destination_point_box: [
            destination_point_box,
            0,
            destination_point,
            destination_point,
        ]
    }  # { point : [ prev_point , distance , enter_point , leave_point ] }

    while queue:
        priority, forward_box, backward_box = heappop(queue)
        boxes[forward_box] = None
        boxes[backward_box] = None
        forward_start_point = forward[forward_box][3]
        backward_start_point = backward[backward_box][3]
        priority_forward_backward = get_distance_point_to_point(
            forward_start_point, backward_start_point
        )

        # when boxes meet each others or either direction of search encounters territory already seen by the other direction.
        if (
            forward_box == backward_box
            or forward_box in backward
            or backward_box in forward
        ):
            print("find a path!")

            def get_path(block):
                boxes[block] = None
                path.insert(0, forward[block][3])
                path.append(backward[block][3])
                get_forward_path(forward[block][0])
                get_backward_path(backward[block][0])

            def get_forward_path(block):
                boxes[block] = None
                if forward[block][0] == source_point_box:
                    boxes[source_point_box] = None
                    path.insert(0, source_point)
                    return
                path.insert(0, forward[block][3])

                get_forward_path(forward[block][0])

            def get_backward_path(block):
                boxes[block] = None
                if backward[block][0] == destination_point_box:
                    boxes[destination_point_box] = None
                    path.append(destination_point)
                    return
                path.append(backward[block][3])

                get_backward_path(backward[block][0])

            if forward_box == backward_box or (forward_box in backward):
                get_path(forward_box)
            else:
                get_path(backward_box)

            break

        for forward_next_box in blocks_adj[forward_box]:
            new_forward_cost = forward[forward_box][1] + distance_point_to_box(
                forward_start_point, forward_next_box
            )
            forward_leave_point = the_closest_point_on_box(
                forward_start_point, forward_next_box
            )  # get a point where to leave the forward_box and enter the forward_next_box
            if (
                forward_next_box not in forward
                or new_forward_cost < forward[forward_next_box][1]
            ):
                forward[forward_next_box] = [
                    forward_box,
                    new_forward_cost,
                    forward_start_point,
                    forward_leave_point,
                ]
                priority = new_forward_cost + priority_forward_backward
                heappush(queue, (priority, forward_next_box, backward_box))

        for backward_next_box in blocks_adj[backward_box]:
            new_backward_cost = backward[backward_box][1] + distance_point_to_box(
                backward_start_point, backward_next_box
            )
            backward_leave_point = the_closest_point_on_box(
                backward_start_point, backward_next_box
            )
            if (
                backward_next_box not in backward
                or new_backward_cost < backward[backward_next_box][1]
            ):
                backward[backward_next_box] = [
                    backward_box,
                    new_backward_cost,
                    backward_start_point,
                    backward_leave_point,
                ]
                priority = new_backward_cost + priority_forward_backward
                heappush(queue, (priority, forward_box, backward_next_box))

    return path, boxes.keys()
