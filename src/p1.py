from p1_support import load_level, show_level, save_level_costs
from math import sqrt, inf
from heapq import heappop, heappush



def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    print(f'Initial position is {initial_position} and Destination is {destination}')
    print(f'Level has waypoints {graph["waypoints"]}')
    h = []
    heappush(h, (initial_position, 0))
    came_from = dict()
    cost_so_far = dict()
    came_from[initial_position] = None
    cost_so_far[initial_position] = 0.0

    while not h == []:
        print()
        current = heappop(h)[0]

        if current == destination:
            break
        for next_h in adj(graph, current):
            print(f'adj_cell {next_h}')
            next_cost = next_h[0]
            next_coord = next_h[1]
            new_cost = cost_so_far[current] + next_cost

            if next_coord not in cost_so_far or new_cost < cost_so_far[next_coord]:
                cost_so_far[next_coord] = new_cost
                priority = new_cost
                heappush(h, (next_coord, priority))
                came_from[next_coord] = current
    end = destination
    path = []
    if end not in came_from:
        return None
    while end is not initial_position:
        print(f'path is currently {path} with current at current cell {end}')
        path.append(end)
        end = came_from[end]
    return path
    pass


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    pass


def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    transformations = [(1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    edges = []
    for transform in transformations:
        print(f'Current Cell is {cell} and + {transform} = {tuple(map(lambda i, j: i + j, cell, transform))}')
        adj_cell = tuple(map(lambda i, j: i + j, cell, transform))
        adj_cost = 0.0
        curr_cost = level["spaces"][cell]
        if adj_cell in level["spaces"]:
            print(f'{adj_cell} is a space and has cost {level["spaces"][adj_cell]}')
            adj_cost = level["spaces"][adj_cell]
        elif adj_cell in level["walls"]:
            print(f'{adj_cell} is a wall')
            continue
        elif adj_cell in level["waypoints"]:
            print(f'{adj_cell} is a waypoint')
            adj_cost = 1.0
        if 0 in transform:
            edge_cost = .5 * adj_cost + .5 * curr_cost
        else:
            edge_cost = (.5 * sqrt(2)) * adj_cost + (.5 * sqrt(2)) * curr_cost
        heappush(edges, (edge_cost, adj_cell))
    print(f'edges for cell {cell} with costs are {edges}')
    return edges
    pass


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'example.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')
