# I am going to build an unconstrained tour
# Let it try to optimize without a specific plan
# Or idea, so we will see

import ortools
import numpy as np
import math

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


# Get the distance matrix from the code that was downloaded and provided
distance_matrix = np.load("distance_matrix.npy")

# This where I find the start point or end point for the optmized graph
# A way point is a point that is in a way of the main point
way_points = np.load("points_lat_long.npy")

depot = int(0)
num_of_vehicles = 1
max_mission_distance = 37725

# Do the index manager first to figure out the code
manager = pywrapcp.RoutingIndexManager(
    len(distance_matrix), num_of_vehicles, int(depot)
)

# Now make the routing model
routing = pywrapcp.RoutingModel(manager)


# Now we need to obtain the distance using nodes, rather than specific indexes
# From the distance matrix
def distance_between_nodes(beginning_index, ending_index):
    beginning_node = manager.IndexToNode(beginning_index)
    ending_node = manager.IndexToNode(ending_index)

    return int(round(distance_matrix[beginning_node][ending_node]))


# RERUN THIS TEST TO CONFIRM YOUR OFF-DIAGONAL VALUES ARE NON-ZERO
print("Distance from node 0 to 1:", distance_matrix[0][1])
print("Distance from node 2778 to 2777:", distance_matrix[2778][2777])
print("Distance from node 1 to 0:", distance_matrix[1][0])


# Ok we need to continue to construct the rest of the graph for the route
# This function will be use to have a function to find distance between nodes,
# using it as a rule to apply to add of the values in the matrix array
transit_callback = routing.RegisterTransitCallback(distance_between_nodes)

# Define cost of each edge or line
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback)

# Building and unconstrained tour, this is correct because an unconstrained
# tour is too ok for calcuating the distance between two incidies
# Still heuristic tour
search_parameters = pywrapcp.DefaultRoutingSearchParameters()

# Find the most optimal parameters
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)


# Find the actual most optimal path this way
solution = routing.SolveWithParameters(search_parameters)
