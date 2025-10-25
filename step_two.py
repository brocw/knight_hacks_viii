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

depot = way_points[0]

# For this step since we only need a single vehicle setup for optimization
# So let's do it
num_of_vehicles = 1


# Do the index manager first to figure out the code
manager = pywrapcp.RoutingIndexManager(
    len(distance_matrix), len(num_of_vehicles), depot
)

# Now make the routing model
routing = pywrapcp.RoutingModel(manager)

# https://colab.research.google.com/github/google/or-tools/blob
# /stable/examples/notebook/constraint_solver/tsp_cities.ipynb#scrollTo=code
# This is supposed to be the link to be able to do the part fully, 
# I dont have the brain power rn

