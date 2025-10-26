from shapely.wkt import loads as wkt_loads
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.sparse.csgraph import shortest_path

import plotly.graph_objects as go
import numpy as np
import sys

import visual

# CONSTANTS
MAX_MISSION_DISTANCE = 37725
NUM_DRONES = 250
SOLVER_LIMIT_TIME_SEC = 120


def calculate_vrp(dist_matrix, photo_coords):
    depot_index = 0

    nodes_of_interest = np.concatenate([[depot_index], photo_coords])
    num_locations = len(nodes_of_interest)

    solver_idx_to_waypoint_idx = {i: nodes_of_interest[i] for i in range(num_locations)}
    sub_dist_matrix = dist_matrix[np.ix_(nodes_of_interest, nodes_of_interest)]

    manager = pywrapcp.RoutingIndexManager(num_locations, NUM_DRONES, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(sub_dist_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    routing.AddDimension(
        transit_callback_index, 0, MAX_MISSION_DISTANCE, True, "Distance"
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(SOLVER_LIMIT_TIME_SEC)

    print("Solving VRP...")
    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        print("No solution found!")
        return None

    print(f"Solution found! Total distance: {solution.ObjectiveValue()} feet.")

    return solution, manager, routing, solver_idx_to_waypoint_idx


def get_full_path(from_node_orig, to_node_orig, predecessors_matrix, n_max):
    path = []
    curr = to_node_orig

    if from_node_orig == to_node_orig:
        return [from_node_orig]

    visted_in_segment = set()

    while curr != from_node_orig:
        path.append(curr)
        visted_in_segment.add(curr)
        prev = predecessors_matrix[from_node_orig][curr]

        if prev < 0 or prev >= n_max:
            if prev == from_node_orig:
                pass
            else:
                print(
                    f"Error: Path reconstruction found invalid node {prev} from predecessors[{from_node_orig}][{curr}]"
                )
                return None

        if prev == from_node_orig:
            path.append(from_node_orig)
            return path[::-1]

        if prev == curr:
            print(f"Path reconstruction failed (self-cycle) at {curr}")
            return None

        if prev in visted_in_segment:
            print(
                f"Path reconstruction failed (multi-node cycle) from {from_node_orig} to {to_node_orig}"
            )

        curr = prev

    return None


def create_missions(solution_package, predecessors, n_max_valid):
    solution, manager, routing, solver_idx_to_waypoint_idx = solution_package
    all_missions_detailed = []
    all_missions_stops = []

    for vehicle_id in range(NUM_DRONES):
        index = routing.Start(vehicle_id)

        mission_stops_solver = []
        while not routing.IsEnd(index):
            node_index_solver = manager.IndexToNode(index)
            mission_stops_solver.append(node_index_solver)
            index = solution.Value(routing.NextVar(index))

        mission_stops_solver.append(manager.IndexToNode(index))

        if len(mission_stops_solver) > 2:
            mission_stops_original = [
                solver_idx_to_waypoint_idx[i] for i in mission_stops_solver
            ]
            all_missions_stops.append(mission_stops_original)

            current_full_mission = []

            for i in range(len(mission_stops_original) - 1):
                start_node = mission_stops_original[i]
                end_node = mission_stops_original[i + 1]

                segment = get_full_path(start_node, end_node, predecessors, n_max_valid)

                if not segment:
                    # print(f"Skipping bad segment: {start_node} to {end_node}")
                    segment = [start_node, end_node]

                if i == 0:
                    current_full_mission.extend(segment)
                else:
                    current_full_mission.extend(segment[1:])

            all_missions_detailed.append(current_full_mission)

    print(f"Created {len(all_missions_detailed)} missions.")
    return all_missions_detailed


def validate(distance_matrix, all_missions_detailed, valid_photo_indices):
    visited_nodes = set()
    for path in all_missions_detailed:
        visited_nodes.update(path)

    required_nodes = set(valid_photo_indices)
    missing_nodes = required_nodes - visited_nodes

    if not missing_nodes:
        print("Validation Success: All required photo waypoints are covered.")
    else:
        print(f"Validation Failed: Missing {len(missing_nodes)} waypoints.")

    max_dist_violation = False
    total_dist_all_missions = 0

    for i, path in enumerate(all_missions_detailed):
        mission_dist = 0
        if not path:
            continue

        for j in range(len(path) - 1):
            u = path[j]
            v = path[j + 1]
            mission_dist += distance_matrix[u][v]

        total_dist_all_missions += mission_dist
        if mission_dist > MAX_MISSION_DISTANCE:
            print(f"Mission {i} failed: {mission_dist} > {MAX_MISSION_DISTANCE}")
            max_dist_violation = True

    if not max_dist_violation:
        print("Validation Success: All missions within battery limits.")

    return total_dist_all_missions


def main():
    # Loading NumPy arrays
    distance_matrix = np.load("distance_matrix.npy")
    # predecessors = np.load("predecessors.npy")
    points_lat_long = np.load("points_lat_long.npy")
    asset_indexes = np.load("asset_indexes.npy")
    photo_indexes = np.load("photo_indexes.npy")

    # Loading polygon
    with open("polygon_lon_lat.wkt", "r") as f:
        polygon_wkt = f.read()
    polygon = wkt_loads(polygon_wkt)

    print("Loaded data!")

    print("Re-computing Predecessor Matrix...")
    dist_matrix_fw, predecessors_fw = shortest_path(
        csgraph=distance_matrix, method="FW", directed=False, return_predecessors=True
    )
    predecessors = predecessors_fw
    distance_matrix = dist_matrix_fw

    # Validating indices' length
    n_points = len(points_lat_long)
    n_dist = len(distance_matrix)
    n_pred = len(predecessors)
    n_max_valid = min(n_points, n_dist, n_pred)

    # Validating Asset, Photo Coordinates, Indices
    valid_asset_indices = [
        i for i in range(asset_indexes[0], asset_indexes[1]) if i < n_max_valid
    ]
    asset_coords = points_lat_long[valid_asset_indices]
    valid_photo_indices = [
        i for i in range(photo_indexes[0], photo_indexes[1]) if i < n_max_valid
    ]
    photo_coords = points_lat_long[valid_photo_indices]

    # Calculating VRP (Vehicle Route Plan)
    solution_package = calculate_vrp(distance_matrix, valid_photo_indices)
    if solution_package:
        # Find individual missions from VRP
        all_missions_detailed = create_missions(
            solution_package, predecessors, n_max_valid
        )
        solution = solution_package[0]

        # Validate missions (All waypoints covered, battery usage)
        total_dist_all_missions = validate(
            distance_matrix, all_missions_detailed, valid_photo_indices
        )

        print(f"Total Missions: {len(all_missions_detailed)}")
        print(f"Total Flight Distance: {total_dist_all_missions:.2f} ft")
        print(f"Solver Objective: {solution.ObjectiveValue()} ft (uses sub-matrix)")

        # Pop-up polygon of route
        visual.visualize_polygon(
            polygon, asset_coords, photo_coords, points_lat_long, all_missions_detailed
        )

    else:
        print("Solver failed to find a solution.")

    sys.exit(0)


if __name__ == "__main__":
    main()
