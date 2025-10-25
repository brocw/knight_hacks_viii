from shapely.wkt import loads as wkt_loads
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import plotly.graph_objects as go
import numpy as np
import sys

MAX_MISSION_DISTANCE = 37725

NUM_DRONES = 250

SOLVER_LIMIT_TIME_SEC = 120


def stat(mat, name):
    print(f"Stats for {name}")
    print(f"Count: {len(mat)}")
    print(f"Mean: {np.mean(mat)}")
    print(f"Median: {np.median(mat)}")
    print(f"Std Dev: {np.std(mat)}")
    print(f"Min: {np.min(mat)}")
    print(f"Max: {np.max(mat)}")
    print(f"{mat}\n")


def visualize_polygon(polygon, assets, photos, points_lat_long):
    poly_lons, poly_lats = polygon.exterior.xy
    center_lon = polygon.centroid.x
    center_lat = polygon.centroid.y

    fig = go.Figure()
    fig.add_trace(
        go.Scattermapbox(
            mode="lines",
            lon=list(poly_lons),
            lat=list(poly_lats),
            fill="toself",
            fillcolor="rgba(173, 216, 230, 0.3)",
            line=dict(color="blue", width=2),
            name="Flight Zone",
        )
    )

    depot_lon_lat = points_lat_long[0]
    depot_lon = depot_lon_lat[0]
    depot_lat = depot_lon_lat[1]

    print(f"Depot at [{depot_lon}, {depot_lat}]")

    layout_layers = []

    depot_layer = {
        "sourcetype": "geojson",
        "source": {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [depot_lon, depot_lat]},
        },
        "type": "circle",
        "color": "green",
    }

    asset_layer = {
        "sourcetype": "geojson",
        "source": {
            "type": "Feature",
            "geometry": {"type": "MultiPoint", "coordinates": assets.tolist()},
        },
        "type": "circle",
        "color": "red",
        "opacity": 0.25,
    }

    photo_layer = {
        "sourcetype": "geojson",
        "source": {
            "type": "Feature",
            "geometry": {"type": "MultiPoint", "coordinates": photos.tolist()},
        },
        "type": "circle",
        "color": "blue",
        "opacity": 0.1,
    }

    layout_layers.append(asset_layer)
    layout_layers.append(photo_layer)
    layout_layers.append(depot_layer)

    fig.update_layout(
        title_text="Drone Flight Polygon",
        mapbox_style="carto-positron",
        mapbox_center_lon=center_lon,
        mapbox_center_lat=center_lat,
        mapbox_zoom=16,
        margin={"r": 0, "t": 40, "l": 0, "b": 0},
        mapbox_layers=layout_layers,
    )
    # fig.show()


def step_two(dist_matrix, photo_coords):
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
    distance_dimension = routing.GetDimensionOrDie("Distance")

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


def main():
    # Loading NumPy arrays
    distance_matrix = np.load("distance_matrix.npy")
    predecessors = np.load("predecessors.npy")
    points_lat_long = np.load("points_lat_long.npy")
    asset_indexes = np.load("asset_indexes.npy")
    photo_indexes = np.load("photo_indexes.npy")

    # Loading polygon
    with open("polygon_lon_lat.wkt", "r") as f:
        polygon_wkt = f.read()
    polygon = wkt_loads(polygon_wkt)

    valid_asset_indices = [
        i for i in range(asset_indexes[0], asset_indexes[1]) if i < len(points_lat_long)
    ]
    asset_coords = points_lat_long[valid_asset_indices]

    valid_photo_indices = [
        i for i in range(photo_indexes[0], photo_indexes[1]) if i < len(points_lat_long)
    ]
    photo_coords = points_lat_long[valid_photo_indices]

    print("Loaded data!")

    stat(distance_matrix, "Distance Matrix")
    stat(predecessors, "Predecessors")
    stat(points_lat_long, "Points (Lat/Long)")
    stat(asset_indexes, "Asset Indexes")
    stat(photo_indexes, "Photo Indexes")

    # Visualizing Polygon
    visualize_polygon(polygon, asset_coords, photo_coords, points_lat_long)

    solution_package = step_two(distance_matrix, valid_photo_indices)

    if solution_package:
        solution, manager, routing, solver_idx_to_waypoint_idx = solution_package
        print("Solution analysis here :)")
    else:
        print("Solver failed to find a solution.")

    sys.exit()


if __name__ == "__main__":
    main()
