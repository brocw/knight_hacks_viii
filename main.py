from shapely.wkt import loads as wkt_loads
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.sparse.csgraph import shortest_path

import plotly.graph_objects as go
import numpy as np
import sys
import streamlit as st
import folium
from streamlit_folium import st_folium
import visual

# CONSTANTS
MAX_MISSION_DISTANCE = 37725
NUM_DRONES = 250
SOLVER_LIMIT_TIME_SEC = 120

# Displays statistics for arrays
def stat(mat, name):
    """Display statistics for a matrix/array in Streamlit"""
    st.subheader(f"📊 Stats for {name}")
    
    # Create columns for a nice layout
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.metric("Count", len(mat))
        st.metric("Mean", f"{np.mean(mat):.4f}")
    
    with col2:
        st.metric("Median", f"{np.median(mat):.4f}")
        st.metric("Std Dev", f"{np.std(mat):.4f}")
    
    with col3:
        st.metric("Min", f"{np.min(mat):.4f}")
        st.metric("Max", f"{np.max(mat):.4f}")
    
    # Optional: Show the data in an expander to keep it clean
    with st.expander(f"View {name} data"):
        st.write(mat)
    
    st.divider()  # Add a visual separator

# Just a number
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

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        st.write("No solution found!")
        return None

    st.write(f"Solution found! Total distance: {solution.ObjectiveValue()} feet.")

    return solution, manager, routing, solver_idx_to_waypoint_idx


# Number
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
                st.write(
                    f"Error: Path reconstruction found invalid node {prev} from predecessors[{from_node_orig}][{curr}]"
                )
                return None

        if prev == from_node_orig:
            path.append(from_node_orig)
            return path[::-1]

        if prev == curr:
            st.write(f"Path reconstruction failed (self-cycle) at {curr}")
            return None

        if prev in visted_in_segment:
            st.write(
                f"Path reconstruction failed (multi-node cycle) from {from_node_orig} to {to_node_orig}"
            )

        curr = prev

    return None


# Number
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

    st.write(f"Created {len(all_missions_detailed)} missions.")
    return all_missions_detailed


# Number
def validate(distance_matrix, all_missions_detailed, valid_photo_indices):
    visited_nodes = set()
    for path in all_missions_detailed:
        visited_nodes.update(path)

    required_nodes = set(valid_photo_indices)
    missing_nodes = required_nodes - visited_nodes

    if not missing_nodes:
        st.write("Validation Success: All required photo waypoints are covered.")
    else:
        st.write(f"Validation Failed: Missing {len(missing_nodes)} waypoints.")

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
            st.write(f"Mission {i} failed: {mission_dist} > {MAX_MISSION_DISTANCE}")
            max_dist_violation = True

    if not max_dist_violation:
        st.write("Validation Success: All missions within battery limits.")

    return total_dist_all_missions


@st.cache_data
def create_folium_map(points_lat_long, _polygon, photos, all_missions_detailed):
    m = folium.Map(location=[_polygon.centroid.y, _polygon.centroid.x], zoom_start=16)

    # Get polygon coordinates
    coords = list(_polygon.exterior.coords)
    coords_latlon = [(lat, lon) for lon, lat in coords]

    folium.Polygon(
        locations=coords_latlon,
        color="blue",
        fill=True,
        fillColor="blue",
        fillOpacity=0.3,
        weight=2,
    ).add_to(m)

    # Add a circle marker for the depot
    folium.CircleMarker(
        location=[points_lat_long[0][1], points_lat_long[0][0]],
        radius=10,
        tooltip="Depot",
        color="green",
        fill=True,
        fillColor="green",
        fillOpacity=0.8,
        stroke=True,
    ).add_to(m)

    # Add a circle marker for each photo
    photo_group = folium.FeatureGroup(name="Photo Nodes")
    for i, asset_coord in enumerate(photos):
        folium.CircleMarker(
            location=[asset_coord[1], asset_coord[0]],  # lat, lon format
            radius=4,
            color="blue",
            fill=True,
            fillColor="blue",
            fillOpacity=0.6,
            weight=1,
            tooltip=f"Photo {i}",
        ).add_to(photo_group)
    photo_group.add_to(m)

    route_colors = [
        "#E6194B",
        "#3CB44B",
        "#FFE119",
        "#4363D8",
        "#F58231",
        "#911EB4",
        "#46F0F0",
        "#F032E6",
        "#BCF60C",
        "#FABEBE",
        "#008080",
        "#E6BEFF",
    ]

    route_group = folium.FeatureGroup(name="Mission Routes")
    for i, path in enumerate(all_missions_detailed):
        if not path:
            continue

        path_coords_lon_lat = points_lat_long[path]
        path_coords_lat_lon = [(coord[1], coord[0]) for coord in path_coords_lon_lat]

        folium.PolyLine(
            locations=path_coords_lat_lon,
            tooltip=f"Mission {i+1}",
            color=route_colors[i % len(route_colors)],
            weight=5,
            opacity=0.7,
        ).add_to(route_group)
    route_group.add_to(m)

    folium.LayerControl().add_to(m)
    return m


def main():
    # Loading NumPy arrays
    distance_matrix = np.load("distance_matrix.npy")
    # predecessors = np.load("predecessors.npy")
    points_lat_long = np.load("points_lat_long.npy")
    asset_indexes = np.load("asset_indexes.npy")
    photo_indexes = np.load("photo_indexes.npy")

    # Inject CSS so all header tags (h1..h6) use a dark earthy green
    header_style = """
    <style>
      h1, h2, h3, h4, h5, h6 {
        color: #355E3B !important;
      }
    </style>
    """
    st.markdown(header_style, unsafe_allow_html=True)

    # Centered title using markdown with custom CSS
    st.markdown(
        "<h1 style='text-align: center;'>Optimizing Drone Flights</h1>",
        unsafe_allow_html=True,
    )
    st.markdown("<h2 style='text-align: center;'>This app helps optimize drone flights within a constrained flight zone</h2>", unsafe_allow_html=True)

    # Show statistics in an expandable section
    with st.expander("📈 View Data Statistics", expanded=False):
        stat(distance_matrix, "Distance Matrix")
        # stat(predecessors, "Predecessors")
        stat(points_lat_long, "Points (Lat/Long)")
        stat(asset_indexes, "Asset Indexes")
        stat(photo_indexes, "Photo Indexes")

    st.divider()

    # Loading polygon (keep spinner active while we read and parse WKT so
    # both `polygon_wkt` and `polygon` are available during the spinner)
    with open("polygon_lon_lat.wkt", "r") as f:
        polygon_wkt = f.read()
    polygon = wkt_loads(polygon_wkt)

    with st.spinner("Re-computing Predecessor Matrix...", show_time=True):
        dist_matrix_fw, predecessors_fw = shortest_path(
            csgraph=distance_matrix,
            method="FW",
            directed=False,
            return_predecessors=True,
        )

    st.markdown("<h3>Re-computing done!</h3>", unsafe_allow_html=True)

    predecessors = predecessors_fw
    distance_matrix = dist_matrix_fw

    # Validating indices' length
    n_points = len(points_lat_long)
    n_dist = len(distance_matrix)
    n_pred = len(predecessors)
    n_max_valid = min(n_points, n_dist, n_pred)

    # Validating Asset, Photo Coordinates, Indices
    valid_asset_indices = [
        i for i in range(asset_indexes[0], asset_indexes[1]) if i < len(points_lat_long)
    ]
    asset_coords = points_lat_long[valid_asset_indices]

    valid_photo_indices = [
        i for i in range(photo_indexes[0], photo_indexes[1]) if i < len(points_lat_long)
    ]
    photo_coords = points_lat_long[valid_photo_indices]

    # Calculating VRP (Vehicle Route Plan)

    with st.spinner("Calculating VRP (Vehicle Route Plan)", show_time=True):
        solution_package = calculate_vrp(distance_matrix, valid_photo_indices)

    st.markdown("<h3>Calculating VRP (Vehicle Route Plan) done!</h3>", unsafe_allow_html=True)

    if solution_package:
        # Find individual missions from VRP
        with st.spinner("Find individual missions from VRP", show_time=True):
            st.session_state.all_missions_detailed = create_missions(
                solution_package, predecessors, n_max_valid
            )
        
        st.markdown("<h3>Individual mission created!</h3>", unsafe_allow_html=True)

        solution = solution_package[0]

        # Validate missions (All waypoints covered, battery usage)
        with st.spinner(
            "Validate missions (All waypoints covered, battery usage)", show_time=True
        ):
            total_dist_all_missions = validate(
                distance_matrix,
                st.session_state.all_missions_detailed,
                valid_photo_indices,
            )

        st.markdown("<h3>Validating missions is done!</h3>", unsafe_allow_html=True)

        st.markdown(f"<h4>Total Missions: {len(st.session_state.all_missions_detailed)}</h4>", unsafe_allow_html=True)
        st.markdown(f"<h4>Total Flight Distance: {total_dist_all_missions:.2f} ft</h4>", unsafe_allow_html=True)
        st.write(f"<h4>Solver Objective: {solution.ObjectiveValue()} ft (uses sub-matrix)</h4>", unsafe_allow_html=True)

        st.header("Full Map")

        # Loading map
        folium_map = create_folium_map(
            points_lat_long,
            polygon,
            photo_coords,
            st.session_state.all_missions_detailed,
        )

        # Display the map first
        st_folium(
            folium_map, width=700, height=700, key="drone_map", returned_objects=[]
        )

        # Pop-up polygon of route
        # visual.visualize_polygon(
        #     polygon,
        #     asset_coords,
        #     photo_coords,
        #     points_lat_long,
        #     st.session_state.all_missions_detailed,
        # )
    else:
        st.write("Solver failed to find a solution.")


if __name__ == "__main__":
    main()
