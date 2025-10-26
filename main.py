from shapely.wkt import loads as wkt_loads
import plotly.graph_objects as go
import streamlit as st
import numpy as np
import sys
import pandas as pd
import time
import folium
from streamlit_folium import st_folium


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
    fig.show()

@st.cache_data
def create_folium_map(points_lat_long, _polygon, assets):
    m = folium.Map(location=[_polygon.centroid.y, _polygon.centroid.x], zoom_start=16)

    # Get polygon coordinates
    coords = list(_polygon.exterior.coords)
    coords_latlon = [(lat, lon) for lon, lat in coords]

    folium.Polygon(
        locations=coords_latlon,
        color='blue',
        fill=True,
        fillColor='blue',
        fillOpacity=0.3,
        weight=2
    ).add_to(m)

    # Add a circle marker for the depot
    depot_marker = folium.CircleMarker(
        location=[points_lat_long[0][1], points_lat_long[0][0]],
        radius=20,
        tooltip='Depot',
        color='green',
        fill=True,
        fillOpacity=0.5,
        stroke=True,
        opacity=20
    ).add_to(m)

    # Add a circle marker for each asset
    for i, asset_coord in enumerate(assets):
        folium.CircleMarker(
            location=[asset_coord[1], asset_coord[0]],  # lat, lon format
            radius=6,
            color='red',
            fill=True,
            fillColor='red',
            fillOpacity=0.6,
            weight=2
        ).add_to(m)

    return m

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

    # Loading map
    folium_map = create_folium_map(points_lat_long, polygon, asset_coords)

    print("Loaded data!")

    # stat(distance_matrix, "Distance Matrix")
    # stat(predecessors, "Predecessors")
    # stat(points_lat_long, "Points (Lat/Long)")
    # stat(asset_indexes, "Asset Indexes")
    # stat(photo_indexes, "Photo Indexes")

    st.title("Drone Flight Optimization!")

    # The key parameter ensures state is maintained across reruns
    # returned_objects=[] prevents the widget from triggering reruns on interaction
    st_folium(folium_map, width=700, height=700, key="drone_map", returned_objects=[])






















    # Visualizing Polygon
    # st.map(visualize_polygon(polygon, asset_coords, photo_coords, points_lat_long))

    # df = pd.DataFrame(
    #     np.random.randn(1000, 2) / [50, 50] + [37.76, -122.4],
    #     columns = ['lat', 'lon']
    # )

    # x = st.slider('x')
    # st.write(x, 'squared is', x * x)

    # st.map(df)

    # if st.checkbox("Show dataframe"):
    #     chart_data = pd.DataFrame(
    #         np.random.randn(20, 3),
    #         columns = ['a', 'b', 'c']
    #     )
    #     st.write(chart_data)

    # dataframe = pd.DataFrame({
    #     'first column': [1, 2, 3, 4],
    #     'second column': [10, 20, 30, 40]
    # })

    # option = st.selectbox(
    #     'Which number do you like best?',
    #     dataframe['first column']
    # )

    # st.write('You selected: ', option)

    # add_selectbox = st.sidebar.selectbox(
    #     'How would you like to be contacted?',
    #     ('Email', 'Home phone', 'Mobile phone')
    # )

    # add_slider = st.sidebar.slider(
    #     'Select a range of values',
    #     0.0, 100.0, (25.0, 75.0)
    # )

    # left_column, right_column = st.columns(2)

    # # you can use a column just like st.sidebar
    # left_column.button('Press me!')

    # # or even better, call streamlit functions inside a "with" block
    # with right_column:
    #     chosen = st.radio(
    #         'Sorting hat',
    #         ("Gryffindor", "Ravenclaw", "Hufflepuff", "Slytherin")
    #     )
    #     st.write(f"You are in {chosen} house!")

    # st.write("Starting a long computation...")

    # # add a placeholder
    # latest_iteration = st.empty()
    # bar = st.progress(0)

    # for i in range(100):
    #     # update the progress bar with each iteration
    #     latest_iteration.text(f"Iteration {i + 1}")
    #     bar.progress(i + 1)
    #     time.sleep(0.1)

    # st.write("...and now we're done!")

    

    quit()


if __name__ == "__main__":
    main()
