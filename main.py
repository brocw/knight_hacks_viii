# from shapely.wkt import loads as wkt_loads

# import plotly.graph_objects as go
import streamlit as st
import numpy as np
import sys
import pandas as pd
import time


def stat(mat, name):
    print(f"Stats for {name}")
    print(f"Count: {len(mat)}")
    print(f"Mean: {np.mean(mat)}")
    print(f"Median: {np.median(mat)}")
    print(f"Std Dev: {np.std(mat)}")
    print(f"Min: {np.min(mat)}")
    print(f"Max: {np.max(mat)}")
    print(f"{mat}\n")


def visualize_polygon(polygon, points_lat_long):
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

    # fig.add_trace(
    #     go.Scattermapbox(
    #         mode="markers",
    #         lon=[depot_lon],
    #         lat=[depot_lat],
    #         marker=dict(size=20, color="green", symbol="star"),
    #         name="Depot",
    #     )
    # )
    fig.update_layout(
        title_text="Drone Flight Polygon",
        mapbox_style="open-street-map",
        mapbox_center_lon=center_lon,
        mapbox_center_lat=center_lat,
        mapbox_zoom=14,
        margin={"r": 0, "t": 40, "l": 0, "b": 0},
    )
    fig.show()


def main():
    # # Loading NumPy arrays
    # distance_matrix = np.load("distance_matrix.npy")
    # predecessors = np.load("predecessors.npy")
    # points_lat_long = np.load("points_lat_long.npy")
    # asset_indexes = np.load("asset_indexes.npy")
    # photo_indexes = np.load("photo_indexes.npy")

    # # Loading polygon
    # with open("polygon_lon_lat.wkt", "r") as f:
    #     polygon_wkt = f.read()
    # polygon = wkt_loads(polygon_wkt)

    # print("Loaded data!")

    # stat(distance_matrix, "Distance Matrix")
    # stat(predecessors, "Predecessors")
    # stat(points_lat_long, "Points (Lat/Long)")
    # stat(asset_indexes, "Asset Indexes")
    # stat(photo_indexes, "Photo Indexes")

    # # Visualizing Polygon
    # visualize_polygon(polygon, points_lat_long)

    df = pd.DataFrame(
        np.random.randn(1000, 2) / [50, 50] + [37.76, -122.4],
        columns = ['lat', 'lon']
    )

    x = st.slider('x')
    st.write(x, 'squared is', x * x)

    st.map(df)

    if st.checkbox("Show dataframe"):
        chart_data = pd.DataFrame(
            np.random.randn(20, 3),
            columns = ['a', 'b', 'c']
        )
        st.write(chart_data)

    dataframe = pd.DataFrame({
        'first column': [1, 2, 3, 4],
        'second column': [10, 20, 30, 40]
    })

    option = st.selectbox(
        'Which number do you like best?',
        dataframe['first column']
    )

    st.write('You selected: ', option)

    add_selectbox = st.sidebar.selectbox(
        'How would you like to be contacted?',
        ('Email', 'Home phone', 'Mobile phone')
    )

    add_slider = st.sidebar.slider(
        'Select a range of values',
        0.0, 100.0, (25.0, 75.0)
    )

    left_column, right_column = st.columns(2)

    # you can use a column just like st.sidebar
    left_column.button('Press me!')

    # or even better, call streamlit functions inside a "with" block
    with right_column:
        chosen = st.radio(
            'Sorting hat',
            ("Gryffindor", "Ravenclaw", "Hufflepuff", "Slytherin")
        )
        st.write(f"You are in {chosen} house!")

    st.write("Starting a long computation...")

    # add a placeholder
    latest_iteration = st.empty()
    bar = st.progress(0)

    for i in range(100):
        # update the progress bar with each iteration
        latest_iteration.text(f"Iteration {i + 1}")
        bar.progress(i + 1)
        time.sleep(0.1)

    st.write("...and now we're done!")

    

    quit()


if __name__ == "__main__":
    main()
