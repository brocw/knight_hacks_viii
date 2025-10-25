import numpy as np


def stat(mat, name):
    print(f"Stats for {name}")
    print(f"Count: {len(mat)}")
    print(f"Mean: {np.mean(mat)}")
    print(f"Median: {np.median(mat)}")
    print(f"Std Dev: {np.std(mat)}")
    print(f"Min: {np.min(mat)}")
    print(f"Max: {np.max(mat)}")
    print(f"{mat}\n")


def main():
    print("Hello World!")
    distance_matrix = np.load("distance_matrix.npy")
    predecessors = np.load("predecessors.npy")
    points_lat_long = np.load("points_lat_long.npy")
    asset_indexes = np.load("asset_indexes.npy")
    photo_indexes = np.load("photo_indexes.npy")

    print("Loaded data!")

    stat(distance_matrix, "Distance Matrix")
    stat(predecessors, "Predecessors")
    stat(points_lat_long, "Points (Lat/Long)")
    stat(asset_indexes, "Asset Indexes")
    stat(photo_indexes, "Photo Indexes")

    # TODO: VISUALIZE (visual.py?)

    quit()


if __name__ == "__main__":
    main()
