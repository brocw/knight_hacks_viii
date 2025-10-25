import numpy as np


def main():
    print("Hello World!")
    distance_matrix = np.load("distance_matrix.npy")
    predecessors = np.load("predecessors.npy")
    points_lat_long = np.load("points_lat_long.npy")
    asset_indexes = np.load("asset_indexes.npy")
    photo_indexes = np.load("photo_indexes.npy")

    print("Loaded data!")

    # TODO: VISUALIZE (visual.py?)

    quit()


if __name__ == "__main__":
    main()
