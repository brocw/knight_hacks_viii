from shapely.wkt import loads as wkt_loads

with open("polygon_lon_lat.wkt", "r") as f:
    polygon_wkt = f.read()
polygon = wkt_loads(polygon_wkt)

print(polygon)
