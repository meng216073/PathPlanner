# PATH PLANNING VISUALISATION

from gmplot import gmplot
import csv

# Place map
gmap = gmplot.GoogleMapPlotter(38.14626944444444, -76.42816388888889, 13)

# Fly zone
golden_gate_park_lats, golden_gate_park_lons = zip(*[
    (38.14626944444444, -76.42816388888889),
    (38.151624999999996, -76.42868333333334),
    (38.15188888888889, -76.43146666666667),
    (38.150594444444444, -76.43536111111112),
    (38.14756666666667, -76.43234166666667),
    (38.144666666666666, -76.43294722222223),
    (38.143255555555555, -76.43476666666668),
    (38.14046388888889, -76.43263611111112),
    (38.14071944444444, -76.42601388888889),
    (38.14376111111111, -76.42120555555556),
    (38.14734722222222, -76.42321111111112),
    (38.14613055555556, -76.42665277777779),
    (38.14626944444444, -76.42816388888889)
    ])
gmap.plot(golden_gate_park_lats, golden_gate_park_lons, 'cornflowerblue', edge_width=4)


# Scatter points
top_attraction_lats, top_attraction_lons = zip(*[
    (38.1508847, -76.4303750),
    (38.1496530, -76.4328839),
    (38.1423795, -76.42570107),
    (38.1438963, -76.4226473),
    (38.1457923, -76.4240670),
    (38.1440126, -76.4288839)
    ])
gmap.scatter(top_attraction_lats, top_attraction_lons, '#3B0B39', size=10, marker=False)

coord = []
with open('../Path_planner/outputs/waypoints_gps.csv', 'r') as waypointsF:
    i = 0
    reader = csv.reader(waypointsF, delimiter=',')
    for row in reader:
        #if(i % 3 == 0):
        coord.append((float(row[0]), float(row[1])))
        #i = i + 1

top_attraction_lats, top_attraction_lons = zip(*coord)
gmap.scatter(top_attraction_lats, top_attraction_lons, '#008000	', size=2, marker=False)

open('../Path_planner/outputs/waypoints_gps.csv', 'w').close()

coord = []
with open('../Path_planner/outputs/obstacles_gps.csv', 'r') as obstaclesF:
    i = 0
    reader = csv.reader(obstaclesF, delimiter=',')
    for row in reader:
        #if(i % 3 == 0):
        coord.append((float(row[0]), float(row[1])))
        #i = i + 1

top_attraction_lats, top_attraction_lons = zip(*coord)
gmap.scatter(top_attraction_lats, top_attraction_lons, '#ff0000	', size=4, marker=False)

open('../Path_planner/outputs/obstacles_gps.csv', 'w').close()

# Marker
#hidden_gem_lat, hidden_gem_lon = 37.770776, -122.461689
#map.marker(hidden_gem_lat, hidden_gem_lon, 'cornflowerblue')

# Draw
gmap.draw("my_map.html")