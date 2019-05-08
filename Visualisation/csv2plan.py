import csv
import numpy as np

coord = []
with open('../Path_planner/outputs/waypoints_gps.csv', 'r') as waypointsF:
    reader = csv.reader(waypointsF, delimiter=',')
    for row in reader:
        coord.append([float(row[0]), float(row[1]), float(row[2])])

obst = []
with open('../Path_planner/outputs/obstacles_initial_gps.csv', 'r') as waypointsF:
    reader = csv.reader(waypointsF, delimiter=',')
    for row in reader:
        obst.append([float(row[0]), float(row[1]), float(row[2])])

# TODO : DISPLAY OBSTACLES (CIRCLES)
var1 = """{
    "fileType": "Plan",
    "geoFence": {
        "circles": ["""

for o in obst:
    radius = o[2]
    latitude = o[0]
    longitude = o[1]

    var1 += """
            {
                "circle": {
                    "center": [
                        %f,
                        %f
                    ],
                    "radius": %f
                },
                "inclusion": false,
                "version": 1
            },""" %(latitude, longitude, radius)

# delete last comma
var1 = var1[:-1]

var1 += """
        ],
        "polygons": [
        ],
        "version": 2
    },
    "groundStation": "QGroundControl",
    "mission": {
        "cruiseSpeed": 15,
        "firmwareType": 12,
        "hoverSpeed": 5,
        "items": ["""


for c in coord:
    altitude = c[2]
    latitude = c[0]
    longitude = c[1]

    var1 += """
                {
                    "AMSLAltAboveTerrain": null,
                    "Altitude": %f,
                    "AltitudeMode": 0,
                    "autoContinue": true,
                    "command": 16,
                    "doJumpId": 1,
                    "frame": 3,
                    "params": [
                        0,
                        0,
                        0,
                        null,
                        %f,
                        %f,
                        %f
                    ],
                    "type": "SimpleItem"
                },""" %(altitude, latitude, longitude, altitude)

# delete last comma
var1 = var1[:-1]

# first waypoint
altitude = coord[0][2]
latitude = coord[0][0]
longitude = coord[0][1] 

var1 += """
        ],
        "plannedHomePosition": [
            %f,
            %f,
            %f
        ],
        "vehicleType": 2,
        "version": 2
    },
    "rallyPoints": {
        "points": [
        ],
        "version": 2
    },
    "version": 1
}"""  %(latitude, longitude, altitude)

with open("mission.plan", "w") as text_file:
    text_file.write(var1)