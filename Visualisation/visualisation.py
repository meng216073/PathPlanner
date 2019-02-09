# PATH PLANNING VISUALISATION

import csv
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from matplotlib.font_manager import FontProperties

fontP = FontProperties()
fontP.set_size('small')

# Waypoints + execution time
x, y, theta = [], [], []
goals = []
nbTotalWaypoints = 0
totalExecTime = 0

with open('../Path_planner/outputs/waypoints.csv', 'r') as waypointsF:
    nbWaypoints = int(waypointsF.readline())
    #execTime = float(waypointsF.readline())
    reader = csv.reader(waypointsF, delimiter=',')

    nbTotalWaypoints = nbTotalWaypoints + nbWaypoints
    i = 0
    for row in reader:
           
        if(i == 0):
            execTime = float(row[0])
            totalExecTime = totalExecTime + execTime
            i = i + 1
        elif(i <= nbWaypoints + 1):

            if(i == 1):
                goals.append(row)

            x.append(float(row[0]))
            y.append(float(row[1]))
            theta.append(float(row[2]))
            i = i + 1
        else:
            nbWaypoints = int(row[0])
            nbTotalWaypoints = nbTotalWaypoints + nbWaypoints
            i = 0

del goals[0]

for g in goals:
    plt.scatter(float(g[0]), float(g[1]), s=100, color='black', marker='o')

lengthX = len(x) - 1
lengthY = len(y) - 1
lengthT = len(theta) - 1

plt.scatter(x[0], y[0], s=100, color='green', marker='o', label='Start')
plt.scatter(x[ lengthX ], y[ lengthY ], s=100, color='red', marker='o', label='Goal')

x.pop(lengthX)
x.pop(0)
y.pop(lengthY)
y.pop(0)

for i in range(1, len(x) - 1, 1):
    plt.scatter(x[i], y[i], s=50, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90))

lengthX = len(x) - 1


# Obstacles
vertices = []
patchesObst = []
patchesObstExtended = []
j = 1


with open('../Path_planner/outputs/obstacles.csv', 'r') as obstaclesF:
    reader = csv.reader(obstaclesF, delimiter=',')
    for row in reader:
        lenRow = range(len(row))
        for i in lenRow:
            if (i % 2 == 0):
                vertices.append((row[i], row[i+1]))

        if(j % 2 == 0):
            polygon = Polygon(vertices, True)
            patchesObstExtended.append(polygon)
        else:
            polygon = Polygon(vertices, True)
            patchesObst.append(polygon)

        del vertices[:]
        j = j + 1


ax = plt.subplot()
p = PatchCollection(patchesObst)
ax.add_collection(p)

color_edge = [1, 0, 0]
color_interior = [1, 1, 1]
p = PatchCollection(patchesObstExtended, alpha=0.5)
p.set_color(color_interior)
p.set_edgecolor(color_edge)
ax.add_collection(p)


################### Fly zone UAS => TBD
pointsX = [14.323, -42.969, -329.12, -729.854, -415.058, -472.247, -672.665, -443.704, 243.285, 729.854, 529.539, 186.096]
pointsY = [15.5574, 604.06, 635.063, 511.162, 170.462, -170.35, -325.255, -635.063, -604.06, -263.248, 139.459, 15.5574]
vertices = []
patchesFlyZone = []

for i in range(len(pointsX)):
    vertices.append((pointsX[i], pointsY[i]))
    polygon = Polygon(vertices, True)
    patchesFlyZone.append(polygon)
    del vertices[:]

p = PatchCollection(patchesFlyZone)
ax.add_collection(p)
######################################


# Map limits + resolution
xrange = []
yrange = []

with open('../Path_planner/outputs/map.csv', 'r') as mapF:
    safetyDist = float(mapF.readline())
    resolution = float(mapF.readline())
    reader = csv.reader(mapF, delimiter=',')
    for row in reader:
        xrange.append(float(row[0]))
        xrange.append(float(row[1]))
        yrange.append(float(row[2]))
        yrange.append(float(row[3]))

plt.scatter(x[ lengthX ], y[ lengthX ], s=50, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90), label='Nb waypoints = %d\nResolution = %.2fm\nSafety distance = %.2fm' %(nbTotalWaypoints, resolution, safetyDist))
leg = plt.legend(loc='upper right', scatterpoints=1, prop=fontP, bbox_to_anchor=(1.3, 1.005))
leg.draggable()

ax.set_title('Path planning (Execution time : %.4f sec)' %totalExecTime)
ax.set_xlim(*xrange)
ax.set_ylim(*yrange)
ax.set_aspect(1)

plt.show()

open('../Path_planner/outputs/waypoints.csv', 'w').close()
open('../Path_planner/outputs/obstacles.csv', 'w').close()
open('../Path_planner/outputs/map.csv', 'w').close()
