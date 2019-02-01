# PATH PLANNING VISUALISATION

from mpl_toolkits.mplot3d import Axes3D
import csv
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from matplotlib.font_manager import FontProperties

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

fontP = FontProperties()
fontP.set_size('small')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Waypoints + execution time
x, y, theta, z = [], [], [], []
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
            z.append(float(row[3]))
            i = i + 1
        else:
            nbWaypoints = int(row[0])
            nbTotalWaypoints = nbTotalWaypoints + nbWaypoints
            i = 0

del goals[0]

for g in goals:
    ax.scatter(float(g[0]), float(g[1]), float(g[3]), s=100, color='black', marker='o')

lengthX = len(x) - 1
lengthY = len(y) - 1
lengthT = len(theta) - 1
lengthZ = len(z) - 1

ax.scatter(x[0], y[0], z[0], s=100, color='green', marker='o', label='Start')
ax.scatter(x[ lengthX ], y[ lengthY ], z[ lengthZ ], s=100, color='red', marker='o', label='Goal')

x.pop(lengthX)
x.pop(0)
y.pop(lengthY)
y.pop(0)
#z.pop(lengthZ)
#z.pop(0)

for i in range(1, len(x) - 1, 1):
    ax.scatter(x[i], y[i], z[i], s=5, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90))

lengthX = len(x) - 1


'''
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
'''



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

ax.scatter(x[ lengthX ], y[ lengthX ], z[ lengthZ ], s=50, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90), label='Nb waypoints = %d\nResolution = %.2fm\nSafety distance = %.2fm' %(nbTotalWaypoints, resolution, safetyDist))
leg = ax.legend(loc='upper right', scatterpoints=1, prop=fontP, bbox_to_anchor=(1.3, 1.005))
leg.draggable()

ax.set_title('Path planning (Execution time : %.4f sec)' %totalExecTime)
ax.set_xlim(*xrange)
ax.set_ylim(*yrange)
ax.set_aspect(1)


# Obstacles
Xc,Yc,Zc = data_for_cylinder_along_z(-274, 480, 30.48, 122)
ax.plot_surface(Xc, Yc, Zc, color='red', alpha=1)

plt.show()

'''
open('../Path_planner/outputs/waypoints.csv', 'w').close()
open('../Path_planner/outputs/obstacles.csv', 'w').close()
open('../Path_planner/outputs/map.csv', 'w').close()
'''

'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x =[1,2,3,4,5,6,7,8,9,10]
y =[5,6,2,3,13,4,1,2,4,8]
z =[2,3,3,3,5,7,9,11,9,700]

ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_xlim(*xrange)
ax.set_ylim(*yrange)

# TBC !!
zrange = 750
ax.set_zlim(zrange)


plt.show()
'''