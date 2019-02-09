# PATH PLANNING VISUALISATION

from mayavi import mlab
import csv
import numpy as np
import math

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

fontP = FontProperties()
fontP.set_size('small')

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')


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

##
for g in goals:
    points3d(float(g[0]), float(g[1]), float(g[3]), s=100)

    #ax.scatter(float(g[0]), float(g[1]), float(g[3]), s=100, color='black', marker='o')

lengthX = len(x) - 1
lengthY = len(y) - 1
lengthT = len(theta) - 1
lengthZ = len(z) - 1

##
ax.scatter(x[0], y[0], z[0], s=100, color='green', marker='o', label='Start')
ax.scatter(x[ lengthX ], y[ lengthY ], z[ lengthZ ], s=100, color='red', marker='o', label='Goal')

x.pop(lengthX)
x.pop(0)
y.pop(lengthY)
y.pop(0)
#z.pop(lengthZ)
#z.pop(0)

##
for i in range(1, len(x) - 1, 1):
    ax.scatter(x[i], y[i], z[i], s=5, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90))

lengthX = len(x) - 1




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

##
ax.scatter(x[ lengthX ], y[ lengthX ], z[ lengthZ ], s=50, color='blue', marker=(2, 1, theta[i] * 180/math.pi + 90), label='Nb waypoints = %d\nResolution = %.2fm\nSafety distance = %.2fm' %(nbTotalWaypoints, resolution, safetyDist))
leg = ax.legend(loc='upper right', scatterpoints=1, prop=fontP, bbox_to_anchor=(1.3, 1.005))
leg.draggable()

ax.set_title('Path planning (Execution time : %.4f sec)' %totalExecTime)
ax.set_xlim(*xrange)
ax.set_ylim(*yrange)
ax.set_aspect(1)


# Obstacles
with open('../Path_planner/outputs/obstacles_cartesian.csv', 'r') as obstaclesF:
    reader = csv.reader(obstaclesF, delimiter=',')
    for row in reader:
        Xc,Yc,Zc = data_for_cylinder_along_z(float(row[0]), float(row[1]), float(row[3]), float(row[2]))
        ax.plot_surface(Xc, Yc, Zc, color='red', alpha=1)

plt.show()