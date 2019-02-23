import plotly as py
import plotly.graph_objs as go
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

trace1 = go.Scatter3d(
    x=x,
    y=y,
    z=z
)

data = [trace1]

# Obstacles
with open('../Path_planner/outputs/obstacles_cartesian.csv', 'r') as obstaclesF:
    reader = csv.reader(obstaclesF, delimiter=',')
    for row in reader:
        Xc,Yc,Zc = data_for_cylinder_along_z(float(row[0]), float(row[1]), float(row[3]), float(row[2]))
        trace2 = go.Surface(
        x=Xc,
        y=Yc,
        z=Zc
        )
        data.append(trace2)

layout = go.Layout(
    margin=dict(
        l=0,
        r=0,
        b=0,
        t=0
    )
)

fig = go.Figure(data=data, layout=layout)
py.offline.plot(fig, filename='simple-3d-scatter.html')