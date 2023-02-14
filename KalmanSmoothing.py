from CustomKalman import TwoDKalman
from numpy import genfromtxt
import numpy as np
from math import cos, sin, pi
def deg2rad(deg):
    '''
    converts degrees to radians, pretty simple
    '''
    return deg*(pi/180.0)

def rad2deg(rad):
    '''
    converts radians to degrees, pretty simple
    '''
    return rad*180.0/pi

def latlonToXY(lat, lon, aspectRatio):
    """
    This function converts lattitude and longitude to x and y values using simple eqirectangular projection. X and y are in meters
    """
    r = 6371000
    lat = deg2rad(lat)
    lon = deg2rad(lon)
    y = r*lat
    x = r*lon*aspectRatio
    return x, y

def logDataUpdate(data, name):
    with open(name, 'w') as f:
        f.write('')
    with open(name, 'a', newline = '') as f:
        i = 0
        while i < len(data):
            f.write(data[i]+'\n')
            i += 1

def XYtolatlon(x, y, aspectRatio):
    """
    This function converts x,y back to lattitude and longitude using simple equirectangular projection. X and y need to be in meters.
    """
    r = 6371000
    lat = rad2deg(y/r)
    lon = rad2deg(x/(r*aspectRatio))
    return lat, lon

name = "test5.csv"
velocity = 0.3 #m/s guess
dt = 1.2 #sec guess
waypoints = genfromtxt(name, delimiter=',')
refLat = deg2rad(waypoints[0,0])
aspectRatio = cos(refLat)
xwaypoints = []
ywaypoints = []
heading = []
filteredX = []
filteredY = []
filteredLat = []
filteredLon = []
data = []
i = 0
x_std = 10 #m
y_std = 10
vx_std = 0.05 #radians
vy_std = 0.05
while i < len(waypoints):
    
    [xpoints, ypoints] = latlonToXY(waypoints[i,0],waypoints[i,1],aspectRatio)
    xwaypoints.append(xpoints)
    ywaypoints.append(ypoints)
    heading.append(waypoints[i,2])
    i += 1

initial_state = np.array([[xwaypoints[0]],[ywaypoints[0]],[sin(heading[0])*velocity],[cos(heading[0])*velocity]])
Filter = TwoDKalman(initial_state,dt,x_std,y_std, vx_std, vy_std)

for i in range(len(waypoints)):
    current_state = np.array([[xwaypoints[i]],[ywaypoints[i]],[sin(heading[i])*velocity],[cos(heading[i])*velocity]])
    Filter.predict()
    [x,y] = Filter.update(current_state)
    if i==3:
        print(x[0])
        print(y[0])
    filteredX.append(x[0])
    filteredY.append(y[0])
for i in range(len(filteredX)):
    [Flat,Flon] = XYtolatlon(filteredX[i],filteredY[i], aspectRatio)
    filteredLat.append(Flat)
    filteredLon.append(Flon)
for i in range(len(filteredLon)):
    data.append(str(filteredLat[i]) + "," + str(filteredLon[i]))
logDataUpdate(data, "filtered"+str(name))