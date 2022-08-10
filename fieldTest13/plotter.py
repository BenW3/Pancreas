import matplotlib.pyplot as plt
from numpy import genfromtxt
import methods


error = []
time = []
Pid = []
xtraversed = []
ytraversed = []
xdesired = []
ydesired = []

errordata = genfromtxt(r'G:\test2\erroroutput.csv', delimiter=',')
PIDdata = genfromtxt(r'G:\test2\PIDoutput.csv', delimiter=',')
pathdata = genfromtxt(r'G:\test2\traversedPath.csv', delimiter=',')
desiredpathdata = genfromtxt(r'G:\test2\street3.csv', delimiter=',')

refLat = methods.deg2rad(desiredpathdata[0,0])
aspectRatio = methods.cos(refLat)

i = 0
while i < len(pathdata):
    
    [xpoints, ypoints] = methods.latlonToXY(pathdata[i,0],pathdata[i,1],aspectRatio)
    xtraversed.append(xpoints)
    ytraversed.append(ypoints)
    i += 1

i = 0
while i < len(desiredpathdata):
    
    [xpoints, ypoints] = methods.latlonToXY(desiredpathdata[i,0],desiredpathdata[i,1],aspectRatio)
    xdesired.append(xpoints)
    ydesired.append(ypoints)
    i += 1

i = 0
while i < len(errordata):
    
    time.append(errordata[i,0])
    error.append(errordata[i,1])
    Pid.append(PIDdata[i,1])
    i += 1

fig, ax = plt.subplots()
ax.set_aspect(1)
plt.plot(xtraversed,ytraversed, label="Robot Path")
plt.plot(xdesired,ydesired, '-', label = "Waypoint Path")
plt.ylabel('meters (+north, -south)')
plt.xlabel('meters (+east, -west)', labelpad=20)
leg = plt.legend(loc = 'upper left', bbox_to_anchor=(-0.1, -0.2), shadow=True)
leg.get_frame().set_boxstyle('Square')
leg.get_frame().set_edgecolor('k')
leg.get_frame().set_linewidth(1.0)
plt.savefig('fieldTestRobotPath4.png', bbox_inches="tight", dpi = 250)

plt.figure(2)
plt.plot(time, error, linewidth = 1)
plt.savefig('fieldTestError4', bbox_inches="tight")

plt.figure(3)
plt.plot(time, Pid, linewidth = 1)
plt.savefig('fieldTestPid4', bbox_inches="tight")
# plt.figure(3)
# plt.plot(time,derErrPlot, label = "Derivative Error")
# plt.plot(time,intErrPlot, label = "Integral Error")
# plt.plot(time,proErrPlot, label = "Proportional Error")
# plt.plot(time,errTotPlot,'-' , label = "Total Error Feedback")
# leg = plt.legend(loc = 'upper left', bbox_to_anchor=(-0.1, -0.07), shadow=True)
# leg.get_frame().set_boxstyle('Square')
# leg.get_frame().set_edgecolor('k')
# leg.get_frame().set_linewidth(1.0)
# plt.savefig('errorbreakdown.png', bbox_inches="tight")