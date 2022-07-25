# distance calculator
# https://www.hindawi.com/journals/ape/2014/507142/

from math import cos, sin, pi, acos, atan
from time import sleep
import serial
import pynmea2
from time import perf_counter
from numpy import genfromtxt, sign, sort
import csv
import serial.tools.list_ports

# steeringArduino = serial.Serial(port = 'COM13', baudrate=9600, timeout=5)
robotLength = 4 #m
robotWidth = 10 #m
angleSignal = 0  # Radians
velocitySignal = 0  # Microseconds
mode = 1  # Steering mode
setpoint = 0  # for synchronous steering
Speed = 1700
speedset = -1

def getSerialPorts():
    """
    This function finds the pancreas' connected devices and stores their serial port names.
    """
    global steeringPort
    global sensorPort
    global gpsPort
    global radioPort
    radioPort = ""
    sensorPort = ""
    gpsPort = ""
    steeringPort = ""
    try:
        radioPort = list(*serial.tools.list_ports.grep('FT232EX'))[0]
    except:
        print("Radio not connected.")
    try:
        gpsPort = list(*serial.tools.list_ports.grep('Controller'))[0]
    except:
        print("GPS not connected.")
    try:
        sensorPort = list(*serial.tools.list_ports.grep('Leonardo'))[0]
    except:
        print("Sensor arduino not connected.")
    try:
        steeringPort = list(*serial.tools.list_ports.grep('USB Serial'))[0]
    except:
        print("Steering arduino not connected.")

def initArduinos():
    """
    This function initializes the two arduinos used in the rest of the program. 
    """
    global steeringArduino
    global sensorArduino
    val = ""

    if steeringPort != "":
        try:
            steeringArduino = serial.Serial(port = steeringPort, baudrate = 115200, timeout= 0.1)
            val +='steering connected'
        except Exception as e:
            val += e
    if sensorPort != "":
        try:
            sensorArduino = serial.Serial(port = sensorPort, baudrate = 115200, timeout = 0.1)
            val += ', sensors connected'
        except Exception as e:
            val += e
    return val

def setSpeed(input):
    global Speed
    Speed = input

def readGPS():
    """
    It reads the GPS Serial port and translates NMEA to usable values. The usb port for the gps needs to be the leftmost one on this particular panda.
    """
    try:
        data = ""
        gps = serial.Serial(port = gpsPort, baudrate=115200, timeout=5)
        gps.flushInput()  # flush input buffer, discarding all its contents
        gps.flushOutput()
        sleep(.05)
        data = gps.readline()
        sleep(.05)
        gps.close()
        data = data.decode("utf-8")
        dataParse = pynmea2.parse(data)
        gpsLat = dataParse.latitude
        gpsLon = dataParse.longitude
        sats = dataParse.num_sats
        return gpsLat, gpsLon, int(sats)
    except Exception as e:
        return e
    


def deg2rad(deg):
    '''
    converts degrees to radians, pretty simple
    '''
    return deg*(pi/180)


def rad2deg(rad):
    '''
    converts radians to degrees, pretty simple
    '''
    return rad*180/pi


def distanceToWaypoint(xp, yp, xw, yw):
    """
    computes the distance to the waypoint
    """
    return ((xp-xw)**2+(yp-yw)**2)**(1/2)


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


def XYtolatlon(x, y, aspectRatio):
    """
    This function converts x,y back to lattitude and longitude using simple equirectangular projection. X and y need to be in meters.
    """
    r = 6371000
    lat = rad2deg(y/r)
    lon = rad2deg(x/(r*aspectRatio))
    return lat, lon

def betweenWaypoints(x1, y1, x2, y2, xp, yp):
    """
    This function is for waypoint following. It decides whether to look at a point on the line segment or one of the endpoints to follow.
    """
    xval = [x1, x2]
    xval = sort(xval)
    yval = [y1, y2]
    yval = sort(yval)
    if x1 == x2:
        if yp > yval[1] or yp < yval[0]:
            return False
    elif y1 == y2:
        if xp > xval[1] or xp < xval[0]:
            return False
    else:
        m = (y2-y1)/(x2-x1)
        intercept1 = y1+x1/m
        intercept2 = y2+x2/m
        a = yp+xp/m
        if intercept1 > intercept2:
            if (a-intercept1) > 0 or (a-intercept2) < 0:
                return False
        else:
            if (a-intercept1) < 0 or (a-intercept2) > 0:
                return False
    return True


def calcAngleError(x1, y1, x2, y2, xp, yp, robotAngle, r): 
    """
    This is the error calculator for the pure persuit line following algorithm. The robot maintains a constant speed and only uses the error in desired and current angle to navigate.
    """
    pt1Dist = ((xp-x1)**2 + (yp-y1)**2)**(1/2)
    pt2Dist = ((xp-x2)**2 + (yp-y2)**2)**(1/2)
    if x2 == x1:
        Acoef = 1
        Bcoef = -2*yp
        Ccoef = x2**2-2*xp*x2+xp**2+yp**2-r**2
        perpDist = ((x2-xp)**2)**(1/2)
    else:
        m = (y2-y1)/(x2-x1)
        b = y2-m*x2
        Acoef = (m**2+1)
        Bcoef = 2*m*b-2*m*yp-2*xp
        Ccoef = xp**2+b**2-2*b*yp+yp**2-r**2
        if betweenWaypoints(x1, y1, x2, y2, xp, yp) == True:
            perpDist = abs(-m*xp+yp-b)/(m**2+1**2)**(1/2)
        else:
            perpDist = pt1Dist+1

    distances = [pt1Dist, pt2Dist, perpDist]
    minDist = distances.index(min(distances))
    descriminant = Bcoef**2-4*Acoef*Ccoef

    if distances[minDist] > r or descriminant < 0:
        if minDist == 0:
            x = x1
            y = y1
        elif minDist == 1:
            x = x2
            y = y2
        else:
            if y2 == y1:
                x = xp
                y = y2
            elif x2 == x1:
                x = x2
                y = yp
            else:
                x = (xp/m+yp-b)/(m+1/m)
                y = m*x+b

    else:
        xpot1 = (-Bcoef + (descriminant)**(1/2))/(2*Acoef)
        xpot2 = (-Bcoef - (descriminant)**(1/2))/(2*Acoef)

        if x1 == x2:
            ypot1 = xpot1
            ypot2 = xpot2
            xpot1 = x2
            xpot2 = x2
        else:
            ypot1 = m*xpot1+b
            ypot2 = m*xpot2+b

        dpot1 = ((xpot1-x2)**2+(ypot1-y2)**2)**(1/2)
        dpot2 = ((xpot2-x2)**2+(ypot2-y2)**2)**(1/2)
        if (dpot2 > dpot1):
            x = xpot1
            y = ypot1
        else:
            x = xpot2
            y = ypot2

    xr = xp+r*sin(robotAngle)
    yr = yp+r*cos(robotAngle)
    ax = xr-xp
    ay = yr-yp
    bx = x-xp
    by = y-yp
    angleError = 0
    if (ax != 0 or ay != 0) and (bx != 0 or by != 0): 
        try:
            angleError = acos(
                (ax*bx+ay*by)/(((ax**2+ay**2)**(1/2))*((bx**2+by**2)**(1/2))))
        except:
            pass
    cp = ax*by-bx*ay
    if cp < 0:
        angleError = -1*angleError
    return angleError


def movingAverageFilter(error, errorlist, filtersize):
    i = len(errorlist)
    if i > filtersize:
        err = 0
        j = 1
        while j < filtersize:
            err += errorlist[i-j]
            j += 1
        err  = (err+error)/(filtersize)
    else:
        err = error
    return err


def writeToArduino(x, microcontroller): 
    """
    This function communicates with the latte panda's onboard arduino or USB connected arduino. It takes what is being written and a port name.
    """
     
    try:
        microcontroller.write(bytes(x, 'utf-8'))
        microcontroller.flushInput()  # flush input buffer, discarding all its contents
        microcontroller.flushOutput()
    except Exception as e:
        print(str(e))
        pass
    return


def write_read(x, microcontroller):  # communucation btw the cpu and ard
    """
    This function communicates with the latte panda's onboard arduino or USB connected arduino and receives a return signal. It takes what is being written and a port name.
    """
    try:
        # arduino = serial.Serial(port = portName, baudrate=9600, timeout=5)
        microcontroller.flushInput()  # flush input buffer, discarding all its contents
        microcontroller.flushOutput()
        microcontroller.write(bytes(x, "utf-8"))
        data = microcontroller.readline()
        data = data.decode("utf-8")
        return data
    except Exception as e:
        print(str(e))
        pass
    return

#This enables manual control from another computer
def manualControl(keystroke):
    """
    This function enables manual control from another radio receiver connected computer. It returns the desired pwm inputs for steering and wheel motion.
    """
    global Speed
    global angleSignal
    global velocitySignal
    global mode
    global setpoint
    global speedset
    adjustedSpeed = Speed-1500
    

    if keystroke == "l":
        speedset *= -1

    if keystroke == "1":
        setpoint = 0
        mode = 1
    elif keystroke == "2":
        angleSignal = 0
        mode = 2

    if mode == 1:
        if keystroke == "a":
            angleSignal -= .05
        elif keystroke == "d":
            angleSignal += .05
    elif mode == 2:
        if keystroke == "a":
            setpoint -= 10
        elif keystroke == "d":
            setpoint += 10

    if keystroke == "w":
        velocitySignal = 1500 + adjustedSpeed
    elif keystroke == "s":
        velocitySignal = 1500 - adjustedSpeed

    if keystroke == "" and speedset < 0:
        velocitySignal = 1500
    # print(angleSignal)
    return str(angleSignal)+","+str(velocitySignal)+","+str(setpoint)


def waypointFollwerVariableInits(ki, kp, kd, R):
    """
    This function initialized the starting values for autonomous waypoint following.
    """
    global x
    global y
    global xPath
    global yPath
    global robotAngle
    global oldErr
    global oldTime
    global intErr
    global wpNum
    global kI
    global kP
    global kD
    global r
    global latPath
    global lonPath
    latPath = []
    lonPath = []
    r = R
    kI = ki
    kP = kp
    kD = kd
    x = 0
    y = 0
    xPath = []
    yPath = []
    robotAngle = 0
    oldErr = 0
    oldTime = perf_counter()
    intErr = 0
    wpNum = 0

def computePID(err, dutycycle):
    """
    This is the pid control calculator, velocity may need to be adjusted
    """
    velocity = ((dutycycle-1500)/500)*1.388 #m/s, constant derived from wheel diameter and measured rpm
    global oldTime
    global intErr
    global oldErr
    intMax = 2.0
    nowTime = perf_counter()
    deltaT = nowTime-oldTime
    try:
        err = atan(err*0.5*(robotLength/(velocity*deltaT))) #convert to actual robot turn angle
    except:
        pass
    intErr += err*deltaT
    if abs(intErr) > intMax:
        intErr = sign(intErr)*intMax 
    derErr = (err-oldErr)/deltaT
    out = kP*err+kI*intErr-kD*derErr
    oldTime = nowTime
    oldErr = err
    return out


def logPath(lat, lon, name):
    """
    This writes a series of position values to a .csv file for later use.
    """

    with open(name, 'w') as f:
        f.write('')
    with open(name, 'a', newline = '') as f:
        writer = csv.writer(f)
        i = 0
        while i < len(lat):

            writer.writerow([lat[i],lon[i]])
            i += 1


def initializeWaypointFollower(name):
    """
    This is to set the inital error so the derivative control doesn't do funny things on startup. The function is just one pass through the waypoint follower loop without actually issuing any motor commands.
    """
    global waypoints
    global refLat
    global aspectRatio
    global xwaypoints
    global ywaypoints
    global wpNum
    global oldErr
    waypoints = genfromtxt(name, delimiter=',')
    refLat = deg2rad(waypoints[0,0])
    aspectRatio = cos(refLat)
    xwaypoints = []
    ywaypoints = []
    i = 0
    while i < len(waypoints):
        
        [xpoints, ypoints] = latlonToXY(waypoints[i,0],waypoints[i,1],aspectRatio)
        xwaypoints.append(xpoints)
        ywaypoints.append(ypoints)
        i += 1

    try:
        [gps_lat,gps_lon,sats] = readGPS()

        [xp, yp] = latlonToXY(gps_lat, gps_lon, aspectRatio)
    except:
        print("No connection to GPS")
        pass
# Set first old error equal to the new one to stop weird inital derivative error behavior
    robotAngle = deg2rad(float(write_read('C', sensorArduino)))
    [xWaypoint,yWaypoint] = xwaypoints[wpNum],ywaypoints[wpNum]
    if wpNum == 0:
        [xlastWaypoint, ylastWaypoint] = [xWaypoint, yWaypoint]
    else:
        [xlastWaypoint,ylastWaypoint] = xwaypoints[wpNum-1],ywaypoints[wpNum-1]
    while distanceToWaypoint(xp,yp,xWaypoint,yWaypoint) < r: #distance is in m, maybe use lat/lon to X/Y?
        wpNum +=1
        [xWaypoint,yWaypoint] = xwaypoints[wpNum],ywaypoints[wpNum]
        print("waypoint "+str(wpNum)+" reached")
    oldErr = calcAngleError(xlastWaypoint,ylastWaypoint,xWaypoint,yWaypoint,xp,yp,robotAngle,r)


def waypointFollower(ki,kp,kd,lookahead, receiver, remoteTransmitter, filename):
    """
    This is the meat and potatoes of the robot. It takes in a set of waypoints and follows them by issuing commands to the onboard arduino. It needs pid control constants, the look ahead distance, a threshold at which to stop going forward and focus on turning, a PWM speed (in microseconds) for the motors, a file name, and a radio reciever and transmitter object.
    """
    global message
    global wpNum
    global latPath
    global lonPath
    global Speed
    outputlist = []
    errplot = []
    timeplot = []
    waypointFollwerVariableInits(ki,kp,kd,lookahead)
    initializeWaypointFollower(filename)
    while True:
    # Take GPS measurement and compass measurement
        robotAngle = deg2rad(float(write_read('C', sensorArduino)))
        try:
            [gps_lat,gps_lon,sats] = readGPS()
            if sats != 0:
                [xraw, yraw] = latlonToXY(gps_lat, gps_lon, aspectRatio)
                xcenter = xraw+(robotWidth/2)*sin(robotAngle)
                ycenter = yraw+(robotWidth/2)*cos(robotAngle)
                [latAdjusted, lonAdjusted] = XYtolatlon(xcenter,ycenter,aspectRatio)
                latPath.append(latAdjusted)
                lonPath.append(lonAdjusted)
                xp = xcenter
                yp = ycenter
        except:
            print("No connection to GPS")
            pass
    
    # Read and select waypoint values
        [xWaypoint,yWaypoint] = xwaypoints[wpNum],ywaypoints[wpNum]
        if wpNum == 0:
            [xlastWaypoint, ylastWaypoint] = [xWaypoint, yWaypoint]
        else:
            [xlastWaypoint,ylastWaypoint] = xwaypoints[wpNum-1],ywaypoints[wpNum-1]
    # If robot within threshold distance increment to next waypoint
        while distanceToWaypoint(xp,yp,xWaypoint,yWaypoint) < r: #distance is in m, maybe use lat/lon to X/Y?
            wpNum +=1
            [xWaypoint,yWaypoint] = xwaypoints[wpNum],ywaypoints[wpNum]
            print("waypoint "+str(wpNum)+" reached")
        if wpNum >= len(waypoints): # Check for more waypoints
            writeToArduino("S0,1500,0", steeringArduino)
            break
        try:
            data = receiver.read_data_from(remoteTransmitter, 0.1)
            message = data.data.decode("utf8")
            writeToArduino("S0,1500,0", steeringArduino)
            break
        except:
            pass
    # Calculate angle error
        err = calcAngleError(xlastWaypoint,ylastWaypoint,xWaypoint,yWaypoint,xp,yp,robotAngle,r)
        infostr = str(distanceToWaypoint(xp,yp,xWaypoint,yWaypoint))+" m,"+str(rad2deg(err))+" degrees"
        receiver.send_data_async(remoteTransmitter, infostr)
        errplot.append(err)
        timeplot.append(perf_counter())
    # Run error through PID control for steering
        output = computePID(-err, Speed)
        output = movingAverageFilter(output,outputlist,1)
        outputlist.append(output)

    # Output to steering motors
        writeToArduino("S"+str(output)+","+str(Speed)+","+str(0), steeringArduino)
    # Check for obstacles

    # Turn parallel to obstacle
    
    #Log robot path
    logPath(latPath,lonPath,"traversedPath.csv")
    logPath(timeplot,errplot,"erroroutput.csv")
    logPath(timeplot,outputlist,"PIDoutput.csv")