#!/usr/bin/python3

# from time import sleep
import methods
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import os
import glob
from math import cos, sin
import sys
from time import perf_counter
import numpy as np
# from CustomKalman import TwoDKalman
#--------------------
# Pancreas Main Code
#--------------------
    #................
    # Necessary code blocks to write
    #................
    # - CHECK - PID control
    # - CHECK - Error calculator
    # - CHECK - Angle/Dist to pwm calculator
    # - CHECK - GPS reader
    # - CHECK - Compass reader
    # - CHECK - Read/write to coordinate file
    # - CHECK - Stop conditions
    # - CHECK - Read/write to log file
    # Obstacle sensor
    # - CHECK - Read power
    #................
#--------------------

#--------------------
# Global Variables
#--------------------
powerReadingDelay = 10
aspectRatio = 0.0
average_timestep = 0.5 #seconds UPDATE THIS!
# XVariance = 5 #noise variance in meters for longitude
# YVariance = 5 #noise variance in meters for latitude
# XVelocityVariance = 0.05 #noise varience in measured velocity
# YVelocityVariance = 0.05 #noise varience in measured velocity
methods.getSerialPorts()
receiver = XBeeDevice(methods.radioPort, 9600)
remoteTransmitter = RemoteXBeeDevice(receiver, XBee64BitAddress.from_hex_string("0013A2004104110E"))
receiver.open()
print(str(methods.initArduinos()))


#--------------------
# Main Radio Message Handler
#--------------------
if __name__ == "__main__":
    print("Pancreas Online")
    #--------------------
    # Read Radio Messages Until Loop Ends
    #--------------------
    while True:
        message = ""
        try:
            data = receiver.read_data_from(remoteTransmitter, 3)
            message = data.data.decode("utf8")
            print(message)
        except:
            # print("no message")
            pass
    #--------------------
    # Manual Mode
    #--------------------
        if message == "manual":
            # print("manual mode activated")
            global pathName
            global powerName
            global CurrentTime
            global t1
            lat = []
            lon = []
            power = []
            heading = []
            dt = []
            logPath = False
            pathName = ""
            #--------------------
            # Continue Reading Messages 
            #--------------------
            while message != '0':
                try:
                    data = receiver.read_data_from(remoteTransmitter, 0.1)
                    receiver.flush_queues()
                    message = data.data.decode("utf8")

                except:
                    message = ""
            #--------------------
            # Path and power recording
            #--------------------
                if message == "3":
                    # powerName = "powerDefault.csv"
                    CurrentTime = perf_counter()
                    t1 = perf_counter()

                    try:
                        print("Getting file name . . .")
                        receiver.flush_queues()
                        receiver.send_data_async(remoteTransmitter, "Please supply a file name with a .csv extension within the next 30s")
                        receiver.flush_queues()
                        data = receiver.read_data_from(remoteTransmitter, 30)
                        pathName = data.data.decode("utf8")
                        powerName = str("power"+str(pathName))
                    except:
                        pathName = "pathDefaultName.csv"
                        powerName = "powerDefaultName.csv"
                    try:
                        methods.logDataInit(pathName)
                        methods.logDataInit(powerName)
                    except Exception as e:
                        receiver.send_data_async(remoteTransmitter, str(e))
                    lat = []
                    lon = []
                    heading = []
                    power = []
                    dt = []
                    # power.append("starting file")
                    logPath = False
                    print("Getting gps . . .")
                    i = 0
                    while i < 5 and logPath == False: 
                        try:
                            i += 1
                            [reflat,reflon,satnum] = methods.readGPS()[0:3]
                            aspectRatio = cos(reflat)
                            [x1,y1] = methods.latlonToXY(reflat,reflon,aspectRatio)
                            robot_heading = methods.deg2rad(float(methods.write_read('C', methods.sensorArduino)))
                            initial_state = np.array([[x1],[y1],[sin(robot_heading)*methods.velocityMagnitude],[cos(robot_heading)*methods.velocityMagnitude]])
                            methods.filterInit(initial_state,average_timestep)
                            receiver.send_data_async(remoteTransmitter, "try "+str(i)+", "+str(reflat))
                            if satnum !=0:
                               logPath = True
                               receiver.send_data_async(remoteTransmitter, "Success!")
                               print("Recording path")
                        except Exception as e:
                            receiver.send_data_async(remoteTransmitter, str(e))

                if len(lat) > 20:
                    print(str(len(lat)) + " vals in list")
                    try:
                        data = []
                        i  = 0
                        while i < len(lat):
                            data.append(str(lat[i])+","+str(lon[i])+","+str(heading[i]) + "," + str(dt[i]))
                            i += 1
                        methods.logDataUpdate(data, pathName)
                        lat = []
                        lon = []
                        heading = []
                        dt = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))
                
                if len(power) > 20:
                    # print(str(len(power)) + " vals in list")
                    try:
                        methods.logDataUpdate(power, powerName)
                        power = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))
            #--------------------
            # Stop recording and write to file
            #--------------------
                if message == "4":
                    logPath = False
                    try:
                        data = []
                        i  = 0
                        while i < len(lat):
                            data.append(str(lat[i])+","+str(lon[i])+","+str(heading[i])+ "," + str(dt[i]))
                            i += 1
                        methods.logDataUpdate(data, pathName)
                        lat = []
                        lon = []
                        heading = []
                        dt = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))
                        
                    try:
                        methods.logDataUpdate(power, powerName)
                        power = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))

                if logPath == True:
                    if (perf_counter()-CurrentTime) > powerReadingDelay:
                        CurrentTime = perf_counter()
                        try:
                            power.append(float(methods.write_read('P', methods.sensorArduino)))
                        except:
                            pass
                    try:
                        [gps_lat, gps_lon, sats] = methods.readGPS()[0:3]
                        robotAngle = methods.deg2rad(float(methods.write_read('C', methods.sensorArduino)))
                        if sats != 0:
                            [xraw, yraw] = methods.latlonToXY(gps_lat, gps_lon, aspectRatio)
                            xcenter = xraw+(methods.robotWidth/2.0)*cos(robotAngle)
                            ycenter = yraw-(methods.robotWidth/2.0)*sin(robotAngle)
                            measured_state = np.array([[xcenter],[ycenter],[sin(robotAngle)*methods.velocityMagnitude],[cos(robotAngle)*methods.velocityMagnitude]])
                            [xfilter,yfilter] = methods.filteredGPS()
                            [latAdjusted, lonAdjusted] = methods.XYtolatlon(xfilter,yfilter,aspectRatio)
                            lat.append(latAdjusted)
                            lon.append(lonAdjusted)
                            heading.append(robotAngle)
                            dt.append(perf_counter() - t1)
                            t1 = perf_counter()
                    except Exception as e:
                        exception_type, exception_object, exception_traceback = sys.exc_info()
                        filename = exception_traceback.tb_frame.f_code.co_filename
                        line_number = exception_traceback.tb_lineno
                        receiver.send_data_async(remoteTransmitter, str(e) +", "+ str(exception_type)+", "+str(filename)+", "+str(line_number))
                        pass
                methods.writeToArduino('S'+methods.manualControl(message), methods.steeringArduino)
                
    #--------------------
    # Setting speed
    #--------------------
        elif message == "set speed":
            try:
                receiver.send_data_async(remoteTransmitter, 'Input a speed in microseconds from 1500 to 2000. Current speed is ' + str(methods.Speed))
                receiver.flush_queues()
                data = receiver.read_data_from(remoteTransmitter, 30)
                speed = int(data.data.decode("utf8"))
                methods.setSpeed(speed)
            except Exception as e:
                print("Error with speed setting")
                print(str(e))
                receiver.send_data_async(remoteTransmitter, str(e))
    #--------------------
    # Read GPS
    #--------------------
        elif message == "gps reading":
            try:
                receiver.send_data_async(remoteTransmitter, str(methods.readGPS()))
            except Exception as e:
                print("Failed to send GPS data")
                print(str(e))
                receiver.send_data_async(remoteTransmitter, str(e))
    #--------------------
    # Read Power
    #--------------------
        elif message == "power reading":
            try:
                receiver.send_data_async(remoteTransmitter, methods.write_read('P', methods.sensorArduino))
            except Exception as e:
                print("Failed to send power data, " + str(e))
                receiver.send_data_async(remoteTransmitter, str(e))
    #--------------------
    # Read Compass
    #--------------------
        elif message == "compass reading":
            try:
                receiver.send_data_async(remoteTransmitter, methods.write_read('C', methods.sensorArduino))
            except Exception as e:
                print("Failed to send compass data")
                receiver.send_data_async(remoteTransmitter, str(e))
    #--------------------
    # Test For Connection
    #--------------------
        elif message == "ping":
            receiver.send_data_async(
                remoteTransmitter, "Robot computer online")
    #--------------------
    # Run Autonomously From File
    #--------------------
        elif message == "autonomous":
            filelist = []
            counter = 1
            filestr = "Please type in the name of one of the available coordinate files which you would like to follow or type CANCEL: \n"
            filelist.append(filestr)
            files = glob.glob('./*.csv')
            for f in files:
                filestr = str(f)+" ["+str(os.path.getsize(f))+" bytes"+"]"
                filelist.append(filestr) 
                counter +=1
            print(filelist)
            i = 0
            receiver.send_data_async(
                    remoteTransmitter, str(counter)) 
            receiver.flush_queues()
            while i < counter:
                receiver.send_data_async(
                    remoteTransmitter, str(filelist[i])) 
                i += 1
                print(i)
            try:
                data = receiver.read_data_from(remoteTransmitter, 60)
                userFilename = data.data.decode("utf8") 
            except:
                userFilename = 'CANCEL'
            if userFilename != 'CANCEL':
                try:
                    methods.waypointFollower(0.0,1.0,0.0,1,receiver,remoteTransmitter, userFilename)
                except Exception as e:
                    print(e)
                    receiver.send_data_async(remoteTransmitter, str(e))
            print(userFilename)
            message == ""

        elif message == "STOP ROBOT":
            break

    receiver.close()
