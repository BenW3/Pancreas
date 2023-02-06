#!/usr/bin/python3

# from time import sleep
import methods
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import os
import glob
from math import cos, sin
import sys
from time import perf_counter
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
powerReadingDelay = 10
aspectRatio = 0.0
methods.getSerialPorts()
receiver = XBeeDevice(methods.radioPort, 9600)
remoteTransmitter = RemoteXBeeDevice(receiver, XBee64BitAddress.from_hex_string("0013A2004104110E"))
receiver.open()
print(str(methods.initArduinos()))

if __name__ == "__main__":
    print("Pancreas Online")
    while True:
        message = ""
        try:
            data = receiver.read_data_from(remoteTransmitter, 3)
            message = data.data.decode("utf8")
            print(message)
        except:
            # print("no message")
            pass
        if message == "manual":
            # print("manual mode activated")
            global pathName
            global powerName
            global CurrentTime
            lat = []
            lon = []
            power = []
            heading = []
            logPath = False
            pathName = ""
            while message != '0':
                try:
                    data = receiver.read_data_from(remoteTransmitter, 0.1)
                    receiver.flush_queues()
                    message = data.data.decode("utf8")

                except:
                    message = ""
                if message == "3":
                    # powerName = "powerDefault.csv"
                    CurrentTime = perf_counter()
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
                    # power.append("starting file")
                    logPath = False
                    print("Getting gps . . .")
                    i = 0
                    while i < 5 and logPath == False: 
                        try:
                            i += 1
                            [reflat,satnum] = methods.readGPS()[0:3:2]
                            aspectRatio = cos(reflat)
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
                            data.append(str(lat[i])+","+str(lon[i])+","+str(heading[i]))
                            i += 1
                        methods.logDataUpdate(data, pathName)
                        lat = []
                        lon = []
                        heading = []
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

                if message == "4":
                    # try:
                        # receiver.flush_queues()
                        # receiver.send_data_async(remoteTransmitter, "Please supply a file name with a .csv extension within the next 30s")
                        # receiver.flush_queues()
                        # data = receiver.read_data_from(remoteTransmitter, 30)
                        # name = data.data.decode("utf8")
                    logPath = False
                    # except:
                    #     name = "defaultName.csv"
                    try:
                        data = []
                        i  = 0
                        while i < len(lat):
                            data.append(str(lat[i])+","+str(lon[i])+","+str(heading[i]))
                            i += 1
                        methods.logDataUpdate(data, pathName)
                        lat = []
                        lon = []
                        heading = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))
                        
                    try:
                        # power.append("ending file")
                        methods.logDataUpdate(power, powerName)
                        power = []
                    except Exception as e:
                        print(str(e))
                        receiver.send_data_async(remoteTransmitter, str(e))

                if logPath == True:
                    # receiver.send_data_async(remoteTransmitter, str(CurrentTime - perf_counter()))
                    if (perf_counter()-CurrentTime) > powerReadingDelay:
                        CurrentTime = perf_counter()
                        # receiver.send_data_async(remoteTransmitter, str("taking power reading"))
                        try:

                            power.append(str(methods.write_read('P', methods.sensorArduino)))
                        except:
                            pass
                    try:
                        [gps_lat, gps_lon, sats] = methods.readGPS()[0:3]
                        robotAngle = methods.deg2rad(float(methods.write_read('C', methods.sensorArduino)))
                        if sats != 0:
                            [xraw, yraw] = methods.latlonToXY(gps_lat, gps_lon, aspectRatio)
                            xcenter = xraw+(methods.robotWidth/2.0)*cos(robotAngle)
                            ycenter = yraw-(methods.robotWidth/2.0)*sin(robotAngle)
                            [latAdjusted, lonAdjusted] = methods.XYtolatlon(xcenter,ycenter,aspectRatio)
                            lat.append(latAdjusted)
                            lon.append(lonAdjusted)
                            heading.append(robotAngle)
                    except Exception as e:
                        exception_type, exception_object, exception_traceback = sys.exc_info()
                        filename = exception_traceback.tb_frame.f_code.co_filename
                        line_number = exception_traceback.tb_lineno
                        receiver.send_data_async(remoteTransmitter, str(e) +", "+ str(exception_type)+", "+str(filename)+", "+str(line_number))
                        pass
                # print(methods.manualControl(message))
                # var = str(methods.write_read('S'+ methods.manualControl(message), methods.steeringArduino))
                # print(var)
                # print('S'+ methods.manualControl(message))
                methods.writeToArduino('S'+methods.manualControl(message), methods.steeringArduino)
                

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

        elif message == "gps reading":
            try:
                receiver.send_data_async(remoteTransmitter, str(methods.readGPS()))
            except Exception as e:
                print("Failed to send GPS data")
                print(str(e))
                receiver.send_data_async(remoteTransmitter, str(e))
        
        elif message == "power reading":
            try:
                receiver.send_data_async(remoteTransmitter, methods.write_read('P', methods.sensorArduino))
            except Exception as e:
                print("Failed to send power data, " + str(e))
                receiver.send_data_async(remoteTransmitter, str(e))

        elif message == "compass reading":
            try:
                receiver.send_data_async(remoteTransmitter, methods.write_read('C', methods.sensorArduino))
            except Exception as e:
                print("Failed to send compass data")
                receiver.send_data_async(remoteTransmitter, str(e))

        elif message == "ping":
            receiver.send_data_async(
                remoteTransmitter, "Robot computer online")

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
