from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import pynput
import serial.tools.list_ports
# print(list(*serial.tools.list_ports.comports()))
#For windows, comment out the below try, except lines and replace radioPort with a the name of the COM port that the radio is connected to.
print(list(*serial.tools.list_ports.grep('FT231X')))
try:
    radioPort = list(*serial.tools.list_ports.grep('FT231X'))[0]
except:
    print("Radio not connected.")
transmitter = XBeeDevice(radioPort, 9600)

remoteReceiver = RemoteXBeeDevice(transmitter, XBee64BitAddress.from_hex_string("0013A20040FCB774"))
transmitter.open()
inputName = False

def on_press(key):
    global inputName
    try:
        print(key.char)
        transmitter.send_data_async(remoteReceiver, key.char)
        if key.char == '0':
            print("manual stopped")
            return False
        elif key.char == '3': # fix this
            print("manual paused, press enter once")
            inputName = True
            return False
        elif key.char == '5':
            try:
                data = transmitter.read_data_from(remoteReceiver, 3)
                message = data.data.decode("utf8")
                print(message)
            except:
                print("No transmission")
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

# def my_data_received_callback(xbee_message):
#     address = xbee_message.remote_device.get_64bit_addr()
#     data = xbee_message.data.decode("utf8")
#     print("Received data from %s: %s" % (address, data))
def beginManual():
    listener = pynput.keyboard.Listener(on_press=on_press)
    listener.start()

# receiver =XBeeDevice("COM22", 9600)
# remoteTransmitter = RemoteXBeeDevice(receiver, XBee64BitAddress.from_hex_string("0013A2004104110E"))
# receiver.open()
# message = receiver.read_data_from(remoteTransmitter, 5)
# message = receiver.read_data(5)
# print(message)
# my_data_received_callback(message)
# receiver.close()


if __name__ == "__main__":
    while True:
        x = input()
        message = ""
        if x == "ping":
            if remoteReceiver.reachable:
                print("Reciever online")
            else:
                print("Reciever offline")
            transmitter.flush_queues()
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 3)
                message = data.data.decode("utf8")
                print(message)
            except:
                print("Robot computer offline")
                
        elif x == "gps reading":
            transmitter.flush_queues()
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 10)
                message = data.data.decode("utf8")
                print(message)
            except:
                print("No data recieved")

        elif x == "power reading":
            transmitter.flush_queues()
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 3)
                message = data.data.decode("utf8")
                print("Power from batteries, power from solar, system input voltage(W,W,V): " + message)
            except:
                print("No data recieved")

        elif x == "compass reading":
            transmitter.flush_queues()
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 3)
                message = data.data.decode("utf8")
                print(message)
            except:
                print("No data recieved")

        elif x == "set speed":
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 5)
                message = data.data.decode("utf8")
                print("\n"+message)
                speed = input()
                transmitter.send_data_async(remoteReceiver, speed)
            except:
                print("No response from robot")
           
            
        elif x == "manual":
            print("----------------------------------\nManual mode activated, avialable commands are: \n   w,a,s,d - where a and d are for turning, and w and s are forward and backward, respectively\n   l-lock forward or reverse\n   1 - for dual Ackerman steering\n   2 - for synchronous steering\n   3 - to record a gps coordinate path\n   4 - stop recording path and write to file\n   5 - read transmission\n   0 - stop manual mode\n----------------------------------\n")
            transmitter.send_data_async(remoteReceiver, x)
            beginManual()

        elif inputName == True: # fix this
            try:
                data = transmitter.read_data_from(remoteReceiver, 3)
                message = data.data.decode("utf8")
                print("\n"+message)
                name = input()
                transmitter.send_data_async(remoteReceiver, name)
            except:
                print("Robot computer offline")
            inputName = False
            print("manual resumed")
            beginManual()
            

        elif x == "autonomous":
            transmitter.send_data_async(remoteReceiver, x)
            try:
                data = transmitter.read_data_from(remoteReceiver, 5)
                message = data.data.decode("utf8")
                counter = int(message)
                i = 0
                while i < counter:
                    data = transmitter.read_data_from(remoteReceiver, 5)
                    message = data.data.decode("utf8")
                    print(message)
                    i+=1
                name = input()
                transmitter.send_data_async(remoteReceiver, name)
            except:
                print("No response from robot")
            while input("Press enter to refresh, or 0 then enter to exit.") != '0':
                try:
                    transmitter.flush_queues()
                    data = transmitter.read_data_from(remoteReceiver, 3)
                    message = data.data.decode("utf8")
                    print(message)
                except:
                    print("No error message recieved")
            
            
            
        elif x == "STOP ROBOT":
            transmitter.send_data_async(remoteReceiver, x)
        elif x =="STOP CONTROLLER":
            break
        else:
            print("----------------------------------\nUnrecognized command, avialable commands are: \n   ping\n   set speed\n   gps reading\n   power reading\n   compass reading\n   manual\n   autonomous\n   STOP ROBOT\n   STOP CONTROLLER\n----------------------------------\n")

    print("loop ended")
    transmitter.close()
