import serial.tools.list_ports
try:
    print(list(*serial.tools.list_ports.grep('FT232EX'))[0])
except:
    print("Radio not connected.")
try:
    print(list(*serial.tools.list_ports.grep('Controller'))[0])
except:
    print("GPS not connected.")
try:
    print(list(*serial.tools.list_ports.grep('USB Serial'))[0])
except:
    print("not connected.")