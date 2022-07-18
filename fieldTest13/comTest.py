import methods

print(str(methods.initArduinos()))

while True:
    x = input()
    if x == "STOP":
        break
    print(methods.write_read(x, methods.steeringArduino))    