from curses import baudrate
import serial, time 

print("Connecting to Telemetry Radio")
try:
    telem = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1)                            # open serial port
    print("Connected to Telemetry Radio ")                            # check which port was really used
except:
    print("Could not connect to Telemetry Radio")

while(True):

    s = telem.readline(50)
    print(s.decode('utf-8'))
    time.sleep(0.1)