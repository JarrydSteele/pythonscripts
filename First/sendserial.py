from threading import Thread
from typing import ByteString
import serial, time
import random

stop_threads = False
global int_s
int_s = 0
global counter
counter = 0
connected = False


print("Connecting to Telemetry Radio")
try:
    telem = serial.Serial(port='/dev/ttyUSB0',baudrate=57600,timeout=1)                            # open serial port
    print("Connected to Telemetry Radio ")                            # check which port was really used
    connected = True
except:
    print("Could not connect to Telemetry Radio")

def get_telem_serial():                                            # get serial data from teensy
    while(True):

        global stop_threads
        global counter

        rand1 = random.randint(0,100)
        rand2 = random.randint(0,100)
        rand3 = random.randint(0,100)

        counter += 1
        data = str(counter) + "," + str(rand1) + "," + str(rand2) + "," + str(rand3) + "\n"
        encoded_data = data.encode('utf-8')

        if (connected):
            telem.write(encoded_data)
            print("Sent: %s" % data)
        else:
            print("Telem not connected")

        time.sleep(0.1)

        if stop_threads:
            print("stop_threads = true")
            break

global telem_serial_thread
telem_serial_thread = Thread(target=get_telem_serial, args=())
telem_serial_thread.daemon = True
telem_serial_thread.start()

def switch():

    option = input("Enter your option ('e' to exit): ")

    if option == 'e':
        print("Exiting Program")
        telem.close()
        time.sleep(1)
        return

    elif option == 'kill':
        print("Stopping all threads")
        global stop_threads
        stop_threads = True
        telem_serial_thread.join()
        switch()
    

 
    else:
        print("Incorrect option")
        switch()

switch()