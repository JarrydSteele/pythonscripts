from curses import baudrate
from threading import Thread
import serial, time

stop_threads = False
global out
out = ''   

print("Connecting to Telemetry Radio")
try:
    telem = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=10)                            # open serial port
    print("Connected to Telemetry Radio ")                            # check which port was really used
except:
    print("Could not connect to Telemetry Radio")

def get_telem_serial():                                            # get serial data from teensy
    while(True):
        global s
        global int_s
        global stop_threads
        global out
    
        #r = telem.read().decode().strip()
        #print(r)

        s = telem.readline()
        print(s.decode('utf-8').strip())
        #int_s = int(telem.read().decode().strip())
        #print(int_s)

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