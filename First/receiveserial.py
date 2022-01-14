from threading import Thread
import serial, time

stop_threads = False   

print("Connecting to Telemetry Radio")
try:
    telem = serial.Serial('/dev/ttyUSB0')                            # open serial port
    print("Connected to Telemetry Radio ")                            # check which port was really used
except:
    print("Could not connect to Telemetry Radio")

def get_telem_serial():                                            # get serial data from teensy
    while(True):
        global s
        global int_s
        global stop_threads
        
        s = telem.readline().decode().strip()
        int_s = int(telem.readline().decode().strip())
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
        print("Closing Vehicles and Exiting Program")
        telem.close()
        time.sleep(2)
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