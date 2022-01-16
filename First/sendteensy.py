import serial, time
from threading import Thread
from physical_vehicle import PhysicalVehicle

pv_1 = PhysicalVehicle()
stop_threads = False

connected = False

global telem_send_time

telem_send_period = 500000000
telem_send_time = time.time_ns() + telem_send_period

global s
s = ""

print("Connecting to Telemetry Radio")
try:
    telem = serial.Serial(port='/dev/ttyUSB0',baudrate=57600,timeout=1)                            # open serial port
    print("Connected to Telemetry Radio ")                            # check which port was really used
    connected = True
except:
    print("Could not connect to Telemetry Radio")

print("Connecting to Teensy")
try:
    teensy = serial.Serial('/dev/teensy4.1')                            # open serial port
    print("Connected to: %s " % teensy.name)                            # check which port was really used
except:
    print("Could not connect to Teensy")

def get_teensy_serial():                                            # get serial data from teensy
    while(True):
        global s
        global stop_threads
        
        s = teensy.readline().decode().strip()
        #print(s)

        data = s.split(', ')

        for x in range(8):
            pv_1.adc[x] = float(data[x])

        pv_1.roll = float(data[8])
        pv_1.pitch = float(data[9])
        pv_1.yaw = float(data[10])

        pv_1.pressure = float(data[11])
        pv_1.temp_c = float(data[12])
        pv_1.alt = float(data[13])

        pv_1.gps_latitude = float(data[14])
        pv_1.gps_longitude = float(data[15])
        pv_1.gps_age = float(data[16])
        pv_1.gps_altitude = float(data[17])
        pv_1.gps_speed_mps = float(data[18])
        pv_1.gps_speed_kmph = float(data[19])
        pv_1.gps_numsats = float(data[20].strip(", "))

        #pv_1.print_stats()

        if stop_threads:
            print("stop_threads = true")
            break

def send_telem_serial():                                            # send serial data from teensy
    while(True):

        global stop_threads
        global s
        global telem_send_time

        if time.time_ns() > telem_send_time:
            data = s + "\n"
            encoded_data = data.encode('utf-8')

            if (connected):
                telem.write(encoded_data)
                print("Sent: %s" % data)
            else:
                print("Telem not connected")

            telem_send_time = time.time_ns() + telem_send_period

        #time.sleep(0.1)

        if stop_threads:
            print("stop_threads = true")
            break

#global telem_serial_thread
telem_serial_thread = Thread(target=send_telem_serial, args=())
telem_serial_thread.daemon = True
telem_serial_thread.start()



teensy_serial_thread = Thread(target=get_teensy_serial, args=())
teensy_serial_thread.daemon = True
teensy_serial_thread.start()

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