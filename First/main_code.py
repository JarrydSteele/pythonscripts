# Import DroneKit-Python

from os import name
from dronekit import Vehicle, connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

from threading import Thread
import serial
from physical_vehicle import PhysicalVehicle
from dk_vehicle import DKVehicle

mavutil.set_dialect("ardupilotmega")

stop_threads = False                                                # variable used to cancel any threads     

fc_1 = DKVehicle('/dev/pixhawk4.1')                                 # initialise dronekit vehicle 1
fc_2 = DKVehicle('/dev/pixhawk4.2')                                 # initialise dronekit vehicle 1

pv_1 = PhysicalVehicle()


print("Connecting to Teensy")
try:
    teensy = serial.Serial('/dev/teensy4.1')                            # open serial port
    print("Connected to: %s " % teensy.name)                            # check which port was really used
except:
    print("Could not connect to Teensy")


try:
    @fc_1.vehicle.on_message('SERVO_OUTPUT_RAW')                        # dronekit vehicle 1 servo message listener
    def listener(self, name, message):
        fc_1.servo_output[0] = message.servo1_raw
        fc_1.servo_output[1] = message.servo2_raw
        fc_1.servo_output[2] = message.servo3_raw
        fc_1.servo_output[3] = message.servo4_raw
        fc_1.servo_output[4] = message.servo5_raw
        fc_1.servo_output[5] = message.servo6_raw
        fc_1.servo_output[6] = message.servo7_raw
        fc_1.servo_output[7] = message.servo8_raw
        fc_1.servo_output[8] = message.servo9_raw
        fc_1.servo_output[9] = message.servo10_raw
        fc_1.servo_output[10] = message.servo11_raw
        fc_1.servo_output[11] = message.servo12_raw
        fc_1.servo_output[12] = message.servo13_raw
        fc_1.servo_output[13] = message.servo14_raw
        fc_1.servo_output[14] = message.servo15_raw
        fc_1.servo_output[15] = message.servo16_raw
except:
    print("Vehicle: 'fc_1' not found")

try:
    @fc_2.vehicle.on_message('SERVO_OUTPUT_RAW')                        # dronekit vehicle 1 servo message listener
    def listener(self, name, message):
        fc_2.servo_output[0] = message.servo1_raw
        fc_2.servo_output[1] = message.servo2_raw
        fc_2.servo_output[2] = message.servo3_raw
        fc_2.servo_output[3] = message.servo4_raw
        fc_2.servo_output[4] = message.servo5_raw
        fc_2.servo_output[5] = message.servo6_raw
        fc_2.servo_output[6] = message.servo7_raw
        fc_2.servo_output[7] = message.servo8_raw
        fc_2.servo_output[8] = message.servo9_raw
        fc_2.servo_output[9] = message.servo10_raw
        fc_2.servo_output[10] = message.servo11_raw
        fc_2.servo_output[11] = message.servo12_raw
        fc_2.servo_output[12] = message.servo13_raw
        fc_2.servo_output[13] = message.servo14_raw
        fc_2.servo_output[14] = message.servo15_raw
        fc_2.servo_output[15] = message.servo16_raw
except:
    print("Vehicle: 'fc_2' not found")

def get_teensy_serial():                                            # get serial data from teensy
    while(True):
        global s
        global int_s
        global stop_threads
        
        s = teensy.readline().decode().strip()
        int_s = int(teensy.readline().decode().strip())
        
        #print(int_s)
        if stop_threads:
            print("stop_threads = true")
            break

global teensy_serial_thread
teensy_serial_thread = Thread(target=get_teensy_serial, args=())
teensy_serial_thread.daemon = True
teensy_serial_thread.start()

def switch():

    option = input("Enter your option ('e' to exit): ")

    if option == 'e':
        print("Closing Vehicles and Exiting Program")
        fc_1.vehicle.close()
        fc_2.vehicle.close()
        teensy.close()
        time.sleep(2)
        return
 
    elif option == '1-stats':
        fc_1.printstats()
        fc_1.print_servos()
        switch()
 
    elif option == '2-stats':
        fc_2.printstats()
        fc_2.print_servos()
        switch()

    elif option == 'teensy':
        print(int_s)
        switch()

    elif option == 'kill':
        print("Stopping all threads")
        global stop_threads
        stop_threads = True
        teensy_serial_thread.join()
        switch()
    
    elif option == '1-qhover':
        fc_1.printstats()
        fc_1.vehicle.wait_for_mode(VehicleMode('QHOVER'))
        fc_1.printstats()
        switch()
    
    elif option == '1-qacro':
        fc_1.printstats()
        fc_1.vehicle.wait_for_mode(VehicleMode('QACRO'))
        fc_1.printstats()
        switch()

    elif option == '1-fbwa':
        fc_1.printstats()
        fc_1.vehicle.wait_for_mode(VehicleMode('FBWA'))
        fc_1.printstats()
        switch()

    elif option == '1-fbwb':
        fc_1.printstats()
        fc_1.vehicle.wait_for_mode(VehicleMode('FBWB'))
        fc_1.printstats()
        switch()

    elif option == '1-stabilize':
        fc_1.printstats()
        fc_1.vehicle.wait_for_mode(VehicleMode('STABILIZE'))
        fc_1.printstats()
        switch()
        
    elif option == '1-arm':
        fc_1.vehicle.armed = True
        switch()

    elif option == '1-disarm':
        fc_1.vehicle.armed = False
        switch()

    elif option == '2-qhover':
        fc_2.printstats()
        fc_2.vehicle.wait_for_mode(VehicleMode('QHOVER'))
        fc_2.printstats()
        switch()
    
    elif option == '2-qacro':
        fc_2.printstats()
        fc_2.vehicle.wait_for_mode(VehicleMode('QACRO'))
        fc_2.printstats()
        switch()

    elif option == '2-fbwa':
        fc_2.printstats()
        fc_2.vehicle.wait_for_mode(VehicleMode('FBWA'))
        fc_2.printstats()
        switch()

    elif option == '2-fbwb':
        fc_2.printstats()
        fc_2.vehicle.wait_for_mode(VehicleMode('FBWB'))
        fc_2.printstats()
        switch()

    elif option == '2-stabilize':
        fc_2.printstats()
        fc_2.vehicle.wait_for_mode(VehicleMode('STABILIZE'))
        fc_2.printstats()
        switch()
        
    elif option == '2-arm':
        fc_2.vehicle.armed = True
        switch()

    elif option == '2-disarm':
        fc_2.vehicle.armed = False
        switch()
 
    else:
        print("Incorrect option")
        switch()

switch()


        