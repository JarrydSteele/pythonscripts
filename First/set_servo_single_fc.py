# Import DroneKit-Python

from os import name
from dronekit import Vehicle, VehicleMode
import time

from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

from dk_vehicle import DKVehicle

mavutil.set_dialect("ardupilotmega")

MAVLINK20 = 1
global stop_threads
stop_threads = False                                                # variable used to cancel any threads

fc_1 = DKVehicle('127.0.0.1:14550')

@fc_1.vehicle.on_message('SERVO_OUTPUT_RAW')
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

def switch():
    option = input("Enter your option ('e' to exit): ")

    if option == 'e':
        print("Closing Vehicles and Exiting Program")
        fc_1.vehicle.close()
        # time.sleep(1)
        return
 
    elif option == '1':
        fc_1.printstats()
        switch()
 
    elif option == '2':
        fc_1.print_servos()
        switch()
    
    elif option == '3':
        for x in range(1,17):
            fc_1.override_servo(x,1800)
        fc_1.print_servos()
        switch()
    
    elif option == '4':
        for x in range(1,17):
            fc_1.override_servo(x,1500)
        fc_1.print_servos()
        switch()
    
    elif option == '5':
        for x in range(1,17):
            fc_1.enable_servo(x)
        fc_1.print_servos()

        switch()

    elif option == '6':
        fc_1.print_servo_functions()

        switch()

    elif option == '7':
        fc_1.print_frame_type()

        switch()

    elif option == '8':
        fc_1.set_frame_type(19)
        switch()

    elif option == '9':
                
        switch()
    
    elif option == '10':
        
        switch()

    elif option == 'k':
        stop_threads = True
        switch()
    
    elif option == 'qhover':
        fc_1.vehicle.wait_for_mode(VehicleMode('QHOVER'))
        fc_1.printstats()
        switch()
    
    elif option == 'qacro':
        fc_1.vehicle.wait_for_mode(VehicleMode('QACRO'))
        fc_1.printstats()
        switch()

    elif option == 'fbwa':
        fc_1.vehicle.wait_for_mode(VehicleMode('FBWA'))
        fc_1.printstats()
        switch()

    elif option == 'fbwb':
        fc_1.vehicle.wait_for_mode(VehicleMode('FBWB'))
        fc_1.printstats()
        switch()

    elif option == 'stabilize':
        fc_1.vehicle.wait_for_mode(VehicleMode('STABILIZE'))
        fc_1.printstats()
        switch()
        
    elif option == 'arm':
        fc_1.vehicle.armed = True
        switch()

    elif option == 'disarm':
        fc_1.vehicle.armed = False
        switch()

    else:
        print("Incorrect option")
        switch()

switch()


        