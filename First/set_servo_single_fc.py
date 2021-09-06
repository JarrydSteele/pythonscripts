# Import DroneKit-Python

from os import name
from dronekit import Vehicle, connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

from threading import Thread
import serial

mavutil.set_dialect("ardupilotmega")

MAVLINK20 = 1
global stop_threads
stop_threads = False                                                # variable used to cancel any threads

class DKVehicle:

    def __init__(self, connection):
        print ("Connecting to vehicle on: %s" % connection)
        self.vehicle = connect(connection, baud=57600, wait_ready=True)
        print ("Connected to vehicle on: %s" % connection)
        self.connection = connection

        # Force Dronekit to use Mavlink v2.0
        self.vehicle._master.first_byte = True

        self.servo_output = None
        self.servo_output = []
        for x in range(16):
            self.servo_output.append(0)

        self.actuator_output = None
        self.actuator_output = []
        for x in range(16):
            self.actuator_output.append(0)   
        
        self.servo_func = {
            '1': 4,             # Aileron
            '2': 19,            # Elevator
            '3': 70,            # Throttle
            '4': 21,            # Rudder
            '5': 33,            # Motor1 
            '6': 34,            # Motor2
            '7': 35,            # Motor3
            '8': 36,            # Motor4
            '9': 0,             # Disabled
            '10': 0,            # Disabled
            '11': 0,            # Disabled
            '12': 0,            # Disabled
            '13': 0,            # Disabled
            '14': 0,            # Disabled
            '15': 0,            # Disabled
            '16': 0,            # Disabled
            }
    
    def printstats(self):
        print ("Vehicle: %s" % self.vehicle.version)
        print (" Connected on: %s" % self.connection)
        print (" GPS: %s" % self.vehicle.gps_0)
        print (" Battery: %s" % self.vehicle.battery)
        print (" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print (" Is Armable?: %s" % self.vehicle.is_armable)
        print (" System status: %s" % self.vehicle.system_status.state)
        print (" Mode: %s" % self.vehicle.mode.name)
    
    def print_servos(self):
        self.servo_output
        print ("Servo1:  %s" % self.servo_output[0])
        print ("Servo2:  %s" % self.servo_output[1])
        print ("Servo3:  %s" % self.servo_output[2])
        print ("Servo4:  %s" % self.servo_output[3])
        print ("Servo5:  %s" % self.servo_output[4])
        print ("Servo6:  %s" % self.servo_output[5])
        print ("Servo7:  %s" % self.servo_output[6])
        print ("Servo8:  %s" % self.servo_output[7])
        print ("Servo9:  %s" % self.servo_output[8])
        print ("Servo10: %s" % self.servo_output[9])
        print ("Servo11: %s" % self.servo_output[10])
        print ("Servo12: %s" % self.servo_output[11])
        print ("Servo13: %s" % self.servo_output[12])
        print ("Servo14: %s" % self.servo_output[13])
        print ("Servo15: %s" % self.servo_output[14])
        print ("Servo16: %s" % self.servo_output[15])

    
    def print_channels(self):
        print(" Ch1: %s" % self.vehicle.channels['1'])
        print(" Ch2: %s" % self.vehicle.channels['2'])
        print(" Ch3: %s" % self.vehicle.channels['3'])
        print(" Ch4: %s" % self.vehicle.channels['4'])
        print(" Ch5: %s" % self.vehicle.channels['5'])
        print(" Ch6: %s" % self.vehicle.channels['6'])
        print(" Ch7: %s" % self.vehicle.channels['7'])
        print(" Ch8: %s" % self.vehicle.channels['8'])
    
    def override_servo(self, servo,val):
        servo_string = 'SERVO' + str(servo) + '_FUNCTION'
        self.vehicle.parameters[servo_string]=0
        
        msg = self.vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo, val, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        
        #print(msg)
        
        self.vehicle.flush()

    def disable_servo(self, servo):
        servo_string = 'SERVO' + str(servo) + '_FUNCTION'
        servo_trim = 'SERVO' + str(servo) + '_TRIM'
        self.vehicle.parameters[servo_string]=0
        val = self.vehicle.parameters[servo_trim]
        
        msg = self.vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo, val, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        
        #print(msg)
        
        self.vehicle.flush()

    def enable_servo(self, servo):
        servo_string = 'SERVO' + str(servo) + '_FUNCTION'
        servo_trim = 'SERVO' + str(servo) + '_TRIM'

        val = self.vehicle.parameters[servo_trim]
        if self.servo_func[str(servo)] == 0:
            val = 0

        msg = self.vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo, val, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        
        self.vehicle.parameters[servo_string]=self.servo_func[str(servo)]
        
        #print(msg)
        
        self.vehicle.flush()
    
    def print_servo_functions(self):
        for servo in range(1,17):
            servo_string = 'SERVO' + str(servo) + '_FUNCTION'
            print(servo_string + ': ' + str(self.vehicle.parameters[servo_string]))
        
fc_1 = DKVehicle('127.0.0.1:14550')

@fc_1.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    # print(message)
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
        fc_1.print_servos()
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
        
        switch()

    elif option == '8':
        
        switch()

    elif option == '9':
        
        switch()
    
    elif option == '10':
        
        switch()

    elif option == 'k':
        stop_threads = True
        switch()
    
    elif option == 'qhover':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('QHOVER')
        fc_1.printstats()
        switch()
    
    elif option == 'qacro':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('QACRO')
        fc_1.printstats()
        switch()

    elif option == 'fbwa':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('FBWA')
        fc_1.printstats()
        switch()

    elif option == 'fbwb':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('FBWB')
        fc_1.printstats()
        switch()

    elif option == 'stabilize':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('STABILIZE')
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


        