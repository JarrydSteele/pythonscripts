# Import DroneKit-Python

from os import name
from dronekit import Vehicle, connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

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
                
        
fc_1 = DKVehicle('/dev/pixhawk4.1')
fc_2 = DKVehicle('/dev/pixhawk4.2')

#fc_1.printstats()
#fc_2.printstats()

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
    #print("FC_1")
    #fc_1.print_servos()

@fc_2.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    # print(message)
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
    #print("FC_2")
    #fc_2.print_servos()

# @fc_1.vehicle.on_message('SERVO_OUTPUT_RAW')
# def listener1(self, name, message):
#     print("Vehicle 1: %s" % message);   

# @fc_2.vehicle.on_message('SERVO_OUTPUT_RAW')
# def listener2(self, name, message):
#     print("Vehicle 2: %s" % message);   

#user_input = input("State a command: ")

def switch():
    option = input("Enter your option ('e' to exit): ")

    if option == 'e':
        print("Closing Vehicles and Exiting Program")
        fc_1.vehicle.close()
        fc_2.vehicle.close()
        time.sleep(2)
        return
 
    elif option == '1':
        fc_1.printstats()
        fc_1.print_servos()
        switch()
 
    elif option == '2':
        fc_2.printstats()
        fc_2.print_servos()
        switch()
 
    else:
        print("Incorrect option")
        switch()

switch()


        