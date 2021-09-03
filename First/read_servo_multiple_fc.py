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

def override_servo_1():                                            # get serial data from teensy
    while(True):
        pins= [1,2,3,4,5,6,7,8,9,10,11,12]
        msg=[]
        for x in pins:
            temp = fc_1.vehicle.message_factory.command_long_encode(
                0, 0,                                   # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,   #command
                0,                                      #confirmation
                x ,                                     # param 1, servo No
                1200,                                   # param 2, pwm
                0, 0, 0, 0, 0)                          # param 3 ~ 7 not used
            msg.append(temp)
    
        # send command to vehicle
        for i in msg:
            fc_1.vehicle.send_mavlink(i)
        
        if stop_threads:
            print("override_servo_1: stop_threads = true")
            break

def override_servo_2():                                            # get serial data from teensy
    while(True):
        pins= [1,2,3,4,5,6,7,8,9,10,11,12]
        msg=[]
        for x in pins:
            temp = fc_2.vehicle.message_factory.command_long_encode(
                0, 0,                                   # target_system, target_component
                183,   #command
                0,                                      #confirmation
                x ,                                     # param 1, servo No
                1200,                                   # param 2, pwm
                0, 0, 0, 0, 0)                          # param 3 ~ 7 not used
            msg.append(temp)
    
        # send command to vehicle
        for i in msg:
            fc_2.vehicle.send_mavlink(i)
        
        if stop_threads:
            print("override_servo_2: stop_threads = true")
            break

def switch():
    option = input("Enter your option ('e' to exit): ")

    if option == 'e':
        print("Closing Vehicles and Exiting Program")
        fc_1.vehicle.close()
        fc_2.vehicle.close()
        time.sleep(1)
        return
 
    elif option == '1':
        fc_1.printstats()
        fc_1.print_servos()
        switch()
    
    elif option == '2':
        fc_2.printstats()
        fc_2.print_servos()
        switch()
 
    elif option == '3':
        # fc_1.vehicle.channels.overrides['1'] = 200
        # fc_1.vehicle.channels.overrides['2'] = 200
        # fc_1.vehicle.channels.overrides['3'] = 200
        # fc_1.vehicle.channels.overrides['4'] = 200

        pins= [1,2,3,4,5,6,7,8,9,10,11,12]
        msg=[]
        for x in pins:
            temp = fc_1.vehicle.message_factory.command_long_encode(
                0, 0,                                   # target_system, target_component
                183,   #command
                0,                                      #confirmation
                x ,                                     # param 1, servo No
                1600,                                   # param 2, pwm
                0, 0, 0, 0, 0)                          # param 3 ~ 7 not used
            fc_1.vehicle.send_mavlink(temp)
            fc_1.vehicle.flush()
            print(temp)
            print("Pin %s changed to 1600" % x)
    
        # send command to vehicle

        fc_1.print_servos()

        switch()
    
    elif option == '4':
        # fc_1.vehicle.channels.overrides['1'] = 200
        # fc_1.vehicle.channels.overrides['2'] = 200
        # fc_1.vehicle.channels.overrides['3'] = 200
        # fc_1.vehicle.channels.overrides['4'] = 200

        pins= [1,2,3,4,5,6,7,8,9,10,11,12]
        msg=[]
        for x in pins:
            temp = fc_2.vehicle.message_factory.command_long_encode(
                0, 0,                                   # target_system, target_component
                183,   #command
                0,                                      #confirmation
                x ,                                     # param 1, servo No
                1600,                                   # param 2, pwm
                0, 0, 0, 0, 0)                          # param 3 ~ 7 not used
            msg.append(temp)
            print("Pin %s changed to 1600" % x)
    
        # send command to vehicle
        for i in msg:
            fc_2.vehicle.send_mavlink(i)
            print(i)

        fc_2.vehicle.flush()

        fc_2.print_servos()

        switch()
    
    elif option == '5':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('QHOVER')
        fc_1.printstats()
        switch()
    
    elif option == '6':
        fc_2.printstats()
        fc_2.vehicle.mode = VehicleMode('QHOVER')
        fc_2.printstats()
        switch()
    
    elif option == '7':
        fc_1.printstats()
        fc_1.vehicle.mode = VehicleMode('QACRO')
        fc_1.printstats()
        switch()
    
    elif option == '8':
        fc_2.printstats()
        fc_2.vehicle.mode = VehicleMode('QACRO')
        fc_2.printstats()
        switch()
    
    elif option == '9':
        global stop_threads
        stop_threads = False
        global override_servo_1_thread
        override_servo_1_thread = Thread(target=override_servo_1, args=())
        override_servo_1_thread.daemon = True
        override_servo_1_thread.start()
        switch()
    
    elif option == '10':
        stop_threads = False
        global override_servo_2_thread
        override_servo_2_thread = Thread(target=override_servo_2, args=())
        override_servo_2_thread.daemon = True
        override_servo_2_thread.start()
        switch()

    elif option == '11':
        #global fc_1
        msg = fc_1.vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1500, 0, 0, 0, 0, 0)
        fc_1.vehicle.send_mavlink(msg)
        fc_1.vehicle.flush()

        switch()

    elif option == 'k':
        stop_threads = True
        switch()

    else:
        print("Incorrect option")
        switch()

switch()


        