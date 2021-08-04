from io import StringIO
import sys
import time
import random
import os

# Import DroneKit-Python



from dronekit import Vehicle, connect, VehicleMode, Command
import pymavlink
from pymavlink.dialects.v20 import *
from pymavlink import mavutil

MAVLINK20 = 1

print("Using Dialect 20: %s" % mavutil.mavlink20())

servo = []

for x in range(16):
    servo.append(0)

vehicle = None
    
connection = '/dev/ttyACM0'       #'/dev/radio' '/dev/pixhawk'

print ("Start simulator (HITL)")

# Connect to the Vehicle. '/dev/ttyACM0'
print ("Connecting to vehicle on: %s" % connection)

vehicle = connect(connection, baud=57600, wait_ready=True)

# Force Dronekit to use Mavlink v2.0
vehicle._master.first_byte = True

# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print (" GPS: %s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
print (" Is Armable?: %s" % vehicle.is_armable)
print (" System status: %s" % vehicle.system_status.state)
print (" Mode: %s" % vehicle.mode.name)  # settable


@vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    #print(message)
    servo[0] = int(message.servo1_raw)
    servo[1] = int(message.servo2_raw)
    servo[2] = int(message.servo3_raw)
    servo[3] = int(message.servo4_raw)
    servo[4] = int(message.servo5_raw)
    servo[5] = int(message.servo6_raw)
    servo[6] = int(message.servo7_raw)
    servo[7] = int(message.servo8_raw)
    servo[0] = int(message.servo9_raw)
    servo[9] = int(message.servo10_raw)
    servo[10] = int(message.servo11_raw)
    servo[11] = int(message.servo12_raw)
    servo[12] = int(message.servo13_raw)
    servo[13] = int(message.servo14_raw)
    servo[14] = int(message.servo15_raw)
    servo[15] = int(message.servo16_raw)
    PrintServos()

def PrintServos():

    print ("Servo 1:   %s" % servo[0])
    print ("Servo 2:   %s" % servo[1])
    print ("Servo 3:   %s" % servo[2])
    print ("Servo 4:   %s" % servo[3])
    print ("Servo 5:   %s" % servo[4])
    print ("Servo 6:   %s" % servo[5])
    print ("Servo 7:   %s" % servo[6])
    print ("Servo 8:   %s" % servo[7])
    print ("Servo 9:   %s" % servo[8])
    print ("Servo 10:  %s" % servo[9])
    print ("Servo 11:  %s" % servo[10])
    print ("Servo 12:  %s" % servo[11])
    print ("Servo 13:  %s" % servo[12])
    print ("Servo 14:  %s" % servo[13])
    print ("Servo 15:  %s" % servo[14])
    print ("Servo 16:  %s" % servo[15])

time.sleep(3)

vehicle.close()

print("Using Dialect 20: %s" % mavutil.mavlink20())

# Shut down simulator
print ("Completed")