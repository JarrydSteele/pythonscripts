# Import DroneKit-Python

from dronekit import connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

servo = []

for x in range(8):
    servo.append(0)

connection = '/dev/my_pixhawk4.1'       #'/dev/my_radio' '/dev/my_pixhawk'

def PrintServos():
    print ("Servo1:  %s" % servo[0])
    print ("Servo2:  %s" % servo[1])
    print ("Servo3:  %s" % servo[2])
    print ("Servo4:  %s" % servo[3])
    print ("Servo5:  %s" % servo[4])
    print ("Servo6:  %s" % servo[5])
    print ("Servo7:  %s" % servo[6])
    print ("Servo8:  %s" % servo[7])
    # print ("Servo9:  %s" % servo[8])
    # print ("Servo10: %s" % servo[9])
    # print ("Servo11: %s" % servo[10])
    # print ("Servo12: %s" % servo[11])
    # print ("Servo13: %s" % servo[12])
    # print ("Servo14: %s" % servo[13])
    # print ("Servo15: %s" % servo[14])
    # print ("Servo16: %s" % servo[15])

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

    
# @vehicle.on_message('ACTUATOR_OUTPUT_STATUS')
# def listener(self, name, message):
#     print(message);   

@vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    print(message);   

# @vehicle.on_message('*')
# def listener(self, name, message):
#     print (message)

time.sleep(2)

vehicle.close()

# Shut down simulator
print ("Completed")
