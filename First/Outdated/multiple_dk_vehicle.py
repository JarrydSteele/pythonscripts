# Import DroneKit-Python

from dronekit import CommandSequence, Vehicle, connect, VehicleMode, Command
import time
import dronekit
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

servo = []

for x in range(8):
    servo.append(0)

class dk_vehicle(Vehicle):
   
    def __init__(self):
        pass

    def dk_connect(self, connection):
        self.name = connection
        return connect(connection, baud=57600, wait_ready=True)

    def print_stats(self):
        print ("Vehicle connected on: %s" % self.name)
        print (" GPS: %s" % self.gps_0)
        print (" Battery: %s" % self.battery)
        print (" Last Heartbeat: %s" % self.last_heartbeat)
        print (" Is Armable?: %s" % self.is_armable)
        print (" System status: %s" % self.system_status.state)
        print (" Mode: %s" % self.mode.name)
        
connection1 = "/dev/pixhawk4.1"
connection2 = "/dev/pixhawk4.2"

fc_1 = dk_vehicle

dk_vehicle.dk_connect(fc_1, connection1)
#fc_2 = dk_vehicle(connection2)

print ("Connecting to vehicle on: %s" % connection1)        
#fc_1 = connect(connection1, baud=57600, wait_ready=True)
print ("Connected to vehicle on: %s" % connection1)

#print ("Connecting to vehicle on: %s" % connection2)
#fc_2 = connect(connection2, baud=57600, wait_ready=True)
#print ("Connected to vehicle on: %s" % connection2)

# Force Dronekit to use Mavlink v2.0
#fc_1._master.first_byte = True

dk_vehicle.print_stats(fc_1)
#dk_vehicle.print_stats(fc_2)


time.sleep(2)

fc_1.close()
#fc_2.close()
        