# Import DroneKit-Python

from dronekit import Vehicle, connect, VehicleMode, Command
import time
import dronekit
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

servo = []

for x in range(8):
    servo.append(0)

class dk_vehicle(Vehicle):

    def __init__(self, connection):
        pass

    def print_stats(self):
        print ("Vehicle connected on: %s" % self.connection)
        print (" GPS: %s" % self.gps_0)
        print (" Battery: %s" % fc_1.battery)
        print (" Last Heartbeat: %s" % fc_1.last_heartbeat)
        print (" Is Armable?: %s" % fc_1.is_armable)
        print (" System status: %s" % fc_1.system_status.state)
        print (" Mode: %s" % fc_1.mode.name)
        
connection1 = "/dev/my_pixhawk4.1"
connection2 = "/dev/my_pixhawk4.2"

fc1 = dk_vehicle(connection1)
fc2 = dk_vehicle(connection2)

print ("Connecting to vehicle on: %s" % connection1)        
fc_1 = connect(connection1, baud=57600, wait_ready=True)
print ("Connected to vehicle on: %s" % connection1)
print ("Connecting to vehicle on: %s" % connection2)
fc_2 = connect(connection2, baud=57600, wait_ready=True)
print ("Connected to vehicle on: %s" % connection1)

# Force Dronekit to use Mavlink v2.0
fc_1._master.first_byte = True

fc_1.print_stats()
fc_2.print_stats()

time.sleep(2)

fc_1.vehicle.close()
fc_2.vehicle.close()
        