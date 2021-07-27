# Import DroneKit-Python

from dronekit import connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

servo = []

for x in range(8):
    servo.append(0)

class DKVehicle:
    def __init__(self, connection):
        print ("Connecting to vehicle on: %s" % connection)
        self.vehicle = connect(connection, baud=57600, wait_ready=True)

        # Force Dronekit to use Mavlink v2.0
        self.vehicle._master.first_byte = True

        print ("Vehicle connected on: %s" %connection)
        print (" GPS: %s" % self.vehicle.gps_0)
        print (" Battery: %s" % self.vehicle.battery)
        print (" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print (" Is Armable?: %s" % self.vehicle.is_armable)
        print (" System status: %s" % self.vehicle.system_status.state)
        print (" Mode: %s" % self.vehicle.mode.name)
        
fc_1 = DKVehicle('/dev/my_pixhawk4.1')
fc_2 = DKVehicle('/dev/my_pixhawk4.2')

time.sleep(2)

fc_1.vehicle.close()
fc_2.vehicle.close()
        