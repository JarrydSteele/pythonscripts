from dronekit import Vehicle, connect, VehicleMode, Command
import time
from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil

class DKVehicle(Vehicle):

    def __init__(self, connection):
        print ("Connecting to vehicle on: %s" % connection)
        self.vehicle = connect(connection, baud=57600, wait_ready=True)
        print ("Connected to vehicle on: %s" % connection)
        self.connection = connection

        # Force Dronekit to use Mavlink v2.0
        self.vehicle._master.first_byte = True

        self.servo_output = []
        for x in range(17):
            self.servo_output.append(0)
        
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
        for x in range(0,16):
            print("Servo%s:  %s" % (x, self.servo_output[x]))
    
    def print_servo(self, servo):
        print("Servo%s:  %s" % (servo, self.servo_output[servo]))
    
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
        
        self.vehicle.flush()

    def disable_servo(self, servo):
        servo_string = 'SERVO' + str(servo) + '_FUNCTION'
        servo_trim = 'SERVO' + str(servo) + '_TRIM'
        self.vehicle.parameters[servo_string]=0
        val = self.vehicle.parameters[servo_trim]
        
        msg = self.vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo, val, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        
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
        
        self.vehicle.flush()
    
    def print_servo_functions(self):
        for servo in range(1,17):
            servo_string = 'SERVO' + str(servo) + '_FUNCTION'
            print(servo_string + ': ' + str(self.vehicle.parameters[servo_string]))

    def print_frame_type(self):
        print("Q_ENABLE: " + str(self.vehicle.parameters['Q_ENABLE']))
        print("Q_FRAME_TYPE: " + str(self.vehicle.parameters['Q_FRAME_TYPE']))
    
    def set_frame_type(self,frame):
        self.vehicle.parameters['Q_FRAME_TYPE'] = frame
    
    