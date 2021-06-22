from io import StringIO
import sys
import time
import random

# Import PyQt5

from PyQt5 import QtWidgets, uic
from readservosGUI import *

# Import DroneKit-Python

#from dronekit import Vehicle, connect, VehicleMode, Command
from dronekit import *
#import dronekit
from pymavlink.dialects.v20 import *
from pymavlink import mavutil

MAVLINK_DIALECT = "ardupilotmega"
MAVLINK20 = 1


mavutil.set_dialect("ardupilotmega")

servo = []

for x in range(16):
    servo.append(0)

vehicle = None

class Main(QtWidgets.QMainWindow):
    def __init__(self):
        super(QtWidgets.QMainWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.btnConnect.clicked.connect(ConnectPressed)
        self.ui.btnDisconnect.clicked.connect(DisconnectPressed)
        self.ui.btnMakeDo.clicked.connect(MakeDo)
        self.ui.btnQuit.clicked.connect(QuitProgram)

        self.show()

def PrintServos():
    print ("Servo1:  %s" % servo[0])
    print ("Servo2:  %s" % servo[1])
    print ("Servo3:  %s" % servo[2])
    print ("Servo4:  %s" % servo[3])
    print ("Servo5:  %s" % servo[4])
    print ("Servo6:  %s" % servo[5])
    print ("Servo7:  %s" % servo[6])
    print ("Servo8:  %s" % servo[7])
    print ("Servo9:  %s" % servo[8])
    print ("Servo10: %s" % servo[9])
    print ("Servo11: %s" % servo[10])
    print ("Servo12: %s" % servo[11])
    print ("Servo13: %s" % servo[12])
    print ("Servo14: %s" % servo[13])
    print ("Servo15: %s" % servo[14])
    print ("Servo16: %s" % servo[15])

def ConnectPressed():   
    global vehicle

    print ('Connect Button Pressed')
    
    window.ui.btnConnect.setText("Connecting")


    #connection = '/dev/my_radio1'       #'/dev/my_radio1' '/dev/my_pixhawk'
    if window.ui.cmbConnect.currentText() == "Pixhawk":
        connection = '/dev/my_pixhawk'
    elif window.ui.cmbConnect.currentText() == "Radio1":
        connection = '/dev/my_radio1'
    elif window.ui.cmbConnect.currentText() == "Radio2":
        connection = '/dev/my_radio2'

    
    #connection = '/dev/my_radio1'       #'/dev/my_radio1' '/dev/my_pixhawk'

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
        servo[0] = int(message.servo1_raw)/20
        servo[1] = int(message.servo2_raw)/20
        servo[2] = int(message.servo3_raw)/20
        servo[3] = int(message.servo4_raw)/20
        servo[4] = int(message.servo5_raw)/20
        servo[5] = int(message.servo6_raw)/20
        servo[6] = int(message.servo7_raw)/20
        servo[7] = int(message.servo8_raw)/20
        servo[8] = int(message.servo9_raw)/20
        servo[9] = int(message.servo10_raw)/20
        servo[10] = int(message.servo11_raw)/20
        servo[11] = int(message.servo12_raw)/20
        servo[12] = int(message.servo13_raw)/20
        servo[13] = int(message.servo14_raw)/20
        servo[14] = int(message.servo15_raw)/20
        servo[15] = int(message.servo16_raw)/20
        #PrintServos()
        UpdateServoSliders()
        
    window.ui.btnConnect.setText("Connected")

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

def UpdateServoSliders():
    window.ui.horizontalSlider_1.setSliderPosition(int(servo[0]))
    window.ui.horizontalSlider_2.setSliderPosition(int(servo[1]))
    window.ui.horizontalSlider_3.setSliderPosition(int(servo[2]))
    window.ui.horizontalSlider_4.setSliderPosition(int(servo[3]))
    window.ui.horizontalSlider_5.setSliderPosition(int(servo[4]))
    window.ui.horizontalSlider_6.setSliderPosition(int(servo[5]))
    window.ui.horizontalSlider_7.setSliderPosition(int(servo[6]))
    window.ui.horizontalSlider_8.setSliderPosition(int(servo[7]))
    window.ui.horizontalSlider_9.setSliderPosition(int(servo[8]))
    window.ui.horizontalSlider_10.setSliderPosition(int(servo[9]))
    window.ui.horizontalSlider_11.setSliderPosition(int(servo[10]))
    window.ui.horizontalSlider_12.setSliderPosition(int(servo[11]))
    window.ui.horizontalSlider_13.setSliderPosition(int(servo[12]))
    window.ui.horizontalSlider_14.setSliderPosition(int(servo[13]))
    window.ui.horizontalSlider_15.setSliderPosition(int(servo[14]))
    window.ui.horizontalSlider_16.setSliderPosition(int(servo[15]))


def DisconnectPressed():
    
    window.ui.horizontalSlider_1.setSliderPosition(0)
    window.ui.horizontalSlider_2.setSliderPosition(0)
    window.ui.horizontalSlider_3.setSliderPosition(0)
    window.ui.horizontalSlider_4.setSliderPosition(0)
    window.ui.horizontalSlider_5.setSliderPosition(0)
    window.ui.horizontalSlider_6.setSliderPosition(0)
    window.ui.horizontalSlider_7.setSliderPosition(0)
    window.ui.horizontalSlider_8.setSliderPosition(0)
    window.ui.horizontalSlider_9.setSliderPosition(0)
    window.ui.horizontalSlider_10.setSliderPosition(0)
    window.ui.horizontalSlider_11.setSliderPosition(0)
    window.ui.horizontalSlider_12.setSliderPosition(0)
    window.ui.horizontalSlider_13.setSliderPosition(0)
    window.ui.horizontalSlider_14.setSliderPosition(0)
    window.ui.horizontalSlider_15.setSliderPosition(0)
    window.ui.horizontalSlider_16.setSliderPosition(0)

    window.ui.btnConnect.setText("Connect")
    
    vehicle.close()

    # Shut down simulator
    print ("Completed")

def MakeDo():
    window.ui.horizontalSlider_1.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_2.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_3.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_4.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_5.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_6.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_7.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_8.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_9.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_10.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_11.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_12.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_13.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_14.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_15.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_16.setSliderPosition(random.randint(0,100))

def QuitProgram():
    # Shut down simulator
    print ("Quit")
    quit()
    
app = QtWidgets.QApplication(sys.argv)
window = Main()
app.exec_()