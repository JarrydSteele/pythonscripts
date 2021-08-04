from io import StringIO
import sys
import time
import random

# Import PyQt5

from PyQt5 import QtWidgets, uic
from readservosGUI import *


servo = []

for x in range(16):
    servo.append(0)

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

    print ('Connect Button Pressed')
    window.ui.btnConnect.setText("Connecting")
    window.ui.btnConnect.repaint()

    time.sleep(5)

    print ('Connected')
    window.ui.btnConnect.setText("Connected")

    RandomSliders()

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
    
    ZeroSliders()

    window.ui.btnConnect.setText("Connect")

    # Shut down simulator
    print ("Completed")

def MakeDo():
    RandomSliders()

def RandomSliders():
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
    window.ui.horizontalSlider_33.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_34.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_35.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_36.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_37.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_38.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_39.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_40.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_41.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_42.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_43.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_44.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_45.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_46.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_47.setSliderPosition(random.randint(0,100))
    window.ui.horizontalSlider_48.setSliderPosition(random.randint(0,100))

def ZeroSliders():
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
    window.ui.horizontalSlider_33.setSliderPosition(0)
    window.ui.horizontalSlider_34.setSliderPosition(0)
    window.ui.horizontalSlider_35.setSliderPosition(0)
    window.ui.horizontalSlider_36.setSliderPosition(0)
    window.ui.horizontalSlider_37.setSliderPosition(0)
    window.ui.horizontalSlider_38.setSliderPosition(0)
    window.ui.horizontalSlider_39.setSliderPosition(0)
    window.ui.horizontalSlider_40.setSliderPosition(0)
    window.ui.horizontalSlider_41.setSliderPosition(0)
    window.ui.horizontalSlider_42.setSliderPosition(0)
    window.ui.horizontalSlider_43.setSliderPosition(0)
    window.ui.horizontalSlider_44.setSliderPosition(0)
    window.ui.horizontalSlider_45.setSliderPosition(0)
    window.ui.horizontalSlider_46.setSliderPosition(0)
    window.ui.horizontalSlider_47.setSliderPosition(0)
    window.ui.horizontalSlider_48.setSliderPosition(0)

def QuitProgram():
    # Shut down simulator
    print ("Quit")
    quit()
    
app = QtWidgets.QApplication(sys.argv)
window = Main()
app.exec_()