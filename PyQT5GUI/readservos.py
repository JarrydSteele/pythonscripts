# Import DroneKit-Python

# MAVLINK20 = 1

# from dronekit import connect, VehicleMode, Command
# import time
# import dronekit
# from pymavlink.dialects.v20 import ardupilotmega
# from pymavlink import mavutil

# mavutil.set_dialect("ardupilotmega")

from readservosGUI import Ui_MainWindow
import sys
#from readservosGUI import *

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget

class Main(QMainWindow):
    def __init__(self):
        super(Main, self).__init__()
        
        self.show()
    
    def authenticate(self):
        print ("Connect Clicked")
        

if __name__ == "__main__":
     
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    window = Main(Ui_MainWindow)
    window.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())