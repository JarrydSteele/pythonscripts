import serial
import time

from threading import Thread

stop_threads = False

def get_teensy_serial():
    print("Starting Telem Forwarding...")
    global teensy 
    teensy = serial.Serial('/dev/teensy4.1')       # open serial port
    print(teensy.name)                             # check which port was really used
    while(True):
        global s
        global int_s
        
        s = teensy.readline().decode().strip()
        int_s = int(teensy.readline().decode().strip())
        #print(int_s)
        if stop_threads:
            break

def switch():
    option = input("Enter Your Option ('e' to Exit): ")

    if option == 'e':
        print("Closing Vehicles and Exiting Program")
        time.sleep(2)
        return
 
    elif option == '1':
        switch()
 
    elif option == '2':
        switch()

    elif option == '3':
        print(int_s)
        switch()
    
    elif option == '4':
        print("Starting Serial Thread")
        test_thread_1 = Thread(target=get_teensy_serial, args=())
        test_thread_1.daemon = True
        test_thread_1.start()
        switch()

    elif option == '5':
        print("Stopping All Threads")
        stop_threads = True
        teensy.close()
        switch()
 
    else:
        print("Incorrect Option")
        switch()

switch()