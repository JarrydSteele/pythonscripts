import subprocess
from threading import Thread
import time
#subprocess.run(['cd', 'python3 Documents/JarrydSteele/PythonScripts/pythonscripts/First/hello.py'])
print("Hello, World!")
#subprocess.run(['python3', 'hello.py'])
#print ("Hello, World!!")

stop_threads = False


def wait_10_seconds():
    #print ("Sleep for 10 seconds...")
    subprocess.run(['python3', 'hello.py'])
    #print ("Finished 10 Seconds")

def wait_15_seconds():
    print ("Sleep for 15 seconds...")
    time.sleep(15)
    print ("Finished 15 Seconds")

def telem_forward_1():
    print("Starting Telem Forwarding...")
    subprocess.run(['mavproxy.py', '--master=/dev/my_pixhawk4.1', '--out=/dev/my_radio2'])
    if stop_threads:
        exit()

def just_print_hi():
    while True:
        print("Hi")
        time.sleep(1)
        if stop_threads:
            break
    print("just_print_hi has been killed")

test_thread_1 = Thread(target=wait_10_seconds, args=())
test_thread_1.daemon = True
test_thread_1.start()

test_thread_2 = Thread(target=wait_15_seconds, args=())
test_thread_2.daemon = True
test_thread_2.start()

test_thread_3 = Thread(target=just_print_hi,args=())
test_thread_3.daemon = True
test_thread_3.start()

test_thread_1.join()
test_thread_2.join()

stop_threads = True
test_thread_3.join()


